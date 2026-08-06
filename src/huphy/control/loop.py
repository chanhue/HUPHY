"""제어 루프 — 정해진 주기로 한 사이클씩 돌림.

한 사이클이 하는 일.

    1  목표를 정함              동작 제공자(`Motion`)가 냄
    2  계산                     로봇이 관절 -> 모터 -> 프레임으로 풂
    3  전송
    4  수거                     상태 갱신
    5  기록                     텔레메트리
    6  다음 주기까지 기다림

**루프는 무엇을 시킬지 모름.** 그건 `Motion` 이 정하고, 루프는 주기와 안전만 봄.
덕분에 같은 루프로 유지, 사인파 흔들기, 보행 궤적을 다 돌릴 수 있음.


## 실제 주기를 잼

목표 주기와 실제 주기는 다름. 계산이 길어지거나 수거가 늦어지면 밀림.

**밀렸을 때 조용히 넘어가면 안 됨.** 느려진 것을 아무도 모른 채로 게인을 튜닝하면,
게인이 아니라 주기가 문제인데 게인을 계속 만지게 됨.

`loop_dt` 를 매 주기 텔레메트리로 내보내고, 목표를 크게 넘긴 주기는 세어 둠.


## 두 가지 모드

    OBSERVE   토크를 끄고 상태만 읽음
    CONTROL   명령을 보냄

`OBSERVE` 는 **들어갈 때 토크를 끊음.** 설정 실수로 명령이 나가는 것을 막으려는
것임 — 관찰하려고 켰는데 다리가 움직이면 안 됨.


## 멈출 때

    1  게인을 유지한 채 현재 자세를 목표로 (`hold`)
    2  토크 차단
    3  텔레메트리 flush

**바로 토크를 끊지 않음.** 서 있는 다리에서 힘이 갑자기 빠지면 주저앉음.
`hold` 를 몇 주기 보내 자세를 붙잡은 뒤 끊음.

예외로 빠져나가도 이 순서를 탐.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, Optional

from ..robots.base import Action, Robot

logger = logging.getLogger(__name__)

OVERRUN_RATIO = 1.5
"""목표 주기의 이 배를 넘으면 밀린 것으로 셈. 100Hz 에서 15ms."""

SETTLE_CYCLES = 5
"""멈출 때 `hold` 를 보내는 주기 수. 힘이 갑자기 빠지지 않게 함."""


class Mode(str, Enum):
    """루프가 무엇을 하는지.

    `OBSERVE` 는 명령을 아예 만들지 않음. 계산을 건너뛰는 것이 아니라 **토크를
    끊고 시작함** — 설정이 잘못돼 있어도 다리가 안 움직이게 하려는 것임.
    """

    OBSERVE = "observe"
    CONTROL = "control"


Motion = Callable[[float, Dict[str, Any]], Optional[Action]]
"""무엇을 시킬지 정하는 함수.

    (경과 시간 초, 지금 관찰) -> 관절 목표  또는  None

`None` 을 내면 그 주기는 명령을 보내지 않음. 궤적이 끝났거나 아직 시작 전일 때 씀.
"""


@dataclass
class LoopStats:
    """돌린 결과. 끝나고 사람이 보는 값임."""

    cycles: int = 0
    overruns: int = 0
    """목표 주기를 크게 넘긴 횟수."""

    worst_dt_ms: float = 0.0
    total_s: float = 0.0
    missing_cycles: int = 0
    """무응답 모터가 하나라도 있었던 주기 수."""

    target_hz: float = 0.0
    """목표 주기. 실제와 대조하는 데 씀."""

    @property
    def mean_hz(self) -> float:
        return self.cycles / self.total_s if self.total_s > 0 else 0.0

    @property
    def kept_up(self) -> bool:
        """목표 주기를 대체로 지켰는지. 90% 미만이면 못 지킨 것으로 봄.

        `overruns` 는 튀는 주기를 세지만, **꾸준히 느린 것은 못 잡음** -- 매 주기
        24% 씩 넘으면 한 번도 "밀림" 으로 세지 않으면서 주파수만 떨어짐.
        """
        return self.target_hz <= 0 or self.mean_hz >= self.target_hz * 0.9

    def summary(self) -> str:
        target = f" / 목표 {self.target_hz:.0f}Hz" if self.target_hz else ""
        warn = "" if self.kept_up else "  ** 주기를 못 지킴 **"
        return (
            f"{self.cycles}주기 {self.total_s:.1f}초 "
            f"(평균 {self.mean_hz:.1f}Hz{target}), "
            f"밀림 {self.overruns}회, 최악 {self.worst_dt_ms:.1f}ms, "
            f"무응답 주기 {self.missing_cycles}회{warn}"
        )


class ControlLoop:
    """로봇 하나를 정해진 주기로 돌림.

    텔레메트리는 선택임 — 없으면 기록만 안 하고 나머지는 같음.
    """

    def __init__(
        self,
        robot: Robot,
        *,
        hz: float = 100.0,
        telemetry: Any = None,
        mode: Mode = Mode.OBSERVE,
    ) -> None:
        if hz <= 0:
            raise ValueError(f"hz 는 0보다 커야 함 (받은 값 {hz})")
        self.robot = robot
        self.hz = float(hz)
        self.period_s = 1.0 / float(hz)
        self.telemetry = telemetry
        self.mode = mode
        self.stats = LoopStats(target_hz=self.hz)
        self._stop = False

    def __repr__(self) -> str:
        return f"ControlLoop({self.robot.id}, {self.hz:.0f}Hz, {self.mode.value})"

    def stop(self) -> None:
        """다음 주기에 빠져나오게 함. 다른 스레드나 시그널 처리기에서 부름."""
        self._stop = True

    # ---- 한 주기 ----------------------------------------------------------
    def step(self, motion: Optional[Motion], t: float) -> Dict[str, Any]:
        """한 사이클. 관찰을 돌려줌.

        **기다리지 않음** — 주기 유지는 `run` 이 함. 한 사이클만 떼어 시험하거나
        대화형으로 한 걸음씩 돌릴 때 이것을 씀.
        """
        observation = self.robot.get_observation()

        if self.mode is Mode.OBSERVE:
            # 아무것도 보내지 않으면 아무것도 오지 않음. MIT 모드에는 읽기 전용
            # 명령이 없어서, 힘이 나가지 않는 명령을 보내고 그 응답을 받음.
            missing = self.robot.refresh()
        else:
            action = motion(t, observation) if motion is not None else None
            if action:
                self.robot.send(self.robot.build_commands(action))
            missing = self.robot.collect()

        if missing:
            self.stats.missing_cycles += 1
        return observation

    # ---- 계속 돌림 ---------------------------------------------------------
    def run(
        self,
        motion: Optional[Motion] = None,
        *,
        duration_s: Optional[float] = None,
        max_cycles: Optional[int] = None,
    ) -> LoopStats:
        """멈출 때까지 돌림.

        `duration_s` 나 `max_cycles` 를 주면 그만큼만 돌고 나옴. 둘 다 없으면
        `stop()` 이 불릴 때까지 돎.

        **끝나는 경로가 하나임.** 정상 종료든 예외든 `finally` 를 지나며 자세를
        붙잡고 토크를 끊음.
        """
        self._stop = False
        self._enter()

        t0 = time.monotonic()
        previous = t0

        try:
            while not self._stop:
                cycle_start = time.monotonic()
                t = cycle_start - t0

                if duration_s is not None and t >= duration_s:
                    break
                if max_cycles is not None and self.stats.cycles >= max_cycles:
                    break

                dt_ms = (cycle_start - previous) * 1000.0
                previous = cycle_start

                self.step(motion, t)
                self._record(dt_ms)
                self._note_timing(dt_ms)

                self._sleep_until(cycle_start + self.period_s)
        finally:
            self.stats.total_s = time.monotonic() - t0
            self._exit()

        if not self.stats.kept_up:
            logger.warning(
                "%s 주기를 못 지킴: 평균 %.1fHz (목표 %.0fHz). "
                "게인을 만지기 전에 이것부터 볼 것 -- 계산이나 수거가 예산을 넘김",
                self.robot.id, self.stats.mean_hz, self.hz,
            )
        logger.info("%s 종료: %s", self.robot.id, self.stats.summary())
        return self.stats

    # ---- 들어가고 나오기 ----------------------------------------------------
    def _enter(self) -> None:
        if not self.robot.is_connected:
            self.robot.connect()

        if self.mode is Mode.OBSERVE:
            # 관찰 모드에서 토크를 끊는 것은 하드 안전장치임. 설정이 잘못돼 있어도
            # 다리가 안 움직이게 함.
            self.robot.disable()
        else:
            self.robot.enable()

        if self.telemetry is not None:
            self.telemetry.open()

    def _exit(self) -> None:
        """자세를 붙잡은 뒤 힘을 뺌.

        바로 끊으면 서 있는 다리가 주저앉음. 예외로 빠져나가도 이 경로를 탐.
        """
        try:
            if self.mode is Mode.CONTROL and self.robot.is_connected:
                self._settle()
        except Exception as e:
            logger.warning("%s 정지 자세 유지 실패 (계속 진행함): %s", self.robot.id, e)

        try:
            if self.robot.is_connected:
                self.robot.disable()
        except Exception as e:
            logger.warning("%s 토크 차단 실패: %s", self.robot.id, e)

        if self.telemetry is not None:
            self.telemetry.close()

    def _settle(self) -> None:
        """현재 자세를 목표로 몇 주기 보냄. 게인은 그대로 둠."""
        hold = getattr(self.robot, "hold", None)
        if not callable(hold):
            return
        for _ in range(SETTLE_CYCLES):
            self.robot.send(hold())
            self.robot.collect()
            time.sleep(self.period_s)

    # ---- 주기 -------------------------------------------------------------
    def _sleep_until(self, deadline: float) -> None:
        """다음 주기까지 기다림. 이미 늦었으면 **따라잡지 않음.**

        밀린 만큼 다음 주기를 줄여 따라잡으면 그 주기가 더 짧아져 계산이 또 밀림.
        한 주기를 늦게 시작하고 마는 편이 나음.
        """
        remaining = deadline - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)

    def _note_timing(self, dt_ms: float) -> None:
        self.stats.cycles += 1
        if dt_ms > self.stats.worst_dt_ms:
            self.stats.worst_dt_ms = dt_ms
        # 첫 주기는 직전이 없어 dt 가 0에 가까움. 세지 않음.
        if self.stats.cycles > 1 and dt_ms > self.period_s * 1000.0 * OVERRUN_RATIO:
            self.stats.overruns += 1
            if self.stats.overruns == 1:
                logger.warning(
                    "%s 주기가 밀림: %.1fms (목표 %.1fms). 이후로는 세기만 함",
                    self.robot.id, dt_ms, self.period_s * 1000.0,
                )

    def _record(self, dt_ms: float) -> None:
        if self.telemetry is None:
            return
        try:
            self.telemetry.record(loop_dt_ms=dt_ms)
        except Exception as e:
            # 관측이 제어를 멈추면 관측할 대상이 없어짐.
            logger.warning("%s 텔레메트리 기록 실패: %s", self.robot.id, e)
