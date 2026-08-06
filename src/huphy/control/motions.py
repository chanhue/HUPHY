"""동작 — 매 주기 무엇을 시킬지 정함.

전부 **순수 함수를 만드는 함수**임. 하드웨어도 CAN 도 모르고, 시간과 관찰을 받아
관절 목표를 냄.

    (경과 시간 초, 지금 관찰) -> {관절 이름: 목표 각도}  또는  None

`None` 을 내면 그 주기는 명령을 보내지 않음.


## 왜 루프에서 떼어 놓나

루프는 주기와 안전만 봄. 무엇을 시킬지는 여기가 정함.

덕분에 같은 루프로 유지, 사인파 흔들기, 보행 궤적을 다 돌릴 수 있고, **동작을
하드웨어 없이 시험할 수 있음** — 함수를 그냥 부르면 됨.


## 게인 튜닝에 쓰는 것

    hold          지금 자세를 붙잡음. 처짐과 떨림을 봄
    step          한 번에 목표를 옮김. 오버슛과 정착을 봄
    sine          왕복. 추종 지연과 진폭 감쇠를 봄

**계단 응답이 가장 많은 것을 알려줌.** 목표를 지나쳤다 돌아오면 `kd` 가 부족하고,
못 미치면 `kp` 가 부족하고, 부르르 떨면 `kp` 가 과함.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Mapping, Optional, Sequence


def hold(targets: Mapping[str, float]):
    """정해진 자세를 계속 보냄. 목표가 바뀌지 않음.

    게인 튜닝의 출발점임 — 자세가 유지되는지, 처지는지, 떨리는지를 봄. 여기서
    떨리면 어떤 동작을 시켜도 떨림.
    """
    fixed = {k: float(v) for k, v in targets.items()}

    def motion(t: float, observation: Dict[str, Any]) -> Dict[str, float]:
        return dict(fixed)

    return motion


def freeze(joints: Sequence[str], *, from_observation: bool = True):
    """**첫 주기의 실측 위치**를 목표로 잡고 그대로 유지함.

    `hold` 와 다름 — 목표를 미리 정하지 않고 다리가 지금 있는 자리를 씀. 토크를
    넣는 순간 관절이 튀지 않게 하려는 것임.

    제어를 시작할 때 첫 동작으로 씀. 목표를 0으로 잡고 시작하면 다리가 0을 향해
    한 번에 움직임.
    """
    captured: Dict[str, float] = {}

    def motion(t: float, observation: Dict[str, Any]) -> Optional[Dict[str, float]]:
        if not captured:
            for joint in joints:
                value = observation.get(f"{joint}.pos")
                if value is None:
                    return None          # 아직 상태를 못 받음. 이번 주기는 건너뜀
                captured[joint] = float(value)
        return dict(captured)

    return motion


def step(
    joint: str,
    *,
    start: float,
    end: float,
    at_s: float = 1.0,
    hold_others: Optional[Mapping[str, float]] = None,
):
    """`at_s` 초에 목표를 한 번에 옮김. **계단 응답.**

    게인 튜닝에서 가장 많은 것을 알려줌.

        목표를 지나쳤다 돌아옴   ->  kd 가 부족
        목표까지 못 감           ->  kp 가 부족
        부르르 떨림              ->  kp 가 과함
        느리게 도달              ->  kp 를 올릴 여지

    앞에 `at_s` 만큼 여유를 두는 이유: 계단 **직전** 그래프가 평평해야 오버슛을
    읽을 수 있음. 시작하자마자 뛰면 초기 과도상태와 섞임.

    **점프 가드가 이 계단을 자름.** 한 주기에 `max_delta_deg` 까지만 가므로
    실제로는 몇 주기에 걸쳐 올라감. 그 기울기가 속도 상한임.
    """
    others = {k: float(v) for k, v in (hold_others or {}).items()}

    def motion(t: float, observation: Dict[str, Any]) -> Dict[str, float]:
        out = dict(others)
        out[joint] = float(end) if t >= at_s else float(start)
        return out

    return motion


def sine(
    joint: str,
    *,
    center: float = 0.0,
    amplitude: float = 5.0,
    hz: float = 0.5,
    hold_others: Optional[Mapping[str, float]] = None,
):
    """사인파로 왕복시킴.

    계단이 못 보는 것을 봄.

        추종 지연     목표보다 얼마나 늦게 따라오나
        진폭 감쇠     명령한 만큼 안 움직이면 게인이 부족
        주파수 의존   빠르게 흔들수록 둘 다 나빠짐

    시작 위치가 `center` 임 — 사인이 0에서 시작하므로 토크를 넣는 순간 튀지 않음.

    `hz` 를 올려 가며 진폭이 절반으로 떨어지는 지점을 찾으면 그것이 이 게인에서의
    대역폭임.
    """
    others = {k: float(v) for k, v in (hold_others or {}).items()}
    omega = 2.0 * math.pi * float(hz)

    def motion(t: float, observation: Dict[str, Any]) -> Dict[str, float]:
        out = dict(others)
        out[joint] = float(center) + float(amplitude) * math.sin(omega * t)
        return out

    return motion


def ramp(
    joint: str,
    *,
    start: float,
    end: float,
    seconds: float,
    hold_others: Optional[Mapping[str, float]] = None,
):
    """정해진 시간에 걸쳐 천천히 옮김. 도착하면 유지함.

    계단과 달리 **점프 가드에 걸리지 않게** 스스로 속도를 제한함. 처음 자세를 잡을
    때나 큰 각도를 안전하게 옮길 때 씀.

    기울기가 `(end - start) / seconds` 임. 이 값이 `max_delta_deg * hz` 를 넘으면
    가드가 또 자르므로, 그때는 `seconds` 를 늘릴 것.
    """
    if seconds <= 0:
        raise ValueError(f"seconds 는 0보다 커야 함 (받은 값 {seconds})")
    others = {k: float(v) for k, v in (hold_others or {}).items()}
    span = float(end) - float(start)

    def motion(t: float, observation: Dict[str, Any]) -> Dict[str, float]:
        out = dict(others)
        fraction = min(1.0, max(0.0, t / seconds))
        out[joint] = float(start) + span * fraction
        return out

    return motion


def chain(*motions):
    """여러 동작을 순서대로 이음. `(동작, 길이초)` 쌍을 받음.

        chain((ramp(...), 2.0), (sine(...), 10.0))

    각 동작은 **자기 구간의 0초부터** 시작하는 시간을 받음. 이어 붙일 때마다
    시간 계산을 다시 하지 않아도 되게 함.

    마지막 동작이 끝나면 `None` 을 내어 명령을 멈춤.
    """
    segments = [(m, float(d)) for m, d in motions]

    def motion(t: float, observation: Dict[str, Any]):
        elapsed = float(t)
        for inner, duration in segments:
            if elapsed < duration:
                return inner(elapsed, observation)
            elapsed -= duration
        return None

    return motion
