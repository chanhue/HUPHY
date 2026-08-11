"""다리 하나 — 관절 이름과 모터 id 를 잇는 곳.

`Robot` 계약을 채움. 설정·실측값·버스·기구학·안전을 한데 모아 쓰는 유일한 계층임.


## 관절과 모터가 1:1 이 아님

    hipz  hipx  hipy  knee    모터 하나가 관절 하나
    ankle_pitch  ankle_roll   모터 **두 개**가 관절 두 개를 만듦

그래서 명령은 6개 관절로 받고, 발목만 기구학을 거쳐 모터 두 개로 풀림.

    명령    hipz hipx hipy knee ankle_pitch ankle_roll     6개
    모터    m7   m8   m9   m10  m11 m12                    6개


## 한 주기에 일어나는 일

    1  관절 목표 (cal)
    2  발목 pitch/roll -> a1/a2          기구학. 닫힌 해
    3  모터별 cal 목표
    4  한계·점프 검사                     safety.guards. cal 공간
    5  cal -> raw                        캘리브레이션
    6  MIT 프레임                         코덱
    7  전송
    8  수거 -> 상태 갱신

4번이 cal 공간인 이유: 한계가 cal 공간에 있음 (이슈 #2). raw 로 내린 뒤에 검사하면
`sign` 이 -1 인 관절에서 부호가 뒤집혀 한계가 반대로 걸림.


## 발목의 두 가지 실패는 다르게 다룸

**IK 가 안 풀리면 통째로 버림.** 두 모터 다 직전 명령을 유지함. 한쪽만 새 명령을
받으면 두 로드가 서로 다른 자세를 요구해 관절이 비틀림.

**한계에 잘리는 것은 개별로 처리해도 됨.** 로드가 둘이고 자유도가 둘이라 어떤
`(a1, a2)` 조합이든 대응하는 발 자세가 하나 있음 — 잘린 각도 쌍도 비틀린 자세가
아니라 그냥 다른 자세임. 실제로 실행된 자세는 FK 로 되짚어 `last_sent` 에 냄.

나머지 관절은 서로 독립이라 한 관절이 잘려도 다른 관절은 정상임.
"""

from __future__ import annotations

import logging
import time
from dataclasses import replace
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from .. import calibration as calib
from ..config.schema import LimbConfig, SafetyConfig
from ..kinematics.ankle import AnkleKinematics, AnkleUnreachableError
from ..motors.base import MotorCalibration
from ..motors.robstride.bus import MitCommand, RobStrideBus
from ..safety import guards
from .base import Action, Observation, Robot

logger = logging.getLogger(__name__)

SINGLE_JOINTS = ("hipz", "hipx", "hipy", "knee")
ANKLE_JOINTS = ("ankle_pitch", "ankle_roll")
ANKLE_MOTORS = ("ankle_a1", "ankle_a2")

JOINT_NAMES = SINGLE_JOINTS + ANKLE_JOINTS
REQUIRED_MOTORS = SINGLE_JOINTS + ANKLE_MOTORS
"""설정에 반드시 있어야 하는 모터 이름. 발목만 관절 이름과 다름."""


class Leg(Robot):
    """다리 하나. CAN 채널 하나를 씀.

    설정에는 모터가 `ankle_a1`/`ankle_a2` 로 있지만, 명령은 `ankle_pitch`/
    `ankle_roll` 로 받음 — 사람과 보행 궤적이 다루는 것은 관절이지 링키지가 아님.
    """

    name = "leg"

    def __init__(
        self,
        config: LimbConfig,
        bus: RobStrideBus,
        *,
        safety: Optional[SafetyConfig] = None,
        calibration: Optional[Mapping[str, MotorCalibration]] = None,
        kinematics: Optional[AnkleKinematics] = None,
        allow_uncalibrated: bool = False,
        imus: Optional[Sequence[Any]] = None,
    ) -> None:
        self.config = config
        self.id = config.name
        self.bus = bus
        self.safety = safety if safety is not None else SafetyConfig()
        self.allow_uncalibrated = allow_uncalibrated

        self.imus: Tuple[Any, ...] = tuple(imus or ())
        """이 다리에 붙은 IMU 들. **없어도 다리는 그대로 돎.**

        어느 다리에 붙었는지는 설정(`ImuConfig.mount`)이 정하고, 조립하는 쪽이
        골라서 넣어 줌. 다리는 어느 회사 센서인지 모름 -- `sensors.base.Imu` 만 봄.
        """

        missing = [m for m in REQUIRED_MOTORS if m not in config.motors]
        if missing:
            raise ValueError(
                f"{config.name}: 다리에 필요한 모터가 없음 {missing}. "
                f"설정에 있는 것: {sorted(config.motors)}"
            )

        self.kinematics = kinematics if kinematics is not None else AnkleKinematics()

        if calibration is None:
            calibration = (
                calib.load(config.calibration_path)
                if config.calibration_path and config.calibration_path.is_file()
                else calib.identity(config.motors)
            )
        self._set_calibration(calibration)

        self._last_sent: Dict[str, float] = {}
        self._ankle_guess: Tuple[float, float] = (0.0, 0.0)
        self.counters = guards.GuardCounters()

        # 링크 상태. 명령이 씹혔는지 판단하는 근거임 -- MIT 모드는 명령을 받으면
        # 반드시 상태 프레임으로 답하므로, 안 오면 그 모터가 처리하지 않은 것임.
        self._missing: set = set(self.config.motor_ids)
        self._miss_streak: Dict[int, int] = {i: 0 for i in self.config.motor_ids}
        self._awaiting: Tuple[int, ...] = tuple(self.config.motor_ids)
        """직전에 명령을 보낸 모터들. 응답을 기다리는 대상임."""

        # 마지막 사건 시각. 누적 카운터만으로는 언제 일어났는지 그래프에서 계단을
        # 찾아야 함. 이 값은 사건 직후 0으로 떨어져 눈에 바로 띔.
        self._last_clip_t: Optional[float] = None
        self._last_reject_t: Optional[float] = None

    # ---- 구성 -------------------------------------------------------------
    def _set_calibration(self, calibration: Mapping[str, MotorCalibration]) -> None:
        """실측값을 붙임. **한계각도 여기서 들어옴.**

        `robot.yaml` 에는 한계가 없음 -- 재는 값이라 캘리브레이션 파일에 있음
        (이슈 #2). 가드는 `Motor.limits_deg` 를 보므로, 읽어 온 값을 설정 사본에
        옮겨 넣어 한 군데서만 보게 함.
        """
        self.calibration = dict(calibration)
        self._by_id = calib.attach(self.calibration, self.config.motors)
        self.config = replace(
            self.config,
            motors={
                name: replace(motor, limits_deg=self.calibration[name].limits_deg)
                for name, motor in self.config.motors.items()
            },
        )

    def _motor_id(self, motor_name: str) -> int:
        return self.config.motors[motor_name].id

    def __repr__(self) -> str:
        return f"Leg({self.id}, {self.config.channel}, 관절 {len(JOINT_NAMES)}개)"

    @property
    def joint_names(self) -> Tuple[str, ...]:
        return JOINT_NAMES

    @property
    def motor_names(self) -> Tuple[str, ...]:
        """설정에 적힌 모터 이름. 발목이 둘로 나뉘어 있음."""
        return tuple(self.config.motors)

    @property
    def observation_features(self) -> Dict[str, type]:
        """관찰 필드. **모터 단위**로 냄 — 실제로 측정되는 것이 그것이기 때문임.

        발목 pitch/roll 은 FK 를 거쳐야 나오는데 뉴턴 반복이라 비쌈. 필요할 때만
        `ankle_pose()` 로 따로 구함.
        """
        out: Dict[str, type] = {}
        for motor in self.motor_names:
            out[f"{motor}.pos"] = float
            out[f"{motor}.vel"] = float
            out[f"{motor}.torque"] = float
            out[f"{motor}.temp"] = float
        out["stale_motors"] = int
        return out

    @property
    def action_features(self) -> Dict[str, type]:
        """명령 필드. **관절 단위**임. 발목은 pitch/roll 로 받음."""
        return {j: float for j in JOINT_NAMES}

    # ---- 수명 -------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self) -> None:
        self.bus.connect()
        self.refresh()
        self._connect_imus()

    def disconnect(self) -> None:
        for imu in self.imus:
            try:
                imu.disconnect()
            except Exception as e:
                logger.warning("%s IMU 종료 실패: %s", self.id, e)
        self.bus.disconnect()

    def _connect_imus(self) -> None:
        """IMU 를 엶. **실패해도 다리는 계속 씀.**

        IMU 는 관측이지 제어가 아님. 센서가 안 붙어 있다고 다리를 못 움직이면,
        고장 났을 때 로봇을 안전한 자세로 되돌리는 것조차 못 하게 됨.
        """
        for imu in self.imus:
            try:
                imu.connect()
            except Exception as e:
                logger.warning(
                    "%s IMU %s 를 열지 못함 (다리는 계속 씀): %s", self.id, imu.name, e
                )

    def imu_states(self) -> Dict[str, Any]:
        """붙은 IMU 들의 최신 값. 개체 이름 -> `ImuState`.

        **새로 통신하지 않음.** 벤더 구현이 백그라운드로 받아 둔 것을 꺼내기만 함.
        """
        return {imu.name: imu.read() for imu in self.imus}

    def enable(self) -> None:
        """토크를 넣음. **캘리브레이션과 설정이 채워져 있어야 함.**

        `allow_uncalibrated=True` 로 만들었으면 경고만 하고 진행함 — 커미셔닝
        단계에서 필요함.
        """
        if not self.is_calibrated:
            message = (
                f"{self.id}: 실측값이 채워지지 않음. "
                f"미설정 관절 {self.config.unconfigured()}, "
                f"미실측 관절 {calib.unmeasured(self.calibration)}"
            )
            if not self.allow_uncalibrated:
                raise RuntimeError(
                    f"{message}. 확인했으면 allow_uncalibrated=True 로 만들 것"
                )
            logger.warning("%s (allow_uncalibrated=True 라 진행함)", message)
        self.bus.enable_torque()

    def disable(self) -> None:
        self.bus.disable_torque()

    # ---- 캘리브레이션 -----------------------------------------------------
    @property
    def is_calibrated(self) -> bool:
        """한계·게인이 채워지고 영점 메모가 남아 있는지.

        둘 다 봐야 함 — 게인만 있으면 어디까지 가도 되는지 모르고, 한계만 있으면
        토크가 안 나감.
        """
        return self.config.is_configured and not calib.unmeasured(self.calibration)

    def calibrate(self) -> None:
        """실측값 파일을 다시 읽어 들임.

        재는 절차 자체는 여기 없음 — 사람이 `scripts/commission.py` 로 함.
        """
        if not self.config.calibration_path:
            raise RuntimeError(f"{self.id}: 캘리브레이션 파일이 설정되어 있지 않음")
        self._set_calibration(calib.load(self.config.calibration_path))
        logger.info("%s 캘리브레이션 다시 읽음", self.id)

    # ---- 좌표 변환 ---------------------------------------------------------
    def raw_to_cal(self, motor_name: str, raw_deg: float) -> float:
        return self._by_id[self._motor_id(motor_name)].raw_to_cal(raw_deg)

    def cal_to_raw(self, motor_name: str, cal_deg: float) -> float:
        return self._by_id[self._motor_id(motor_name)].cal_to_raw(cal_deg)

    # ---- 관찰 -------------------------------------------------------------
    def get_observation(self) -> Observation:
        """마지막으로 수거된 상태. **새로 읽지 않음.**

        각도는 cal 공간임. 제어 루프가 `collect()` 로 갱신함.
        """
        out: Observation = {}
        stale = 0
        for motor_name in self.motor_names:
            state = self.bus.state(self._motor_id(motor_name))
            if not state.is_valid:
                stale += 1
            out[f"{motor_name}.pos"] = self.raw_to_cal(motor_name, state.position_deg)
            out[f"{motor_name}.vel"] = state.velocity_deg_s
            out[f"{motor_name}.torque"] = state.torque_nm
            out[f"{motor_name}.temp"] = state.temp_c
        out["stale_motors"] = stale
        return out

    def ankle_pose(self) -> Optional[Tuple[float, float]]:
        """지금 발목 (pitch, roll). 구할 수 없으면 `None`.

        **FK 는 뉴턴 반복이라 비쌈.** 제어 루프에서 매 주기 부르지 말 것 — 관찰
        필드에 넣지 않은 이유임.

        직전 결과를 추정으로 씀. 같은 모터각 조합이 서로 다른 자세 둘에 대응하므로
        추정이 가까워야 함.
        """
        a1 = self.raw_to_cal("ankle_a1", self.bus.state(self._motor_id("ankle_a1")).position_deg)
        a2 = self.raw_to_cal("ankle_a2", self.bus.state(self._motor_id("ankle_a2")).position_deg)
        try:
            pose = self.kinematics.solve_fk(
                a1, a2,
                guess_pitch_deg=self._ankle_guess[0],
                guess_roll_deg=self._ankle_guess[1],
            )
        except AnkleUnreachableError as e:
            logger.debug("%s 발목 FK 실패: %s", self.id, e)
            return None
        self._ankle_guess = pose
        return pose

    # ---- 명령 -------------------------------------------------------------
    def _motor_targets(self, action: Action) -> Dict[str, float]:
        """관절 목표 -> 모터별 cal 목표. 발목만 기구학을 거침.

        모르는 관절 이름은 에러임. 오타를 조용히 무시하면 그 관절만 직전 명령을
        유지해 자세가 어긋남.
        """
        unknown = sorted(set(action) - set(JOINT_NAMES))
        if unknown:
            raise ValueError(
                f"{self.id}: 모르는 관절 {unknown} (가용: {list(JOINT_NAMES)})"
            )

        targets = {j: float(action[j]) for j in SINGLE_JOINTS if j in action}

        wants_ankle = any(j in action for j in ANKLE_JOINTS)
        if wants_ankle:
            if not all(j in action for j in ANKLE_JOINTS):
                raise ValueError(
                    f"{self.id}: 발목은 pitch 와 roll 을 함께 줘야 함 "
                    f"(받은 것: {sorted(set(action) & set(ANKLE_JOINTS))}). "
                    f"모터 두 개가 두 자유도를 같이 만들기 때문임"
                )
            try:
                a1, a2 = self.kinematics.solve_ik(
                    action["ankle_pitch"], action["ankle_roll"]
                )
            except AnkleUnreachableError as e:
                # 통째로 버림. 한쪽만 보내면 두 로드가 다른 자세를 요구해 비틀림.
                logger.warning("%s 발목 목표를 버림: %s", self.id, e)
            else:
                targets["ankle_a1"] = a1
                targets["ankle_a2"] = a2

        return targets

    def build_commands(self, action: Action) -> Dict[int, MitCommand]:
        """명령을 계산함. **CAN 을 쓰지 않음.**

        한계와 점프를 cal 공간에서 검사한 뒤 raw 로 내림. 무엇이 잘렸는지는
        `counters` 에 쌓임.
        """
        now = time.monotonic()
        commands: Dict[int, MitCommand] = {}
        sent: Dict[str, float] = {}

        for motor_name, target_cal in self._motor_targets(action).items():
            motor = self.config.motors[motor_name]
            motor_id = motor.id
            state = self.bus.state(motor_id)
            current_cal = (
                self.raw_to_cal(motor_name, state.position_deg)
                if state.is_valid
                else None
            )

            result = guards.apply(
                target_cal,
                current_cal,
                limits=motor.limits_deg,
                command_margin_deg=self.safety.command_margin_deg,
                max_delta_deg=self.safety.max_delta_deg,
                enforce_limits=self.safety.enforce_limits,
            )
            self.counters.record(result)
            if result.clips:
                self._last_clip_t = now
            if result.reject is not None:
                self._last_reject_t = now
            if not result.sendable:
                continue

            sent[motor_name] = result.value
            commands[motor_id] = MitCommand(
                position_deg=self.cal_to_raw(motor_name, result.value),
                kp=motor.gains.kp,
                kd=motor.gains.kd,
            )

        self._last_sent = self._as_joint_space(sent)
        return commands

    def _as_joint_space(self, motor_cal: Mapping[str, float]) -> Dict[str, float]:
        """실제로 나간 모터 목표를 관절 이름으로 되돌림.

        발목은 FK 를 거쳐야 하는데 비싸므로, 잘리지 않았으면 명령한 값을 그대로 씀.
        잘렸으면 FK 로 되짚어 **실제로 실행된 자세**를 냄 — 로그를 믿으려면
        무엇을 보냈는지가 아니라 무엇이 실행됐는지가 필요함.
        """
        out = {j: v for j, v in motor_cal.items() if j in SINGLE_JOINTS}
        if not all(m in motor_cal for m in ANKLE_MOTORS):
            return out
        try:
            pitch, roll = self.kinematics.solve_fk(
                motor_cal["ankle_a1"], motor_cal["ankle_a2"],
                guess_pitch_deg=self._ankle_guess[0],
                guess_roll_deg=self._ankle_guess[1],
            )
        except AnkleUnreachableError:
            return out
        out["ankle_pitch"] = pitch
        out["ankle_roll"] = roll
        return out

    def send(self, commands: Mapping[int, MitCommand]) -> int:
        """계산된 명령을 보냄. **수거하지 않음.**

        보낸 대상을 기억해 둠 — 응답은 **명령을 받은 모터만** 보내므로, 기다릴
        개수가 명령한 개수임. 전체를 기다리면 명령하지 않은 모터가 무응답으로 잡힘.
        """
        self._awaiting = tuple(commands)
        return self.bus.send_mit(dict(commands))

    def refresh(self) -> Tuple[int, ...]:
        """명령하지 않고 상태만 읽음. 응답이 없었던 모터 id 를 반환함.

        게인과 토크가 0인 명령을 보내고 그 응답을 받음 — MIT 모드에는 읽기 전용
        명령이 없음. 토크가 0이라 아무 일도 일어나지 않음.

        관찰 모드와 시작 직전에 씀.
        """
        self._awaiting = tuple(self.config.motor_ids)
        missing = self.bus.refresh_states()
        self._note_link(missing)
        return tuple(missing)

    def collect(self) -> Tuple[int, ...]:
        """응답을 수거함. **직전에 명령한 모터만** 기다림."""
        missing = self.bus.collect(expect=len(self._awaiting))
        self._note_link(missing)
        return tuple(m for m in missing if m in self._awaiting)

    def _note_link(self, missing) -> None:
        """이번 주기에 누가 답했는지 기록함.

        MIT 모드는 명령을 받으면 반드시 상태 프레임으로 답함. **안 오면 그 모터가
        명령을 처리하지 않은 것임** -- 이것이 애플리케이션 레벨 ack 임.

        CAN 하드웨어 ACK 와 다름. 하드웨어 ACK 는 버스에 붙은 아무 노드나 찍어주므로
        "누군가 들었다" 일 뿐이고, 그건 `can/tx_errors` 가 봄. 둘을 같이 보면
        원인이 좁혀짐 -- tx_errors 가 0인데 응답이 없으면 모터가 명령을 무시한
        것이고, 프로토콜이나 제어 모드가 어긋난 것임 (이슈 #11).
        """
        # 명령하지 않은 모터는 판정 대상이 아님. 직전 상태를 그대로 둠.
        self._missing = {m for m in missing if m in self._awaiting}
        for motor_id in self._awaiting:
            if motor_id in self._missing:
                self._miss_streak[motor_id] += 1
            else:
                self._miss_streak[motor_id] = 0

    def link_status(self, now: Optional[float] = None) -> Dict[str, Dict[str, float]]:
        """모터별 링크 상태. 관절 이름 -> `{age, ack, miss}`.

            age    마지막 응답 이후 경과 (ms). 한 번도 못 받았으면 -1
            ack     1  직전 주기에 명령하고 응답을 받음
                    0  직전 주기에 명령했는데 응답이 없음  <- 씹힘
                   -1  직전 주기에 명령하지 않음
            miss   연속 무응답 주기 수

        **`ack` 가 세 값인 이유**: 명령하지 않은 모터를 `1` 로 내면 거짓말이 되고
        `0` 으로 내면 없는 고장이 보임.

        이 구분이 실제로 필요함. 응답이 한 번도 없던 모터는 현재 위치를 모르므로
        가드가 명령을 거부함(`reject_nostate`) — 그러면 명령이 안 나가고, 명령이
        안 나갔으니 응답 대상에서도 빠짐. **그 모터가 진단에서 통째로 사라짐.**

        그래서 진짜 신호는 `age` 임. `-1` 이면 한 번도 응답한 적이 없다는 뜻이고,
        이 값은 명령 여부와 무관하게 참임.

        `-1` 을 쓰는 이유: 무한대는 JSON 으로 못 보내고 CSV 에서도 읽기 어려움.
        음수는 정상 범위에 없으므로 "없음" 을 뜻함.
        """
        stamp = time.monotonic() if now is None else now
        out: Dict[str, Dict[str, float]] = {}
        for motor_name, motor in self.config.motors.items():
            state = self.bus.state(motor.id)
            age = state.age(stamp)
            if motor.id not in self._awaiting:
                ack = -1.0
            elif motor.id in self._missing:
                ack = 0.0
            else:
                ack = 1.0
            out[motor_name] = {
                "age": -1.0 if age == float("inf") else age * 1000.0,
                "ack": ack,
                "miss": float(self._miss_streak[motor.id]),
            }
        return out

    def since_clip(self, now: Optional[float] = None) -> float:
        """마지막 클리핑 이후 경과 (초). 없었으면 -1."""
        return self._since(self._last_clip_t, now)

    def since_reject(self, now: Optional[float] = None) -> float:
        """마지막 거부 이후 경과 (초). 없었으면 -1."""
        return self._since(self._last_reject_t, now)

    @staticmethod
    def _since(stamp: Optional[float], now: Optional[float]) -> float:
        if stamp is None:
            return -1.0
        return max(0.0, (time.monotonic() if now is None else now) - stamp)

    @property
    def last_sent(self) -> Dict[str, float]:
        return dict(self._last_sent)

    def hold(self) -> Dict[int, MitCommand]:
        """지금 자세를 그대로 유지하는 명령. 정지 직전이나 대기에 씀.

        목표를 현재 위치로 두므로 오차가 0이 되어 자세를 붙잡기만 함.
        """
        commands: Dict[int, MitCommand] = {}
        for motor_name in self.motor_names:
            motor = self.config.motors[motor_name]
            state = self.bus.state(motor.id)
            if not state.is_valid:
                continue
            commands[motor.id] = MitCommand(
                position_deg=state.position_deg,
                kp=motor.gains.kp,
                kd=motor.gains.kd,
            )
        return commands
