"""벤더 중립 모터 추상.

**어느 벤더에도 특유하지 않은 개념만** 둠. 인코딩 범위(pmax/vmax/tmax) 같은 것은
MIT류 프로토콜 특유의 개념이므로 `robstride/tables.py` 로 내려감. CANopen 계열은
pulse 단위를 쓰고 인코딩 범위라는 개념 자체가 없음.

`python-can` 을 import하지 않음. 자료형과 인터페이스만 있어 하드웨어 없이 테스트됨.


## raw 와 cal 은 다른 공간임

모터가 보고하는 각도(raw)와 사람이 말하는 관절 각도(cal)는 다름.

    cal = sign * raw + offset

`sign` 은 모터 회전 방향이 관절 양의 방향과 반대일 때 -1 임. `offset` 은 기계 영점과
관절 영자세의 차이임. 둘 다 조립 결과라 **실측해야만 알 수 있음.**

    raw   모터가 보고하는 값. 코덱과 버스가 다루는 공간
    cal   관절 각도. 기구학·안전·명령이 다루는 공간

경계는 `robots/` 임. 버스 위쪽은 전부 cal, 아래쪽은 전부 raw 로 둠.


## 적은 것과 잰 것을 나눔

    Motor              사람이 적는 것. 배선·설계·튜닝에서 옴  ->  config/
    MotorCalibration   조립을 재서 얻는 것                    ->  calibration/

기준이 다르므로 파일을 나눔. 모터를 다시 달면 `MotorCalibration` 만 무효가 되고,
기구 설계가 바뀌면 `Motor` 만 바뀜.

id 로 키를 맞춘 표를 여러 개 늘어놓지 않고 **모터 하나당 항목 하나**로 둠. 표가
나뉘어 있으면 모터를 추가할 때 한 표에만 넣고 다른 표에서 빠뜨릴 수 있음. 관절이
20개를 넘어가면 실제로 일어남.


## 한계값은 Motor 에 있고 cal 공간임

관절 가동범위는 기구 설계에서 오는 값이라 첫 동작 **전에** 이미 알고 있어야 함.
움직여 보고 알아내는 값이 아님 — 사람이 다리를 손으로 하드스톱까지 훑는 것이
중력과 감속비 때문에 불가능하고, 발목은 두 모터가 폐루프로 물려 있어 한쪽만 훑으면
링크가 물림.

설계에서 오는 값이므로 **영점을 어디에 잡았는지와 무관함.** 그래서 cal 공간에 두고
`MotorCalibration` 과 분리함. raw 공간에 두면 영점을 다시 잡을 때 숫자는 그대로인데
가리키는 물리적 위치가 달라짐.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class Gains:
    """MIT 모드에서 매 프레임 함께 보내는 PD 게인.

    모터 펌웨어가 이 값으로 토크를 계산함:

        tau = kp * (목표각 - 현재각) + kd * (0 - 현재속도) + 토크_FF

    레지스터에 저장하는 방식이 아니라 명령마다 실려 나가므로, 한 주기 안에서도
    관절마다 다르게 줄 수 있음. 브링업 중 전체를 한꺼번에 낮추는 것도 여기서 함.

    기본값 0은 **토크 없음**임. 실측 전에 움직이지 않도록 일부러 0으로 둠.
    """

    kp: float = 0.0
    kd: float = 0.0

    def scaled(self, factor: float) -> "Gains":
        """전체를 같은 비율로 낮춤. 브링업 초반에 씀."""
        f = float(factor)
        return Gains(kp=self.kp * f, kd=self.kd * f)


@dataclass(frozen=True)
class Motor:
    """한 모터에 대해 **사람이 적는** 것. 재서 얻는 값은 `MotorCalibration` 에 있음.

    이름(관절 이름)은 여기 없음 — 이 자료형을 담는 dict 의 키가 이름임.
    같은 이름이 두 모터를 가리킬 수 없게 하려는 것임.

    `model` 이 문자열인 이유: 이 계층은 벤더를 모름. 유효한 모델인지는 벤더 모듈이
    판단함 (`robstride.tables.Model`). 여기서 enum 으로 좁히면 벤더를 추가할 때마다
    중립 계층을 고쳐야 함.
    """

    id: int
    """CAN id. 버스 안에서 유일해야 함."""

    model: str
    """벤더 모델명. 인코딩 범위를 고르는 데 쓰임 (`RS02` 와 `RS00` 은 토크 범위가 다름)."""

    limits_deg: Optional[Tuple[float, float]] = None
    """하드스톱 위치, **cal 공간.** 여유를 뺀 값이 아니라 하드스톱 자체임.

    `None` 은 "아직 모름"이고 "제한 없음"이 아님. 상위에서 제어 진입을 막아야 함 —
    한계를 모르는 채로 토크를 넣는 것이 가장 위험함.
    """

    gains: Gains = field(default_factory=Gains)

    def __post_init__(self) -> None:
        if self.limits_deg is None:
            return
        lo, hi = float(self.limits_deg[0]), float(self.limits_deg[1])
        if not lo < hi:
            raise ValueError(
                f"m{self.id}: limits_deg 의 lo 가 hi 보다 작아야 함 (받은 값 {lo}, {hi}). "
                f"cal 공간이라 sign 으로 뒤집히지 않음"
            )

    @property
    def is_configured(self) -> bool:
        """제어에 쓸 만큼 채워졌는지. 한계와 게인이 둘 다 있어야 함."""
        return self.limits_deg is not None and self.gains.kp > 0.0


@dataclass
class MotorState:
    """모터가 보고한 한 시점의 상태. 각도는 **raw 공간**임."""

    position_deg: float = 0.0
    velocity_deg_s: float = 0.0
    torque_nm: float = 0.0
    temp_c: float = 0.0
    stamp: float = 0.0
    """`time.monotonic()`. 0이면 아직 한 번도 못 받음.

    벽시계(`time.time()`)가 아님 — NTP 보정이나 서머타임에 뒤로 갈 수 있어 나이
    계산이 음수가 됨.
    """

    @property
    def is_valid(self) -> bool:
        """한 번이라도 응답을 받았는지."""
        return self.stamp > 0.0

    def age(self, now: float) -> float:
        """마지막 갱신 이후 흐른 초. 한 번도 못 받았으면 무한대임."""
        if not self.is_valid:
            return float("inf")
        return max(0.0, float(now) - self.stamp)

    def is_fresh(self, now: float, max_age_s: float) -> bool:
        """제어에 쓸 만큼 최신인지.

        오래된 상태로 계산하면 안 됨. 점프 가드는 "지금 위치"를 기준으로 자르는데,
        그 기준이 몇 초 전 값이면 실제로는 큰 점프를 통과시킴.
        """
        return self.age(now) <= float(max_age_s)


def wrap180(deg: float) -> float:
    """`[-180, 180)` 으로 접음.

    모터는 `zero_sta = 1` 이라 각도를 이 범위로만 보고함. 관절이 경계를 지나면
    보고값이 179 에서 -177 로 **360 만큼 건너뜀.**

    `cal = sign x raw + offset` 은 산수라 그 건너뜀이 그대로 통과함. 접지 않으면
    영점이 raw 의 `±180` 근처에 잡힌 관절에서 40도 움직인 것이 356도 범위로 나오고,
    한계 가드도 그 값을 보고 엉뚱하게 자름.

    두 공간에 **같은 규약**을 씀 (이슈 #1). 전제는 관절이 한 바퀴를 돌지 않는 것 --
    영점에서 `±180` 안에 전 가동 범위가 들어와야 함. 다리 관절은 전부 그러함.
    """
    return (float(deg) + 180.0) % 360.0 - 180.0


@dataclass
class MotorCalibration:
    """조립을 재서 얻는 값. 코드가 아니라 데이터이며 JSON으로 저장됨.

        cal = sign * raw + offset

    모터를 떼거나 다시 조립하면 무효가 됨. 튜닝값(게인)은 성격이 달라 여기 없음 --
    같은 모델로 갈면 그대로 쓰는 값이라 `robot.yaml` 에 있음.
    """

    motor_id: int
    sign: float = 1.0
    """관절 양의 방향과 모터 회전 방향이 반대면 -1."""

    offset_deg: float = 0.0
    """기계 영점에서 관절 영자세까지의 차이 (cal 공간)."""

    zero_reference: str = ""
    """기계 영점을 **어느 자세에서** 잡았는지 사람이 남기는 메모.

    `0xFE`(영점 설정)는 모터가 저장하지만 "그때 다리가 어떤 자세였는지"는 어디에도
    남지 않음. 이 메모가 없으면 나중에 영점을 재현할 수 없음.
    """

    limits_deg: Optional[Tuple[float, float]] = None
    """하드스톱 위치 (cal 공간). `commission sweep` 이 재서 적음.

    `None` 은 **아직 안 잼** 이지 "제한 없음" 이 아님. 값이 없으면
    `Motor.is_configured` 가 `False` 라 제어 진입이 막힘.
    """

    def __post_init__(self):
        if self.limits_deg is None:
            return
        lo, hi = float(self.limits_deg[0]), float(self.limits_deg[1])
        if lo >= hi:
            raise ValueError(
                f"m{self.motor_id}: limits_deg 의 lo 가 hi 보다 작아야 함 "
                f"(받은 값 {lo}, {hi})"
            )
        self.limits_deg = (lo, hi)

    def raw_to_cal(self, raw_deg: float) -> float:
        return wrap180(float(self.sign) * float(raw_deg) + float(self.offset_deg))

    def cal_to_raw(self, cal_deg: float) -> float:
        if abs(float(self.sign)) < 1e-9:
            raise ValueError(
                f"m{self.motor_id}: sign이 0이라 역변환 불가. "
                f"캘리브레이션이 채워지지 않았을 수 있음"
            )
        return wrap180((float(cal_deg) - float(self.offset_deg)) / float(self.sign))


@dataclass
class MotorFault:
    """고장 상태. 비트 필드라 여러 고장이 동시에 설 수 있음.

    비트 이름은 벤더마다 다르므로 해석표는 벤더 모듈에 있음
    (`robstride.tables.FAULT_BITS`). 여기는 결과만 담음.
    """

    raw: int = 0
    bits: Dict[str, bool] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.raw == 0

    def active(self) -> List[str]:
        """켜진 고장 이름들. 정상이면 빈 리스트."""
        return [name for name, on in self.bits.items() if on]


class MotorsBus(abc.ABC):
    """모터 버스 최소 인터페이스 — **런타임 조작만.**

    되돌리기 어려운 1회성 조작(CAN ID 변경, 프로토콜 전환, 기계영점, 플래시 저장)은
    여기 두지 않음. 벤더별 `commissioning` 모듈로 격리함. 제어 루프에서 실수로
    호출될 수 있는 자리에 두면 안 되는 것들임.

    각도는 전부 **raw 공간**임. 변환은 `robots/` 가 함.
    """

    # ---- 구성 -------------------------------------------------------------
    @property
    @abc.abstractmethod
    def motor_ids(self) -> Tuple[int, ...]:
        """이 버스에 물린 모터 id들."""

    @property
    @abc.abstractmethod
    def is_connected(self) -> bool:
        ...

    # ---- 수명 -------------------------------------------------------------
    @abc.abstractmethod
    def connect(self) -> None:
        ...

    @abc.abstractmethod
    def disconnect(self) -> None:
        """토크를 끊고 소켓을 닫음. **여러 번 불려도 안전해야 함.**"""

    def __enter__(self) -> "MotorsBus":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """예외로 빠져나가도 토크가 끊기도록 보장함.

        이게 없으면 제어 중 예외가 났을 때 모터가 마지막 명령을 계속 유지함 —
        사람이 전원을 뽑을 때까지 다리가 힘을 주고 있음.
        """
        self.disconnect()

    # ---- 토크 -------------------------------------------------------------
    @abc.abstractmethod
    def enable_torque(self, motors: Optional[Iterable[int]] = None) -> None:
        ...

    @abc.abstractmethod
    def disable_torque(self, motors: Optional[Iterable[int]] = None) -> None:
        ...

    # ---- 상태 -------------------------------------------------------------
    @abc.abstractmethod
    def refresh_states(self, motors: Optional[Iterable[int]] = None) -> List[int]:
        """상태를 요청하고 응답을 수거함.

        반환: **응답이 없었던** 모터 id 리스트. 빈 리스트면 전부 응답함.

        예외를 던지지 않는 이유: 한 모터가 한 주기 빠지는 것은 흔한 일이고, 그때마다
        루프가 죽으면 안 됨. 호출부가 몇 주기 연속인지 보고 판단함.
        """

    @abc.abstractmethod
    def state(self, motor_id: int) -> MotorState:
        """마지막으로 수거된 상태. 새로 읽지 않음."""

    @abc.abstractmethod
    def states(self) -> Dict[int, MotorState]:
        ...


def resolve_motor_list(
    motors: Optional[Iterable[int]], all_ids: Sequence[int]
) -> List[int]:
    """`None` 이면 전체, 아니면 지정된 것만. 이 버스에 없는 id면 에러임.

    조용히 무시하면 "명령을 보냈는데 안 움직인다"가 되고, 원인이 오타인지 배선인지
    구분되지 않음.
    """
    if motors is None:
        return list(all_ids)
    out = [int(m) for m in motors]
    unknown = [m for m in out if m not in all_ids]
    if unknown:
        raise ValueError(f"이 버스에 없는 모터 id: {unknown} (가용: {list(all_ids)})")
    return out
