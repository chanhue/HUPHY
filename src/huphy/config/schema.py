"""설정 자료형 — 사람이 적는 것의 구조.

`config/robot.yaml` 을 읽어 들이면 이 모양이 됨. 파일 읽기는 `loader.py` 가 함.

전부 `frozen` 임. 설정은 시작할 때 한 번 읽고 나면 바뀌지 않음 — 제어 중에 누가
한계를 넓히거나 게인을 올리는 일이 없어야 함.


## 종류와 개체를 나눔

    limbs:
      right_leg:          <- 개체 이름. 이 로봇에서 이 팔다리를 부르는 이름
        kind: leg         <- 종류. 어떤 기구학을 쓰는지
        side: right       <- 기하. 거울상인지

셋을 한 필드가 겸하면 확장할 때 막힘 (이슈 #5). 팔이 붙으면 `left_arm`, `right_arm`
이 생기고, 허리처럼 좌우가 없는 것도 생김.

개체 이름이 키인 이유: 같은 이름이 둘일 수 없게 하려는 것임.


## 각도는 전부 cal 공간임

`Motor.limits_deg` 는 관절 각도임. 모터가 보고하는 raw 값이 아님.
자세한 것은 `motors/README.md` 의 "raw 와 cal" 참조.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional, Tuple

from ..motors.base import Motor

DEFAULT_CONTROL_HZ = 100.0
DEFAULT_INTERFACE = "socketcan"


@dataclass(frozen=True)
class SafetyConfig:
    """명령이 모터로 나가기 전에 통과할 관문의 설정.

    `safety.guards.apply` 가 그대로 받아 씀.
    """

    command_margin_deg: float = 3.0
    """한계에서 이만큼 안쪽까지만 명령함.

    오버슛·관성·측정오차를 흡수하는 여유임. 게인을 튜닝하면 필요한 값이 달라짐 —
    지금 3° 는 임의로 잡은 출발점임.
    """

    max_delta_deg: float = 50.0
    """한 주기에 움직일 수 있는 최대 각도.

    100Hz 기준이라 50° 는 초당 5000° 임. 실제로 그렇게 도는 것이 아니라, 계산이
    튀었을 때 그 이상은 안 나가게 막는 상한임.
    """

    enforce_limits: bool = True
    """`False` 로 두면 한계 클리핑을 건너뜀. **커미셔닝 전용임.**

    캘리브레이션 전에는 한계가 어디인지 모르므로 검사할 것이 없음. 제어에서는
    켜 둘 것.
    """

    def __post_init__(self) -> None:
        if self.command_margin_deg < 0:
            raise ValueError(f"command_margin_deg 는 0 이상이어야 함 (받은 값 {self.command_margin_deg})")
        if self.max_delta_deg <= 0:
            raise ValueError(f"max_delta_deg 는 0보다 커야 함 (받은 값 {self.max_delta_deg})")


@dataclass(frozen=True)
class TelemetryConfig:
    """관측 설정. 제어에 영향을 주지 않음."""

    host: Optional[str] = None
    """UDP 수신 주소. `None` 이면 송신하지 않음."""

    port: int = 9870
    csv_path: Optional[str] = None
    csv_flush_every: int = 50
    """N 주기마다 디스크에 씀. 매번 쓰면 제어 주기가 튐."""

    @property
    def udp_enabled(self) -> bool:
        return bool(self.host)


@dataclass(frozen=True)
class LimbConfig:
    """팔다리 하나. **CAN 채널 하나에 대응함.**

    다리 하나가 버스 하나를 쓰는 이유: 양다리를 한 버스에 묶으면 12개 모터의
    프레임이 같은 선을 나눠 쓰게 되어 주기 예산이 두 배가 됨. 두 버스는 물리적으로
    독립이라 진짜로 겹쳐 보낼 수 있음 (이슈 #10).
    """

    name: str
    """개체 이름. `limbs` 의 키와 같음."""

    kind: str
    """종류. `leg`, `arm` 등. 어떤 기구학을 쓸지 정함."""

    side: Optional[str] = None
    """`left` / `right`. 좌우가 없는 부위는 `None`.

    거울상 여부를 정함. 실제 부호 뒤집기는 캘리브레이션의 `sign` 이 함 — 여기는
    "거울상이다" 라는 사실만 적음.
    """

    channel: str = "can0"
    interface: str = DEFAULT_INTERFACE
    control_hz: float = DEFAULT_CONTROL_HZ

    motors: Dict[str, Motor] = field(default_factory=dict)
    """관절 이름 -> 모터. 이름은 이 팔다리 안에서만 유일하면 됨.

    `hipz`, `knee` 처럼 짧게 씀. 어느 다리인지는 `LimbConfig.name` 이 이미 말함.
    """

    calibration_path: Optional[Path] = None
    """실측값 파일. `None` 이면 미캘리브레이션 상태임."""

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("팔다리 이름이 비어 있음")
        if not self.channel:
            raise ValueError(f"{self.name}: channel 이 비어 있음")
        if self.control_hz <= 0:
            raise ValueError(f"{self.name}: control_hz 는 0보다 커야 함 (받은 값 {self.control_hz})")
        if self.side not in (None, "left", "right"):
            raise ValueError(f"{self.name}: side 는 left/right 여야 함 (받은 값 {self.side!r})")

        ids = [m.id for m in self.motors.values()]
        dup = {i for i in ids if ids.count(i) > 1}
        if dup:
            raise ValueError(
                f"{self.name}: 같은 CAN id 를 여러 관절이 씀 {sorted(dup)}. "
                f"한 버스에서 id 가 겹치면 응답이 충돌해 구분되지 않음"
            )

    @property
    def period_s(self) -> float:
        return 1.0 / self.control_hz

    @property
    def motor_ids(self) -> Tuple[int, ...]:
        return tuple(m.id for m in self.motors.values())

    def motors_by_id(self) -> Dict[int, Motor]:
        """`RobStrideBus` 가 받는 형태. 관절 이름을 버린 것임."""
        return {m.id: m for m in self.motors.values()}

    def joint_of(self, motor_id: int) -> Optional[str]:
        """모터 id 로 관절 이름을 찾음. 진단 메시지에 씀."""
        for joint, motor in self.motors.items():
            if motor.id == motor_id:
                return joint
        return None

    @property
    def is_configured(self) -> bool:
        """모든 모터에 한계와 게인이 채워졌는지."""
        return bool(self.motors) and all(m.is_configured for m in self.motors.values())

    def unconfigured(self) -> Tuple[str, ...]:
        """아직 채워지지 않은 관절 이름들."""
        return tuple(name for name, m in self.motors.items() if not m.is_configured)


@dataclass(frozen=True)
class RobotConfig:
    """로봇 전체.

    지금은 다리 하나지만 팔·상체가 붙어도 이 구조가 유지됨 — `limbs` 에 항목이
    늘어날 뿐임.
    """

    name: str
    limbs: Dict[str, LimbConfig] = field(default_factory=dict)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    telemetry: TelemetryConfig = field(default_factory=TelemetryConfig)
    source_path: Optional[Path] = None
    """읽어 온 파일. 오류 메시지에 씀."""

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("로봇 이름이 비어 있음")

        # 채널이 겹치면 두 팔다리가 같은 선을 쓰는 것이므로, id 도 겹치면 안 됨.
        by_channel: Dict[str, list] = {}
        for limb in self.limbs.values():
            by_channel.setdefault(limb.channel, []).extend(
                (limb.name, joint, m.id) for joint, m in limb.motors.items()
            )
        for channel, entries in by_channel.items():
            seen: Dict[int, str] = {}
            for limb_name, joint, motor_id in entries:
                if motor_id in seen:
                    raise ValueError(
                        f"{channel}: CAN id {motor_id} 를 "
                        f"{seen[motor_id]} 와 {limb_name}.{joint} 가 같이 씀"
                    )
                seen[motor_id] = f"{limb_name}.{joint}"

    def limb(self, name: str) -> LimbConfig:
        try:
            return self.limbs[name]
        except KeyError:
            raise KeyError(
                f"{name!r} 이라는 팔다리가 없음 (가용: {sorted(self.limbs)})"
            ) from None

    def limbs_of_kind(self, kind: str) -> Dict[str, LimbConfig]:
        """종류로 골라냄. 두 다리에 같은 처리를 걸 때 씀."""
        return {n: l for n, l in self.limbs.items() if l.kind == kind}

    @property
    def channels(self) -> Tuple[str, ...]:
        """쓰이는 CAN 채널들. 중복 없이, 나온 순서대로."""
        out = []
        for limb in self.limbs.values():
            if limb.channel not in out:
                out.append(limb.channel)
        return tuple(out)

    @property
    def motor_count(self) -> int:
        return sum(len(l.motors) for l in self.limbs.values())
