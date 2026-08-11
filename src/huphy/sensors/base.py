"""센서 계층의 중립 자료형 — **벤더를 모름.**

`motors/base.py` 와 같은 자리임. 위쪽(로봇, 텔레메트리)은 여기 있는 것만 보고,
어느 회사 IMU 인지는 `sensors/<벤더>/` 가 앎.


## 단위

    각도    도 (deg)
    각속도  도/초 (deg/s)
    가속도  m/s^2

센서가 라디안으로 주면 벤더 모듈이 바꿔서 올림. 프로젝트 전체가 도를 쓰므로 여기서
섞이면 안 됨.


## `read()` 는 통신하지 않음

제어 루프 안에서 시리얼을 기다리면 주기가 통째로 밀림. 벤더 구현이 백그라운드로
받아 두고, `read()` 는 **가장 최근 값을 꺼내기만** 함.

그래서 `stamp` 가 중요함 — 값이 언제 것인지 모르면 센서가 죽었는지 알 수 없음.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional, Protocol, Tuple, runtime_checkable

ZERO3: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class ImuState:
    """IMU 가 보고한 한 시점의 값.

    **못 받았어도 객체는 나옴.** `is_valid` 가 거짓일 뿐임 — 호출부가 `None` 검사를
    하지 않아도 되게 함. 값은 전부 0 이고, 그것이 0도라는 뜻이 아니라 모른다는
    뜻임. 구분은 `is_valid` 와 `stamp` 로 함.
    """

    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0

    accel_mps2: Tuple[float, float, float] = field(default=ZERO3)
    """몸체 좌표계 가속도. 중력이 포함됨."""

    gyro_dps: Tuple[float, float, float] = field(default=ZERO3)
    """각속도. 벤더가 rad/s 로 주면 바꿔서 올림."""

    temp_c: float = 0.0

    stamp: float = 0.0
    """이 값을 받은 시각. `time.monotonic()` 기준.

    벽시계를 쓰지 않는 이유: 시각 동기화가 뒤로 점프하면 경과 시간이 음수가 됨.
    """

    is_valid: bool = False
    """한 번이라도 패킷을 받았는지. 거짓이면 나머지 필드는 의미 없음."""

    def age_ms(self, now: Optional[float] = None) -> float:
        """마지막 값 이후 경과. **한 번도 못 받았으면 -1.**

        `-1` 을 쓰는 이유: 무한대는 JSON 으로 못 보내고 CSV 에서도 읽기 어려움
        (`telemetry/snapshot.py` 의 규약과 같음).
        """
        if not self.is_valid:
            return -1.0
        return ((now if now is not None else time.monotonic()) - self.stamp) * 1000.0


@runtime_checkable
class Imu(Protocol):
    """IMU 하나가 갖춰야 할 것.

    `Robot` 계약과 같은 모양임 — 열고, 읽고, 닫음. 구현체는 이 프로토콜만 맞추면
    되고 상속하지 않아도 됨.
    """

    name: str
    """이 로봇에서 이 IMU 를 부르는 이름. **텔레메트리 필드 앞에 붙음.**

    붙는 자리(다리, 몸통)가 아니라 개체 이름을 쓰는 이유: 센서를 다리에서 몸통으로
    옮겨도 필드 이름이 그대로라 예전 로그·그래프 레이아웃과 맞음.
    """

    @property
    def is_connected(self) -> bool: ...

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def read(self) -> ImuState:
        """가장 최근 값. **새로 통신하지 않음.**"""
        ...
