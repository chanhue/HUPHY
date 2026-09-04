"""붙어 있는 IMU 묶음.

**IMU 는 팔다리의 부속이 아님.** 같은 센서가 다리에 붙었다가 몸통으로 옮겨감.
그래서 다리와 나란히 놓인 별도 단위로 다룸 — 왼다리·오른다리처럼.

    Biped
     ├ Leg("right_leg")
     ├ Leg("left_leg")
     └ ImuGroup           <- 여기

`ImuConfig` 를 팔다리 안이 아니라 로봇 밑에 둔 것과 짝임 (`config/schema.py`).
설정이 그렇게 생겼으니 실행 객체도 그렇게 생겨야 함.


## 여는 데 실패해도 올리지 않음

IMU 는 **관측이지 제어가 아님.** 센서가 안 붙어 있다고 로봇을 못 움직이면, 센서가
고장 났을 때 안전한 자세로 되돌리는 것조차 못 하게 됨.

그래서 실패는 경고로 남기고 그 센서만 빠짐. 값이 필요한 쪽(정책 등)이 시작 전에
`len()` 으로 확인함.


## 새로 통신하지 않음

`states()` 는 벤더 구현이 백그라운드로 받아 둔 것을 꺼내기만 함. 제어 주기 안에서
시리얼을 기다리면 주기가 센서에 끌려감.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Iterable, Iterator, Tuple

logger = logging.getLogger(__name__)


class ImuGroup:
    """IMU 여럿. 여는 것과 읽는 것만 함.

    목록처럼 다뤄짐 (`len`, `for`, `[0]`) — 텔레메트리가 붙은 센서를 훑어 열을
    만들기 때문임.
    """

    def __init__(self, imus: Iterable[Any] = (), *, owner: str = "") -> None:
        self.imus: Tuple[Any, ...] = tuple(imus)
        self.owner = owner
        """어디에 달렸는지. 로그 메시지에만 씀."""

    def __repr__(self) -> str:
        names = ", ".join(i.name for i in self.imus) or "없음"
        return f"ImuGroup({names})"

    def __len__(self) -> int:
        return len(self.imus)

    def __iter__(self) -> Iterator[Any]:
        return iter(self.imus)

    def __getitem__(self, index):
        return self.imus[index]

    @property
    def names(self) -> Tuple[str, ...]:
        return tuple(i.name for i in self.imus)

    def connect(self) -> None:
        """전부 엶. **실패한 것만 빠지고 나머지는 씀.**"""
        for imu in self.imus:
            try:
                imu.connect()
            except Exception as e:  # noqa: BLE001 - 센서 하나로 로봇을 멈추지 않음
                logger.warning(
                    "%s IMU %s 를 열지 못함 (없이 진행함): %s",
                    self.owner or "robot", imu.name, e,
                )

    def disconnect(self) -> None:
        """전부 닫음. **하나가 실패해도 나머지를 닫음.**"""
        for imu in self.imus:
            try:
                imu.disconnect()
            except Exception as e:  # noqa: BLE001
                logger.warning(
                    "%s IMU %s 종료 실패 (계속 진행함): %s",
                    self.owner or "robot", imu.name, e,
                )

    def states(self) -> Dict[str, Any]:
        """개체 이름 -> `ImuState`. **새로 통신하지 않음.**"""
        return {imu.name: imu.read() for imu in self.imus}
