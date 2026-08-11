"""Xsens MTi — `Imu` 프로토콜 구현.

`xbus/` 의 리더가 백그라운드 스레드로 받아 둔 dict 를 `ImuState` 로 바꿔 올림.
위쪽은 Xsens 도 Xbus 도 모름.


## 시리얼을 늦게 엶

`pyserial` 을 import 하는 것이 `connect()` 안에 있음. 센서를 안 쓰는 실행(테스트,
설정 확인)에서 패키지가 없다고 죽으면 안 됨. `motors/canbus.py` 가 `python-can` 을
다루는 방식과 같음.


## 시각을 다시 찍음

리더는 `time.time()`(벽시계)으로 찍음. 시각 동기화가 뒤로 점프하면 경과 시간이
음수가 되므로, **패킷이 새로 왔을 때 `time.monotonic()` 으로 다시 찍음.**

새 패킷인지는 리더가 넣어 준 벽시계 값이 바뀌었는지로 봄. 값이 그대로면 stamp 도
그대로 두어 `age_ms` 가 자라남 -- 센서가 멈춘 것을 그걸로 알아챔.
"""

from __future__ import annotations

import logging
import math
import time
from typing import Any, Optional, Tuple

from ..base import ImuState

logger = logging.getLogger(__name__)

DEFAULT_BAUDRATE = 921600
"""MTi-680G 출하 설정. 센서에 저장된 값과 **반드시 같아야** 함 -- 다르면 조용히
아무 패킷도 안 들어옴.
"""


def _triple(value: Any) -> Tuple[float, float, float]:
    """세 값짜리로 맞춤. 없거나 길이가 다르면 0.

    필드가 나타났다 사라지면 텔레메트리 열이 밀림 -- 값이 없어도 키는 내보냄.
    """
    if not value or len(value) < 3:
        return (0.0, 0.0, 0.0)
    return (float(value[0]), float(value[1]), float(value[2]))


class XsensImu:
    """Xsens MTi 하나.

    `port` 는 udev 로 고정한 심볼릭 링크를 쓸 것. USB 는 꽂는 순서대로 `ttyUSB0`,
    `ttyUSB1` 이 붙어 **재부팅마다 달라짐.**
    """

    def __init__(
        self,
        name: str,
        port: str,
        *,
        baudrate: int = DEFAULT_BAUDRATE,
    ) -> None:
        self.name = name
        self.port = port
        self.baudrate = int(baudrate)
        self._reader: Any = None
        self._last_packet_time: Optional[float] = None
        self._stamp: float = 0.0

    def __repr__(self) -> str:
        where = self.port if self.is_connected else f"{self.port} (안 열림)"
        return f"XsensImu({self.name}, {where})"

    @property
    def is_connected(self) -> bool:
        return self._reader is not None

    def connect(self) -> None:
        """포트를 열고 측정 모드로 들어감. 여러 번 불려도 안전함."""
        if self._reader is not None:
            return
        try:
            from .xbus.imu_reader import ImuReader  # noqa: PLC0415
        except ImportError as e:
            raise ImportError(
                f"IMU 를 읽으려면 pyserial 과 numpy 가 필요함. `pip install pyserial` ({e})"
            ) from e

        reader = ImuReader(port=self.port, baudrate=self.baudrate)
        try:
            reader.start()
        except Exception as e:
            raise ConnectionError(
                f"{self.name}: {self.port} 를 열 수 없음: {e}\n"
                f"포트가 맞는지, 보드레이트가 센서 설정과 같은지 확인할 것"
            ) from e

        self._reader = reader
        logger.info("IMU 연결됨: %s", self)

    def disconnect(self) -> None:
        """스레드를 멈추고 포트를 닫음. 여러 번 불려도 안전함."""
        if self._reader is None:
            return
        try:
            self._reader.stop()
        except Exception as e:
            logger.warning("%s 종료 실패: %s", self.name, e)
        finally:
            self._reader = None

    def __enter__(self) -> "XsensImu":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    def read(self) -> ImuState:
        """가장 최근 값. **새로 통신하지 않음.**

        아직 아무것도 못 받았으면 `is_valid` 가 거짓인 상태를 냄. 예외를 던지지
        않음 -- 센서 하나 때문에 제어 루프가 멈추면 안 됨.
        """
        if self._reader is None:
            return ImuState()

        try:
            data = self._reader.get_data()
        except Exception as e:
            logger.warning("%s 읽기 실패: %s", self.name, e)
            return ImuState()

        if not data:
            return ImuState()

        packet_time = data.get("timestamp")
        if packet_time != self._last_packet_time:
            self._last_packet_time = packet_time
            self._stamp = time.monotonic()

        roll, pitch, yaw = _triple(data.get("euler"))
        gyro_rad = _triple(data.get("rate_of_turn"))

        return ImuState(
            roll_deg=roll,
            pitch_deg=pitch,
            yaw_deg=yaw,
            accel_mps2=_triple(data.get("acceleration")),
            # 센서는 rad/s 로 줌. 프로젝트 전체가 도를 쓰므로 여기서 바꿈.
            gyro_dps=tuple(math.degrees(v) for v in gyro_rad),
            temp_c=float(data.get("temperature", 0.0)),
            stamp=self._stamp,
            is_valid=True,
        )
