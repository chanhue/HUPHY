"""EBIMU-9DOF — `Imu` 프로토콜 구현.

백그라운드 스레드가 시리얼을 계속 읽어 최신 `ImuState` 만 들고 있고, `read()` 는
그것을 꺼내기만 함. `sensors/xsens/imu.py` 와 같은 구조임.


## 시리얼을 늦게 엶

`pyserial` 을 import 하는 것이 `connect()` 안에 있음. 센서를 안 쓰는 실행(테스트,
설정 확인)에서 패키지가 없다고 죽으면 안 됨. `motors/canbus.py` 가 `python-can` 을
다루는 방식과 같음.


## 첫 패킷으로 구성을 확인함

패킷에는 어떤 항목이 켜져 있는지가 안 적혀 있음. 그래서 `connect()` 가 첫 줄을
받아 **필드 수가 `output` 합계와 같은지** 보고, 다르면 거기서 멈춤.

개수가 같은데 순서가 다른 경우는 못 잡음 -- 그건 `huphy-imu show` 가 센서 설정을
직접 읽어 대조함.

여기서 안 잡으면 값이 한 칸씩 밀린 채로 제어가 돌아감. 숫자가 그럴듯해서 실물에서
찾기 매우 어려움.


## 큐를 두지 않음

받은 줄을 쌓지 않고 최신 하나만 덮어씀. 제어 루프가 같은 주기로 읽으므로 쌓아 둘
이유가 없고, 쌓으면 루프가 한 번 밀렸을 때 묵은 값부터 꺼내게 됨.

몇 개가 버려졌는지는 `sensor_ms` 증가량으로 드러남.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, List, Optional, Sequence

from ..base import ImuState
from . import commands, protocol

logger = logging.getLogger(__name__)

DEFAULT_BAUDRATE = commands.BAUD_DEFAULT
"""공장 기본값. **센서에 저장된 값과 반드시 같아야 함** -- 다르면 조용히 아무
패킷도 안 들어옴.

15필드를 100Hz 로 보내면 한 줄 약 110바이트라 11 KB/s 임. 115200(약 11.5 KB/s)의
거의 전부라 여유가 없음. 항목을 더 켤 거면 `<sb6>` 으로 230400 으로 올릴 것.
"""

DEFAULT_OUTPUT = ("quat", "gyro", "accel")
"""설정에 `output` 이 없을 때 쓰는 구성. 제어에 필요한 최소임."""

READ_TIMEOUT_S = 0.2
"""`readline()` 한 번의 대기. 스레드가 `stop()` 에 반응하는 데 걸리는 시간이기도 함."""

HANDSHAKE_S = 1.0
"""`connect()` 가 첫 패킷을 기다리는 시간.

100Hz 면 10ms 안에 와야 함. 1초를 기다리는 것은 넉넉히 잡은 것이고, 이 시간을 다
쓰면 포트나 보레이트가 틀린 것임.
"""


class EbimuImu:
    """EBIMU 하나.

    `port` 는 udev 로 고정한 심볼릭 링크를 쓸 것. USB 는 꽂는 순서대로 `ttyUSB0`,
    `ttyUSB1` 이 붙어 **재부팅마다 달라짐.**

    `output` 은 센서에 저장된 설정과 같아야 함. 이 코드는 센서를 바꾸지 않음 --
    맞추는 것은 `huphy-imu apply` 가 함.
    """

    extra_fields = protocol.EXTRA_FIELDS
    """텔레메트리로 나가는 EBIMU 고유 값. `output` 과 무관하게 고정임."""

    def __init__(
        self,
        name: str,
        port: str,
        *,
        baudrate: int = DEFAULT_BAUDRATE,
        output: Sequence[str] = DEFAULT_OUTPUT,
    ) -> None:
        self.name = name
        self.port = port
        self.baudrate = int(baudrate)

        self.output = tuple(output)
        commands.validate(self.output)
        if "quat" not in self.output:
            raise ValueError(
                f"{name}: 자세는 쿼터니언이어야 함 (받은 output {list(self.output)}). "
                f"오일러는 회전 순서 규약을 알아야 해서 쓰지 않음"
            )
        self.field_count = commands.field_count(self.output)

        self._serial: Any = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        self._latest: Optional[ImuState] = None
        self.dropped = 0
        """읽었지만 버린 줄 수. 개수가 안 맞거나 `*` 로 시작 안 한 줄임."""

    def __repr__(self) -> str:
        where = self.port if self.is_connected else f"{self.port} (안 열림)"
        return f"EbimuImu({self.name}, {where}, {'+'.join(self.output)})"

    @property
    def is_connected(self) -> bool:
        return self._serial is not None

    # ---- 수명 -------------------------------------------------------------
    def connect(self) -> None:
        """포트를 열고 첫 패킷으로 구성을 확인함. 여러 번 불려도 안전함."""
        if self._serial is not None:
            return

        try:
            import serial  # noqa: PLC0415
        except ImportError as e:
            raise ImportError(
                f"IMU 를 읽으려면 pyserial 이 필요함. `pip install pyserial` ({e})"
            ) from e

        try:
            port = serial.Serial(
                port=self.port, baudrate=self.baudrate, timeout=READ_TIMEOUT_S
            )
        except Exception as e:
            raise ConnectionError(
                f"{self.name}: {self.port} 를 열 수 없음: {e}\n"
                f"  포트 확인   ls /dev/ttyUSB* /dev/ttyACM* /dev/serial*\n"
                f"  권한        sudo usermod -aG dialout $USER  (재로그인)"
            ) from e

        try:
            self._handshake(port)
        except Exception:
            port.close()
            raise

        self._serial = port
        self._running = True
        self._thread = threading.Thread(
            target=self._read_loop, name=f"ebimu-{self.name}", daemon=True
        )
        self._thread.start()
        logger.info("IMU 연결됨: %s", self)

    def _handshake(self, port: Any) -> None:
        """첫 패킷의 필드 수가 `output` 과 맞는지 확인함.

        안 맞으면 여기서 멈춤. 값이 한 칸씩 밀린 채로 제어를 시작하는 것보다
        연결이 안 되는 편이 나음.
        """
        port.reset_input_buffer()
        deadline = time.monotonic() + HANDSHAKE_S
        seen: List[int] = []

        while time.monotonic() < deadline:
            try:
                line = port.readline().decode("utf-8", errors="ignore").strip()
            except Exception as e:
                raise ConnectionError(f"{self.name}: 읽기 실패: {e}") from e
            if not line.startswith(protocol.PREFIX):
                continue

            count = len(protocol._floats(line[1:]))
            seen.append(count)
            if count == self.field_count:
                return
            # 첫 줄은 중간부터 잘려 들어올 수 있음. 두 줄까지는 더 봄.
            if len(seen) >= 3:
                break

        if not seen:
            raise ConnectionError(
                f"{self.name}: {HANDSHAKE_S:.0f}초 동안 패킷이 없음. "
                f"보레이트({self.baudrate})가 센서 설정과 같은지 확인할 것"
            )
        raise ConnectionError(
            f"{self.name}: 필드 수가 안 맞음 -- 받은 것 {seen}, "
            f"설정한 output {list(self.output)} 은 {self.field_count}개. "
            f"`huphy-imu show` 로 센서 설정을 확인하고 `huphy-imu apply` 로 맞출 것"
        )

    def disconnect(self) -> None:
        """스레드를 멈추고 포트를 닫음. 여러 번 불려도 안전함."""
        self._running = False
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=READ_TIMEOUT_S * 3)

        port, self._serial = self._serial, None
        if port is None:
            return
        try:
            port.close()
        except Exception as e:
            logger.warning("%s 종료 실패 (무시함): %s", self.name, e)

    def __enter__(self) -> "EbimuImu":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    # ---- 읽기 -------------------------------------------------------------
    def _read_loop(self) -> None:
        """계속 읽어 최신 값만 덮어씀. **예외로 죽지 않음.**

        센서 하나 때문에 제어가 멈추면 안 됨. 못 읽으면 그 줄을 세고 넘어가고,
        값이 안 들어오는 것은 `age_ms` 로 드러남.
        """
        port = self._serial
        while self._running:
            try:
                raw = port.readline()
            except Exception as e:
                if self._running:
                    logger.warning("%s 시리얼 읽기 실패: %s", self.name, e)
                    self._running = False
                return

            if not raw:
                continue                      # 타임아웃. 다음 바퀴에서 다시 봄

            line = raw.decode("utf-8", errors="ignore").strip()
            state = protocol.decode(line, self.output)
            if state is None:
                self.dropped += 1
                continue

            with self._lock:
                self._latest = state

    def read(self) -> ImuState:
        """가장 최근 값. **새로 통신하지 않음.**

        아직 아무것도 못 받았으면 `is_valid` 가 거짓인 상태를 냄. 예외를 던지지
        않음 -- 센서 하나 때문에 제어 루프가 멈추면 안 됨.
        """
        with self._lock:
            return self._latest if self._latest is not None else ImuState()
