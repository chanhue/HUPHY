"""CAN 전송 계층.

프레임을 **주고받기만** 함. 안에 든 바이트가 무슨 뜻인지 모름 — 그건 벤더 코덱이 앎.

`python-can` 을 import하는 **유일한 파일**임. 위 계층은 `CanFrame` 만 다루므로,
드라이버를 바꾸거나 테스트에서 가짜 버스를 끼울 때 이 파일만 보면 됨.
`codec/mit.py` 가 라디안 변환을 혼자 떠안는 것과 같은 방식임.


## 전송과 수거를 나눔

한 번에 묶어 두면 버스가 둘일 때 두 다리의 명령 시각이 벌어짐 (이슈 #10).

    보내기(can0) -> 보내기(can1) -> 수거(can0) -> 수거(can1)

이 순서가 되려면 `send_many` 와 `drain` 이 별개여야 함. 두 버스는 물리적으로
독립이라 전송이 진짜로 겹침.

수거가 더 비쌈. `recv()` 는 큐가 비면 타임아웃만큼 블로킹하므로, 순차로 수거하면
그 시간이 버스 수만큼 곱해짐.


## 드레인 비용

100Hz 루프의 예산은 10ms 임. 그 안에서 `recv(timeout=2ms)` 를 한 번 부르고 큐가
비어 있으면 **2ms 를 그냥 버림.**

그래서 짧게 여러 번 폴링하고, 기다릴 총 시간을 따로 둠. 기대한 개수가 다 오면 즉시
빠져나감 — 정상 상황에서는 총 시간을 다 쓰지 않음.


## 스레드

지금은 제어 루프 한 곳에서만 부름. 송신에만 락을 걸어 프레임이 섞이지 않게 함.

**수신 전용 스레드를 두면 이 가정이 깨짐.** 그때 락 구성을 다시 봐야 함 —
slcan(시리얼)은 송수신이 같은 포트를 쓰므로 하나의 락으로 묶어야 함.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence

logger = logging.getLogger(__name__)

DEFAULT_BITRATE = 1_000_000
"""RobStride 기본값. 버스에 물린 모든 노드가 같아야 함."""

DEFAULT_POLL_S = 0.0002
"""recv 한 번의 대기 시간. 짧게 여러 번 도는 쪽이 한 번 길게 기다리는 것보다 나음."""

DEFAULT_DRAIN_S = 0.002
"""수거에 쓸 총 시간. 100Hz 예산 10ms 의 20%."""


@dataclass(frozen=True)
class CanFrame:
    """CAN 프레임 하나. 바이트의 뜻은 모름.

    `python-can` 의 `Message` 를 그대로 쓰지 않는 이유: 그걸 위 계층에 흘리면
    `python-can` 이 없는 환경에서 import가 깨지고 테스트도 못 돌림.
    """

    can_id: int
    data: bytes
    is_extended: bool = False
    """MIT 프로토콜은 11-bit 표준 프레임이라 `False`. private 은 29-bit 확장임."""

    stamp: float = 0.0
    """수신 시각. 보낼 때는 쓰지 않음."""


@dataclass
class CanCounters:
    """무슨 일이 있었는지 세는 곳. 텔레메트리로 나감.

    한 번의 실패는 흔한 일이라 예외를 던지지 않음. **몇 번 연속인지**가 의미 있고,
    그 판단은 호출부가 함.
    """

    tx_errors: int = 0
    rx_errors: int = 0
    frames_sent: int = 0
    frames_received: int = 0
    drain_timeouts: int = 0
    """기대한 개수를 못 채우고 총 시간을 다 쓴 횟수."""

    def reset(self) -> None:
        self.tx_errors = 0
        self.rx_errors = 0
        self.frames_sent = 0
        self.frames_received = 0
        self.drain_timeouts = 0

    def as_fields(self) -> Dict[str, int]:
        """0이어도 모든 키를 냄. 필드가 중간에 생기면 그래프가 끊김."""
        return {
            "can.tx_errors": self.tx_errors,
            "can.rx_errors": self.rx_errors,
            "can.frames_sent": self.frames_sent,
            "can.frames_received": self.frames_received,
            "can.drain_timeouts": self.drain_timeouts,
        }


def _guess_interface(channel: str) -> str:
    """채널 이름으로 드라이버를 추측함.

        /dev/tty.usbmodem...  ->  slcan     시리얼 어댑터 (macOS 개발용)
        can0, can1            ->  socketcan 리눅스 커널 (로봇 본체)

    맞히지 못하면 설정에서 명시하면 됨.
    """
    return "slcan" if channel.startswith("/dev/") else "socketcan"


class CanBus:
    """CAN 채널 하나. 다리 하나가 버스 하나를 씀.

    되돌리기 어려운 조작은 여기 없음 — 이 계층은 바이트의 뜻을 모르므로 애초에
    그런 명령을 만들 수 없음.
    """

    def __init__(
        self,
        channel: str,
        *,
        interface: Optional[str] = None,
        bitrate: int = DEFAULT_BITRATE,
    ) -> None:
        self.channel = channel
        self.interface = interface or _guess_interface(channel)
        self.bitrate = int(bitrate)

        self._bus = None
        self._tx_lock = threading.Lock()
        self.counters = CanCounters()

    def __repr__(self) -> str:
        state = "연결됨" if self.is_connected else "끊김"
        return f"CanBus({self.channel!r}, {self.interface}, {self.bitrate}bps, {state})"

    # ---- 수명 -------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self._bus is not None

    def connect(self) -> None:
        """채널을 엶. `python-can` 은 여기서 처음 import됨.

        모듈 최상단에서 import하면 이 파일을 import하는 것만으로 `python-can` 이
        필요해짐. 순수 계산 계층의 테스트까지 같이 막힘.
        """
        if self.is_connected:
            return

        try:
            import can  # noqa: PLC0415
        except ImportError as e:
            raise ImportError(
                "CAN 통신에는 python-can 이 필요함. `pip install python-can` "
                "(순수 계산 계층은 이것 없이도 동작함)"
            ) from e

        try:
            self._bus = can.interface.Bus(
                channel=self.channel, interface=self.interface, bitrate=self.bitrate
            )
        except Exception as e:
            raise ConnectionError(
                f"{self.channel} ({self.interface}) 를 열 수 없음: {e}"
            ) from e

        logger.info("CAN 연결됨: %s", self)

    def disconnect(self) -> None:
        """채널을 닫음. 여러 번 불려도 안전함."""
        if self._bus is None:
            return
        try:
            self._bus.shutdown()
        except Exception as e:
            logger.warning("%s 종료 중 예외 (무시함): %s", self.channel, e)
        finally:
            self._bus = None

    def __enter__(self) -> "CanBus":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    def _require(self):
        if self._bus is None:
            raise ConnectionError(f"{self.channel} 이 연결되지 않음. connect() 를 먼저 부를 것")
        return self._bus

    # ---- 송신 -------------------------------------------------------------
    def send(self, frame: CanFrame) -> bool:
        """한 프레임을 보냄. 실패하면 세고 `False` 를 반환함."""
        return self.send_many([frame]) == 1

    def send_many(self, frames: Sequence[CanFrame]) -> int:
        """여러 프레임을 **연속으로** 보냄. 보낸 개수를 반환함.

        락을 한 번만 잡음. 프레임마다 잡았다 놓으면 그 사이에 다른 프레임이 끼어들어
        한 관절의 명령들이 버스에서 흩어짐.

        중간에 실패해도 나머지를 계속 보냄. 다리 하나가 6모터인데 첫 프레임이
        실패했다고 멈추면 나머지 5개가 직전 명령을 유지해 자세가 어긋남.
        """
        bus = self._require()
        import can  # noqa: PLC0415

        sent = 0
        with self._tx_lock:
            for f in frames:
                msg = can.Message(
                    arbitration_id=f.can_id,
                    data=f.data,
                    is_extended_id=f.is_extended,
                )
                try:
                    bus.send(msg)
                    sent += 1
                except Exception as e:
                    self.counters.tx_errors += 1
                    logger.debug("%s 송신 실패 id=0x%X: %s", self.channel, f.can_id, e)
        self.counters.frames_sent += sent
        return sent

    # ---- 수신 -------------------------------------------------------------
    def drain(
        self,
        *,
        expect: Optional[int] = None,
        timeout_s: float = DEFAULT_DRAIN_S,
        poll_s: float = DEFAULT_POLL_S,
        max_frames: int = 4096,
    ) -> List[CanFrame]:
        """도착한 프레임을 모음.

        `expect` 개를 채우면 **즉시** 빠져나감 — 정상 상황에서는 `timeout_s` 를 다
        쓰지 않음. `None` 이면 버스가 조용해질 때까지 읽음.

        예외를 던지지 않음. 한 주기에 프레임이 덜 오는 것은 흔한 일이고, 그때마다
        제어 루프가 죽으면 안 됨. 몇 개 왔는지는 반환값 길이로 알 수 있음.
        """
        bus = self._require()
        out: List[CanFrame] = []
        deadline = time.monotonic() + max(0.0, float(timeout_s))

        # 마감은 **기다릴 때만** 봄. 이미 큐에 있는 것은 마감과 무관하게 다 읽음.
        # 읽기 전에 마감을 보면 timeout_s=0 일 때 한 프레임도 못 꺼냄.
        while len(out) < max_frames:
            if expect is not None and len(out) >= expect:
                break                       # expect=0 이면 recv 자체를 부르지 않음
            try:
                msg = bus.recv(timeout=poll_s)
            except Exception as e:
                self.counters.rx_errors += 1
                logger.debug("%s 수신 실패: %s", self.channel, e)
                break

            if msg is not None:
                out.append(
                    CanFrame(
                        can_id=int(msg.arbitration_id),
                        data=bytes(msg.data),
                        is_extended=bool(msg.is_extended_id),
                        stamp=time.monotonic(),
                    )
                )
                continue

            # 큐가 비었음
            if expect is None:
                break                       # 조용해짐 -- 다 받은 것으로 봄
            if time.monotonic() >= deadline:
                self.counters.drain_timeouts += 1
                break

        self.counters.frames_received += len(out)
        return out

    def flush_rx(self, *, max_frames: int = 4096) -> int:
        """큐에 남은 것을 버리고 개수를 반환함.

        직전 주기의 응답이 남아 있으면 이번 주기의 것으로 오해함. 특히 고장 조회는
        일반 상태 프레임과 CAN ID 가 같아 구분되지 않으므로, 보내기 전에 비워야 함.
        """
        return len(self.drain(timeout_s=0.0, poll_s=0.0, max_frames=max_frames))


def drain_all(
    buses: Iterable[CanBus],
    *,
    expect_per_bus: Optional[int] = None,
    timeout_s: float = DEFAULT_DRAIN_S,
) -> Dict[str, List[CanFrame]]:
    """여러 버스를 수거함. 채널 이름으로 묶어 반환함.

    지금은 순차라 총 대기가 버스 수만큼 곱해짐 (이슈 #10). 양다리에서 문제가 되면
    버스마다 수신 스레드를 두어 제어 루프에서 수거를 없애야 함.

    그래도 이 함수를 두는 이유: 호출부가 "전부 보내고 전부 수거" 순서를 지키게 됨.
    버스별로 보내고 바로 수거하면 두 다리의 명령 시각이 벌어짐.
    """
    return {
        b.channel: b.drain(expect=expect_per_bus, timeout_s=timeout_s) for b in buses
    }
