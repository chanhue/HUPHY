"""UDP 로 JSON 한 줄씩 보냄. PlotJuggler 가 받음.

**보내고 잊음.** 받는 쪽이 없어도, 느려도, 꺼져 있어도 제어 루프가 멈추지 않음 —
관측이 제어를 방해하면 안 됨.


## 왜 UDP 인가

TCP 는 상대가 안 받으면 **송신이 막힘.** 제어 루프 한가운데서 막히면 주기가 통째로
밀림. UDP 는 커널 버퍼에 넣고 바로 돌아옴.

패킷이 몇 개 빠져도 상관없음. 그래프에 점 하나가 비는 것뿐이고, 100Hz 로 보내므로
눈에 띄지도 않음.


## 패킷 크기

이더넷 MTU 는 1500 바이트임. 넘으면 조각나서 나가는데, **조각 하나만 잃어도 패킷
전체가 버려짐** — 손실률이 확 올라감.

소수점 둘째 자리로 반올림해 줄임. 그래도 넘으면 세어서 알림 — 조용히 조각나게 두면
왜 데이터가 듬성듬성해지는지 알 수 없음.


## 시간축

`t` 를 PlotJuggler 의 timestamp 필드로 지정할 것. 지정하지 않으면 **수신 시각**을
쓰게 되어 네트워크 지터가 데이터에 섞임.
"""

from __future__ import annotations

import json
import logging
import socket
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional

logger = logging.getLogger(__name__)

MTU_LIMIT = 1400
"""이 크기를 넘으면 조각날 위험이 있음. 1500 에서 헤더 몫을 뺀 값."""

ROUND_DIGITS = 2
"""소수점 자리. 0.01도는 모터 해상도(0.022도)보다 촘촘해서 정보를 잃지 않음."""


@dataclass
class UdpCounters:
    """무슨 일이 있었는지. 자체 진단용임.

    이 값들은 UDP 로 나가지 않음 — 나가는 경로가 고장났는데 그 사실을 같은 경로로
    알릴 수는 없음. 로그와 CSV 로 봄.
    """

    sent: int = 0
    errors: int = 0
    oversize: int = 0

    def as_fields(self) -> Dict[str, int]:
        return {
            "udp.sent": self.sent,
            "udp.errors": self.errors,
            "udp.oversize": self.oversize,
        }


class UdpSink:
    """UDP 소켓 하나. 스냅샷을 JSON 한 줄로 보냄.

    `host` 가 비어 있으면 **아무것도 하지 않음.** 설정에서 꺼 두는 것이 기본이고,
    끄고 켤 때 호출부가 분기하지 않아도 되게 함.
    """

    def __init__(
        self,
        host: Optional[str],
        port: int = 9870,
        *,
        round_digits: int = ROUND_DIGITS,
    ) -> None:
        self.host = host or None
        self.port = int(port)
        self.round_digits = int(round_digits)
        self.counters = UdpCounters()
        self._socket: Optional[socket.socket] = None

    def __repr__(self) -> str:
        target = f"{self.host}:{self.port}" if self.enabled else "꺼짐"
        return f"UdpSink({target})"

    @property
    def enabled(self) -> bool:
        return self.host is not None

    def open(self) -> None:
        """소켓을 만듦. **연결하지 않음** — UDP 라 상대가 없어도 됨."""
        if not self.enabled or self._socket is not None:
            return
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setblocking(False)
        logger.info("텔레메트리 UDP: %s:%d", self.host, self.port)

    def close(self) -> None:
        """여러 번 불려도 안전함."""
        if self._socket is None:
            return
        try:
            self._socket.close()
        finally:
            self._socket = None

    def __enter__(self) -> "UdpSink":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def send(self, snapshot: Mapping[str, Any]) -> bool:
        """한 패킷 보냄. 보냈으면 `True`.

        **예외를 던지지 않음.** 관측이 제어를 멈추면 안 됨 — 네트워크가 끊기거나
        상대가 꺼져 있는 것은 로봇 입장에서 정상 상황임.
        """
        if not self.enabled:
            return False
        if self._socket is None:
            self.open()

        payload = json.dumps(self._rounded(snapshot), separators=(",", ":")).encode()

        if len(payload) > MTU_LIMIT:
            self.counters.oversize += 1
            if self.counters.oversize == 1:
                logger.warning(
                    "텔레메트리 패킷이 %d 바이트임 (한계 %d). 조각나면 손실이 커짐 — "
                    "필드를 줄이거나 저주기 값을 별도 패킷으로 나눌 것",
                    len(payload), MTU_LIMIT,
                )

        try:
            self._socket.sendto(payload, (self.host, self.port))
        except OSError as e:
            self.counters.errors += 1
            if self.counters.errors == 1:
                logger.warning("텔레메트리 송신 실패 (이후로는 세기만 함): %s", e)
            return False

        self.counters.sent += 1
        return True

    def _rounded(self, snapshot: Mapping[str, Any]) -> Dict[str, Any]:
        """소수점을 줄임. 패킷 크기를 반으로 줄이는 데 이게 제일 큼."""
        out: Dict[str, Any] = {}
        for key, value in snapshot.items():
            out[key] = round(value, self.round_digits) if isinstance(value, float) else value
        return out
