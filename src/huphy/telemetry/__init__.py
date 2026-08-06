"""텔레메트리 — 무슨 일이 일어났는지 내보냄.

    snapshot.py   한 시점의 사전. **필드 이름을 정하는 유일한 곳**
    udp.py        UDP JSON 송신. 실시간으로 보는 것
    csv_log.py    CSV 기록. 끝나고 되돌아보는 것

둘 다 같은 스냅샷을 소비함. 두 군데에서 필드를 만들면 반드시 어긋남.

**제어를 방해하지 않음.** 어느 쪽도 예외를 던지지 않고, 실패는 세기만 함. 관측이
제어를 멈추면 관측할 대상이 없어짐.


## 팔다리마다 하나씩 둠

다리 하나가 필드 46개에 약 1.3 KB 임. 둘을 한 패킷에 합치면 이더넷 MTU(1500)를
넘어 조각나고, **조각 하나만 잃어도 패킷 전체가 버려짐.**

    Telemetry(left_leg,  host=...)    -> 패킷 하나
    Telemetry(right_leg, host=...)    -> 패킷 하나

PlotJuggler 는 여러 출처를 같은 타임라인에 올림. 팔·상체까지 붙으면 이 구성이
그대로 늘어남.

CSV 는 크기 제약이 없으므로 `merge()` 로 한 파일에 모아도 됨.
"""

from __future__ import annotations

import time
from typing import Any, Dict, Mapping, Optional

from . import csv_log, snapshot, udp
from .csv_log import CsvSink
from .snapshot import build, field_names, merge
from .udp import UdpSink

__all__ = [
    "snapshot",
    "udp",
    "csv_log",
    "UdpSink",
    "CsvSink",
    "Telemetry",
    "build",
    "field_names",
    "merge",
]


class Telemetry:
    """UDP 와 CSV 를 함께 다룸. 제어 루프가 이것만 부르면 됨.

    둘 다 꺼져 있어도 동작함 — 호출부가 분기하지 않아도 되게 함.
    """

    def __init__(
        self,
        robot: Any,
        *,
        host: Optional[str] = None,
        port: int = 9870,
        csv_path: Optional[str] = None,
        flush_every: int = csv_log.DEFAULT_FLUSH_EVERY,
    ) -> None:
        self.robot = robot
        self.fields = field_names(robot)
        self.udp = UdpSink(host, port)
        self.csv = CsvSink(csv_path, self.fields, flush_every=flush_every)
        self._t0: Optional[float] = None

    def __repr__(self) -> str:
        return f"Telemetry({self.udp}, {self.csv})"

    @classmethod
    def from_config(cls, robot: Any, config: Any) -> "Telemetry":
        """`TelemetryConfig` 로 만듦."""
        return cls(
            robot,
            host=config.host,
            port=config.port,
            csv_path=config.csv_path,
            flush_every=config.csv_flush_every,
        )

    @property
    def enabled(self) -> bool:
        return self.udp.enabled or self.csv.enabled

    def open(self) -> None:
        self.udp.open()
        self.csv.open()

    def close(self) -> None:
        """**남은 것을 반드시 밀어 넣음.**

        버퍼에 남은 몇 줄이 사라지면 하필 사고 직전 부분을 잃음.
        """
        self.udp.close()
        self.csv.close()

    def __enter__(self) -> "Telemetry":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def record(self, *, loop_dt_ms: float = 0.0, missing: int = 0) -> Dict[str, float]:
        """지금 상태를 찍어 내보냄. 만든 스냅샷을 돌려줌.

        시각은 **첫 호출을 0으로** 하는 상대 시간임. 벽시계를 쓰면 그래프의 x 축이
        1.7e9 같은 값에서 시작해 읽을 수 없음.
        """
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now

        data = build(
            self.robot, t=now - self._t0, loop_dt_ms=loop_dt_ms, missing=missing
        )
        self.emit(data)
        return data

    def emit(self, data: Mapping[str, float]) -> None:
        """이미 만든 스냅샷을 내보냄. 여러 팔다리를 `merge` 로 합쳤을 때 씀."""
        self.udp.send(data)
        self.csv.write(data)

    def as_fields(self) -> Dict[str, int]:
        """텔레메트리 자체의 상태. 로그로만 봄 — 나가는 경로가 고장났는데 그 사실을
        같은 경로로 알릴 수는 없음.
        """
        out = dict(self.udp.counters.as_fields())
        out.update(self.csv.counters.as_fields())
        return out
