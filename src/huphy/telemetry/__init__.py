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
from .snapshot import (
    build,
    build_diag,
    build_fast,
    build_imu,
    diag_field_names,
    fast_field_names,
    field_names,
    imu_field_names,
    merge,
)
from .udp import UdpSink

__all__ = [
    "snapshot",
    "udp",
    "csv_log",
    "UdpSink",
    "CsvSink",
    "Telemetry",
    "build",
    "build_fast",
    "build_diag",
    "build_imu",
    "field_names",
    "fast_field_names",
    "diag_field_names",
    "imu_field_names",
    "merge",
]

DEFAULT_DIAG_EVERY = 10
"""진단 패킷을 N주기마다 보냄. 100Hz 에서 10Hz 임.

`temp` 는 초 단위로 변하고 카운터는 사건이 있을 때만 변함. 매 주기 보낼 이유가
없고, 합치면 패킷이 MTU 를 넘음.
"""


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
        diag_every: int = DEFAULT_DIAG_EVERY,
    ) -> None:
        self.robot = robot
        self.fields = field_names(robot)
        self.has_imu = bool(getattr(robot, "imus", ()))
        """IMU 가 붙었는지. 없으면 그 패킷은 아예 안 보냄 -- `t` 만 든 패킷을
        100Hz 로 보내는 것은 낭비임. CSV 열도 `t` 하나뿐이라 늘지 않음.
        """
        self.diag_every = max(1, int(diag_every))
        self.udp = UdpSink(host, port)
        self.csv = CsvSink(csv_path, self.fields, flush_every=flush_every)
        self._t0: Optional[float] = None
        self._cycle = 0

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

    def record(self, *, loop_dt_ms: float = 0.0) -> Dict[str, float]:
        """지금 상태를 찍어 내보냄. **CSV 한 줄에 해당하는 전체 사전**을 돌려줌.

        시각은 **첫 호출을 0으로** 하는 상대 시간임. 벽시계를 쓰면 그래프의 x 축이
        1.7e9 같은 값에서 시작해 읽을 수 없음.

        UDP 는 두 패킷으로 나가고 주기가 다름.

            빠른 것   매번
            진단      `diag_every` 주기마다

        진단 값은 **매번 계산함** — 사전 만드는 비용은 무시할 만하고, CSV 는 매 줄에
        다 있어야 나중에 대조하기 쉬움. 나누는 것은 보내는 쪽뿐임.
        """
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0

        fast = build_fast(self.robot, t=t, loop_dt_ms=loop_dt_ms)
        diag = build_diag(self.robot, t=t)
        imu = build_imu(self.robot, t=t)

        self.udp.send(fast)
        if self._cycle % self.diag_every == 0:
            self.udp.send(diag)
        if self.has_imu:
            self.udp.send(imu)
        self._cycle += 1

        row = dict(fast)
        row.update(diag)
        row.update(imu)
        self.csv.write(row)
        return row

    def emit(self, data: Mapping[str, float]) -> None:
        """이미 만든 사전을 그대로 내보냄. 여러 팔다리를 `merge` 로 합쳤을 때 씀.

        **UDP 로는 합친 것을 보내지 말 것** — MTU 를 넘음. CSV 전용으로 쓸 것.
        """
        self.udp.send(data)
        self.csv.write(data)

    def as_fields(self) -> Dict[str, int]:
        """텔레메트리 자체의 상태. 로그로만 봄 — 나가는 경로가 고장났는데 그 사실을
        같은 경로로 알릴 수는 없음.
        """
        out = dict(self.udp.counters.as_fields())
        out.update(self.csv.counters.as_fields())
        return out
