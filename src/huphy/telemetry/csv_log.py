"""CSV 로 남김. 나중에 다시 보는 기록.

UDP 는 실시간으로 보는 것이고, 이쪽은 **끝나고 나서 되돌아보는 것**임. 사고가 났을
때 그 직전 몇 초를 프레임 단위로 다시 볼 수 있어야 함.


## 열이 실행 전에 정해짐

`snapshot.field_names(robot)` 로 헤더를 먼저 씀. 첫 줄을 쓰고 나면 열이 고정됨 —
중간에 필드가 생기면 그 값은 버리고 세기만 함.

**열이 밀리면 기록 전체가 못 쓰게 됨.** 나중에 열어 보면 어느 열이 무엇인지 알 수
없고, 그걸 알아채는 것은 보통 사고 조사 중임.


## 디스크 쓰기가 주기를 흔듦

매 줄마다 디스크에 밀어 넣으면 제어 주기가 튐. `flush_every` 주기마다 한 번만 밀어
넣고, 나머지는 파이썬 버퍼에 쌓아 둠.

**멈출 때는 반드시 밀어 넣음.** 버퍼에 남은 것이 사라지면 사고 직전 몇 줄을 잃는데,
하필 그게 제일 보고 싶은 부분임.
"""

from __future__ import annotations

import csv
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence

logger = logging.getLogger(__name__)

DEFAULT_FLUSH_EVERY = 50
"""100Hz 에서 0.5초마다 한 번. 잃을 수 있는 양과 주기 흔들림의 절충."""


@dataclass
class CsvCounters:
    rows: int = 0
    dropped_fields: int = 0
    """헤더에 없어서 버린 값의 수. 0이 아니면 스냅샷이 도중에 바뀐 것임."""

    def as_fields(self) -> Dict[str, int]:
        return {"csv.rows": self.rows, "csv.dropped_fields": self.dropped_fields}


class CsvSink:
    """CSV 파일 하나.

    `path` 가 비어 있으면 **아무것도 하지 않음.** 끄고 켤 때 호출부가 분기하지
    않아도 되게 함.
    """

    def __init__(
        self,
        path: "Optional[str | Path]",
        fields: Sequence[str],
        *,
        flush_every: int = DEFAULT_FLUSH_EVERY,
    ) -> None:
        self.path = Path(path) if path else None
        self.fields = tuple(fields)
        self.flush_every = max(1, int(flush_every))
        self.counters = CsvCounters()
        self._file = None
        self._writer = None
        self._since_flush = 0

    def __repr__(self) -> str:
        target = str(self.path) if self.enabled else "꺼짐"
        return f"CsvSink({target}, 열 {len(self.fields)}개)"

    @property
    def enabled(self) -> bool:
        return self.path is not None

    def open(self) -> None:
        """파일을 열고 헤더를 씀. **덮어씀.**

        이어 쓰지 않는 이유: 열 구성이 실행마다 다를 수 있는데(모터를 추가하거나
        관절 이름을 바꾸면) 이어 쓰면 한 파일 안에 두 형식이 섞임.
        """
        if not self.enabled or self._file is not None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.fields)
        self._since_flush = 0
        logger.info("텔레메트리 CSV: %s (열 %d개)", self.path, len(self.fields))

    def close(self) -> None:
        """남은 것을 밀어 넣고 닫음. 여러 번 불려도 안전함."""
        if self._file is None:
            return
        try:
            self._file.flush()
        except Exception as e:
            logger.warning("CSV flush 실패: %s", e)
        try:
            self._file.close()
        finally:
            self._file = None
            self._writer = None

    def __enter__(self) -> "CsvSink":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def write(self, snapshot: Mapping[str, Any]) -> bool:
        """한 줄 씀. 썼으면 `True`.

        **예외를 던지지 않음.** 디스크가 가득 차거나 매체가 빠져도 제어 루프는
        계속 돌아야 함.

        헤더에 없는 키는 버리고 셈. 열 순서를 지키는 것이 값 하나보다 중요함.
        """
        if not self.enabled:
            return False
        if self._file is None:
            self.open()

        extra = len(snapshot) - sum(1 for f in self.fields if f in snapshot)
        if extra > 0:
            self.counters.dropped_fields += extra
            if self.counters.dropped_fields == extra:
                logger.warning(
                    "헤더에 없는 필드 %d개를 버림. 스냅샷 구성이 실행 중에 바뀜 — "
                    "field_names() 와 build() 가 어긋났을 수 있음", extra,
                )

        try:
            self._writer.writerow(
                [_format(snapshot.get(name, "")) for name in self.fields]
            )
        except Exception as e:
            logger.warning("CSV 쓰기 실패: %s", e)
            return False

        self.counters.rows += 1
        self._since_flush += 1
        if self._since_flush >= self.flush_every:
            self._since_flush = 0
            try:
                self._file.flush()
            except Exception as e:
                logger.warning("CSV flush 실패: %s", e)
        return True


def _format(value: Any) -> Any:
    """소수점을 줄임. 파일 크기가 절반 이하가 됨.

    UDP 보다 한 자리 더 남김 — 패킷 크기 제약이 없고, 나중에 미분해서 볼 때
    반올림 오차가 커짐.
    """
    if isinstance(value, float):
        return f"{value:.3f}"
    return value
