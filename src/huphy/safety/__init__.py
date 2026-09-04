"""안전 로직 — 전부 순수 함수, 상태 없음, 하드웨어 무관.

CAN도 모르고 락도 없으므로 하드웨어 없이 단위 테스트할 수 있다.
"""

from .guards import (
    ClipReason,
    GuardCounters,
    GuardResult,
    RejectReason,
    apply,
    clamp_jump,
    is_finite,
)
from .limits import Limits, clamp, closest_to_limit, margin_to_limit, safe_window
from .link import DEFAULT_MAX_MISS_CYCLES, LinkLoss, LinkWatch

__all__ = [
    # limits — 한계와의 관계 계산
    "Limits",
    "safe_window",
    "clamp",
    "margin_to_limit",
    "closest_to_limit",
    # guards — 명령의 최종 관문
    "apply",
    "is_finite",
    "clamp_jump",
    "GuardResult",
    "GuardCounters",
    "ClipReason",
    # link — 응답이 오고 있나
    "LinkWatch",
    "LinkLoss",
    "DEFAULT_MAX_MISS_CYCLES",
    "RejectReason",
]
