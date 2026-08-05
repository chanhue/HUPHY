"""관절 한계와의 관계 계산 — 전부 순수 함수.

각도 하나가 한계와 어떤 관계인지를 **계산만** 한다. 그걸로 무엇을 할지(보낼까,
자를까, 경고할까)는 부르는 쪽이 정한다.

CAN도 상태도 모른다. 숫자를 받아 숫자를 돌려주므로 하드웨어 없이 테스트된다.


## 소비자

    guards.apply()        위치 제한 클리핑          <- 명령 경로
    telemetry             m{id}_margin 필드         <- 매 사이클
    브링업 메뉴            현재 자세가 정상인가       <- 사람이 봄

명령을 보내지 않는 상황에서도 한계와의 거리는 알아야 하므로 guards와 나눠 둔다.


## 공간 규약

모든 각도는 **raw 공간**이다 -- 모터가 보고하는 각도 그대로. sign/offset을 적용한
calibrated 공간이 아니다.

한계값은 무동력으로 하드스톱까지 밀어 raw를 읽어 얻으므로 raw가 자연스럽다.
사람에게 보여줄 때만 calibrated로 변환한다. (docs/issues.md #2)


## limits는 하드스톱 그 자체다

`(lo, hi)`는 **기계적으로 더 갈 수 없는 지점**의 실측값이다. 안전 여유가 이미
빠진 값이 아니다. 따라서 여유는 하드스톱에서 **안쪽 방향**으로 뺀다.

    하드스톱                                    하드스톱
       |---3도---|                    |---3도---|
       |      명령 허용 구간           |
       lo                            hi


## 왜 하드스톱까지 안 가고 여유를 두나

명령을 하드스톱에 정확히 두면 부딪힌다.

  1. 오버슛     PD 제어는 목표를 지나친다. kd가 충분해도 0은 아니다
  2. 관성       빠르게 움직이는 중에 명령을 멈춰도 바로 서지 않는다
  3. 측정 오차   하드스톱 실측이 실제보다 크면 여유 없이는 닿는다

여유는 이 셋을 흡수하는 공간이다.

**3도는 임의값이다.** 제대로 정하려면 게인 튜닝 후 목표를 한계 근처로 보내
텔레메트리에서 오버슛 크기를 재고, 거기에 안전계수를 더해야 한다.
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple

Limits = Tuple[float, float]


def safe_window(limits: Limits, margin_deg: float) -> Limits:
    """하드스톱에서 margin만큼 안쪽으로 좁힌 구간."""
    lo, hi = float(limits[0]), float(limits[1])
    m = abs(float(margin_deg))
    return lo + m, hi - m


def clamp(
    deg: float, limits: Optional[Limits], *, margin_deg: float
) -> Tuple[float, bool]:
    """한계 안으로 자른다. (자른 값, 잘렸는지)를 반환. limits가 None이면 통과.

    두 번째 값이 True면 호출부가 카운터를 올려 텔레메트리로 내보낸다.
    클리핑은 조용한 변조이므로 반드시 드러내야 한다.
    """
    if limits is None:
        return float(deg), False
    lo_safe, hi_safe = safe_window(limits, margin_deg)
    v = float(deg)
    if v < lo_safe:
        return lo_safe, True
    if v > hi_safe:
        return hi_safe, True
    return v, False


def margin_to_limit(deg: float, limits: Limits) -> float:
    """가까운 쪽 하드스톱까지 남은 여유(도). 넘었으면 음수.

    절대 각도보다 이 값이 직관적이다 -- 한계가 모터마다 다르고 비대칭이라
    (예: knee = -20.65 ~ 74.79) 절대각을 보면서 매번 머리로 빼야 한다.
    margin은 0선 하나만 보면 된다.
    """
    lo, hi = float(limits[0]), float(limits[1])
    v = float(deg)
    if v < lo:
        return v - lo          # 음수
    if v > hi:
        return hi - v          # 음수
    return min(v - lo, hi - v)


def closest_to_limit(
    values: Dict[int, float],
    limits: Dict[int, Optional[Limits]],
) -> Tuple[Optional[int], float]:
    """가장 한계에 가까운 모터와 그 여유. 대상이 없으면 (None, inf).

    id를 함께 돌려주는 것이 중요하다. 여럿 중 어느 관절이 문제인지 알아야
    원인을 찾을 수 있다.
    """
    worst_id: Optional[int] = None
    worst = float("inf")
    for motor_id, value in values.items():
        lim = limits.get(motor_id)
        if lim is None:
            continue
        m = margin_to_limit(value, lim)
        if m < worst:
            worst_id, worst = motor_id, m
    return worst_id, worst
