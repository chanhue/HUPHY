"""관절 한계 판정과 클리핑 — 전부 순수 함수.

CAN도 상태도 모른다. 숫자를 받아 숫자나 참/거짓을 돌려주므로 하드웨어 없이
테스트된다.


## 공간 규약

이 모듈의 모든 각도는 **raw 공간**이다 -- 모터가 보고하는 각도 그대로.
sign/offset을 적용한 calibrated 공간이 아니다.

한계값은 무동력으로 하드스톱까지 밀어 raw를 읽어 얻으므로 raw가 자연스럽다.
사람에게 보여줄 때만 calibrated로 변환한다. (docs/issues.md #2)


## limits는 하드스톱 그 자체다

`(lo, hi)`는 **기계적으로 더 갈 수 없는 지점**의 실측값이다. 안전 여유가 이미
빠진 값이 아니다.

따라서 세 여유가 전부 **하드스톱에서 안쪽 방향**이다.

    하드스톱                                              하드스톱
       |                                                    |
       |-1도-|--3도--|----8도----          ----|--|--|-------|
       | state command  near_stop                           |
       |   |     |         |
       | E-STOP 클리핑   감쇠전환

중앙에서 바깥으로 나가며 만나는 순서:

    1. near_stop (8도 남음)  감쇠 전용으로 전환 (kp=0). 부드럽게 감속
    2. command   (3도 남음)  명령을 여기까지만 클리핑
    3. state     (1도 남음)  E-STOP. 가드를 뚫고 여기까지 왔으면 사고
    4. 하드스톱   (0도)      충돌

그래서 순서는 state <= command <= near_stop 이다 (전부 안쪽 거리).

E-STOP이 가장 바깥인 것은 역할이 다르기 때문이다:
  - command 클리핑은 **예방** -- 그쪽으로 명령을 안 보낸다
  - state E-STOP은 **사후** -- 외력·중력으로 밀려 가드를 뚫었을 때의 최후 수단


## 거부가 아니라 클리핑이다

한계를 넘는 명령을 **버리지 않고 한계까지만 보낸다.**

버리면 그 모터만 직전 명령을 유지해 **다리 자세가 어긋난다** -- 발목처럼 2모터가
연동된 곳에서 특히 나쁘다. 클리핑하면 연속성이 유지되고, 정책이 계속 밖을
요구해도 한계에 붙어 있을 뿐 자세가 깨지지 않는다.

다만 클리핑은 **조용한 변조**이므로 잘렸는지를 반드시 함께 돌려준다.
호출부가 세어서 텔레메트리로 내보낸다.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

Limits = Tuple[float, float]


@dataclass(frozen=True)
class LimitMargins:
    """하드스톱에서 안쪽으로 얼마인지. 전부 같은 방향이다."""

    state_deg: float = 1.0        # 이 안으로 들어오면 E-STOP
    command_deg: float = 3.0      # 명령을 여기까지만
    near_stop_deg: float = 8.0    # 이 안으로 들어오면 감쇠 전용

    def validate(self) -> None:
        """순서를 검사한다. 설정을 읽자마자 부를 것."""
        if not (self.state_deg <= self.command_deg <= self.near_stop_deg):
            raise ValueError(
                f"여유 순서가 잘못됐다: state({self.state_deg}) <= "
                f"command({self.command_deg}) <= near_stop({self.near_stop_deg}) "
                f"이어야 한다. 전부 하드스톱에서 안쪽 거리다."
            )

    def validate_against(self, limits: Limits, motor_id: int) -> None:
        """가동범위가 여유를 감당할 만큼 넓은지. 설정 로드 시 부를 것.

        near_stop이 양쪽에 적용되므로 가동범위가 그 2배보다 넓어야 움직일 구간이
        남는다.
        """
        span = abs(float(limits[1]) - float(limits[0]))
        need = 2.0 * self.near_stop_deg
        if span <= need:
            raise ValueError(
                f"m{motor_id}: 가동범위 {span:.1f}도가 near_stop 여유 2배({need:.1f}도)"
                f"보다 좁다. 여유를 줄이거나 관절별 여유를 둘 것."
            )


# ---------------------------------------------------------------------------
# 핵심 — 세 판정이 전부 이 형태다. 여유만 바꿔 넣는다.
# ---------------------------------------------------------------------------
def safe_window(limits: Limits, margin_deg: float) -> Limits:
    """하드스톱에서 margin만큼 안쪽으로 좁힌 구간.

    가동범위가 여유의 2배보다 좁아 구간이 뒤집히면 중점 하나로 접힌다.
    (설정 로드 시 validate_against가 잡아야 할 상황이지만, 여기서도 무너지지
     않게 한다 -- 제어 루프에서 예외를 던지면 안 된다)
    """
    lo, hi = float(limits[0]), float(limits[1])
    m = abs(float(margin_deg))
    lo_safe, hi_safe = lo + m, hi - m
    if lo_safe > hi_safe:
        mid = 0.5 * (lo + hi)
        return mid, mid
    return lo_safe, hi_safe


def within(deg: float, limits: Optional[Limits], *, margin_deg: float) -> bool:
    """하드스톱에서 margin 이상 떨어져 있는가. limits가 None이면 항상 True.

    여유만 바꿔 세 곳에 쓴다:
        margin = state_deg      -> E-STOP 아님
        margin = command_deg    -> 명령 허용
        margin = near_stop_deg  -> 감쇠 전환 아님
    """
    if limits is None:
        return True
    lo_safe, hi_safe = safe_window(limits, margin_deg)
    return lo_safe <= float(deg) <= hi_safe


def clamp(
    deg: float, limits: Optional[Limits], *, margin_deg: float
) -> Tuple[float, bool]:
    """한계 안으로 자른다. (자른 값, 잘렸는지)를 반환.

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


# ---------------------------------------------------------------------------
# 관측용
# ---------------------------------------------------------------------------
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
    """가장 한계에 가까운 모터와 그 여유. 없으면 (None, inf).

    id를 함께 돌려주는 것이 중요하다. 모터 하나만 걸려도 다리 전체가 감쇠로
    전환되므로, "왜 갑자기 힘이 빠졌나"의 범인을 알려면 어느 모터인지가 필요하다.
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
