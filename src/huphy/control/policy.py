"""학습한 정책을 `Motion` 으로 만듦.

모델은 관절 이름을 모름. **정해진 순서의 숫자 벡터**를 받고 같은 순서로 냄. 그
순서와 단위를 정하는 것이 이 파일의 일임.

    관찰 dict + IMU  ->  벡터  ->  모델  ->  벡터  ->  관절 목표 dict

`motions.py` 의 함수들과 같은 모양(`(t, observation) -> action`)이라 제어 루프가
그대로 받음. 텔레메트리·주기 측정·정지 순서가 같이 따라옴.


## 관찰 구성

시뮬(mjlab)이 이 순서로 이어 붙임. 가중치 파일의 입력 차원과 맞음.

    base_ang_vel        3    IMU 각속도 (rad/s)
    projected_gravity   3    중력 방향을 몸체 좌표로. IMU 모듈이 만들어 올림
    joint_pos           6    기본 자세 기준 상대 각도 (rad)
    joint_vel           6    (rad/s)
    actions             6    직전 주기에 낸 행동 그대로
    hop_phase           2    sin/cos. hopping 정책에만 있음

    balance  24    hopping  26


## 단위

    저장소   도, 도/초
    모델     라디안, rad/s

여기서 바꿈. 모델 경계가 단위가 갈리는 유일한 자리임.


## 정규화

학습 중에 쌓인 평균·표준편차가 가중치 파일에 같이 들어 있음.

    정규화된 입력 = (관찰 - mean) / std

**빼먹으면 모델이 전혀 다르게 동작함.** 모델을 부르는 쪽에서 적용해 넘길 것.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional, Sequence, Tuple

import numpy as np

JOINT_ORDER: Tuple[str, ...] = (
    "hip_pitch",
    "hip_roll",
    "hip_yaw",
    "knee",
    "ankle_pitch",
    "ankle_roll",
)
"""모델이 보는 관절 순서. **시뮬과 같아야 함.**

이름은 모델에 안 들어감 -- 순서만 들어감. 여기가 어긋나면 값은 다 정상인데 로봇이
엉뚱하게 움직이고, 프레임도 응답도 정상이라 코드로는 안 잡힘.
"""

ACTION_DIM = len(JOINT_ORDER)


@dataclass(frozen=True)
class PolicySpec:
    """정책 하나의 규격. 시뮬 설정에서 그대로 옮긴 값임."""

    name: str
    action_scale: float
    """관절 목표 = 기본자세 + `action_scale` x 행동. 시뮬의 `JointPositionActionCfg`."""

    obs_dim: int
    """관찰 벡터 길이. 가중치 파일의 입력 차원과 같아야 함."""

    uses_hop_phase: bool = False
    """`sin/cos` 위상 두 칸이 관찰 끝에 붙는지."""

    hop_period_s: float = 0.0
    """한 번 뛰는 데 걸리는 시간. `uses_hop_phase` 일 때만 씀."""


BALANCE = PolicySpec(name="balance", action_scale=0.25, obs_dim=24)
HOPPING = PolicySpec(
    name="hopping", action_scale=0.5, obs_dim=26, uses_hop_phase=True, hop_period_s=0.6
)




def hop_phase(t: float, period_s: float) -> Tuple[float, float]:
    """뛰는 주기 안에서 지금 어디인지. `(sin, cos)`.

    각도가 아니라 사인·코사인으로 주는 이유: 한 바퀴 돌 때 값이 튀지 않음. 위상이
    0.99 에서 0.0 으로 넘어가도 sin/cos 는 이어짐.
    """
    if period_s <= 0:
        return (0.0, 1.0)
    angle = 2.0 * math.pi * ((t / period_s) % 1.0)
    return (math.sin(angle), math.cos(angle))


def observation_vector(
    observation: Dict[str, Any],
    imu_state: Any,
    last_action: Sequence[float],
    *,
    spec: PolicySpec,
    t: float = 0.0,
) -> np.ndarray:
    """관찰을 모델 입력 벡터로. 길이는 `spec.obs_dim`.

    `last_action` 은 **모델이 직전에 낸 값 그대로**임 -- 관절 목표로 바꾸기 전의
    것. 시뮬의 `last_action` 이 그것임.
    """
    out = []

    # 각속도. IMU 가 도/초로 냄.
    out.extend(math.radians(v) for v in imu_state.gyro_dps)

    # 중력방향은 센서 모듈이 이미 만들어 둔 것을 씀. 센서가 오일러를 주든
    # 쿼터니언을 주든 여기는 모름 -- 형식을 아는 쪽에서 계산해 올림.
    out.extend(imu_state.gravity)

    # 관절 각도·속도. 기본 자세가 전부 0 이라 상대 각도가 곧 각도임.
    out.extend(math.radians(float(observation.get(f"{j}.pos", 0.0))) for j in JOINT_ORDER)
    out.extend(math.radians(float(observation.get(f"{j}.vel", 0.0))) for j in JOINT_ORDER)

    out.extend(float(v) for v in last_action)

    if spec.uses_hop_phase:
        out.extend(hop_phase(t, spec.hop_period_s))

    vector = np.asarray(out, dtype=np.float32)
    if vector.size != spec.obs_dim:
        raise ValueError(
            f"{spec.name}: 관찰이 {vector.size}개인데 {spec.obs_dim}개여야 함. "
            f"관절 수나 hop_phase 설정이 시뮬과 다름"
        )
    return vector


def joint_targets(action: Sequence[float], *, spec: PolicySpec) -> Dict[str, float]:
    """모델 출력 -> 관절 목표 각도 (도).

        목표 = 기본자세 + action_scale x 행동

    기본 자세가 전부 0 이라 두 번째 항만 남음.
    """
    if len(action) != ACTION_DIM:
        raise ValueError(
            f"{spec.name}: 행동이 {len(action)}개인데 {ACTION_DIM}개여야 함"
        )
    return {
        joint: math.degrees(spec.action_scale * float(value))
        for joint, value in zip(JOINT_ORDER, action)
    }


Model = Callable[[np.ndarray], Sequence[float]]
"""정규화까지 끝난 벡터를 받아 행동을 내는 것. 프레임워크를 가리지 않음."""


def policy_motion(
    model: Model,
    imu: Any,
    *,
    spec: PolicySpec = BALANCE,
) -> Callable[[float, Dict[str, Any]], Optional[Dict[str, float]]]:
    """정책을 `Motion` 으로 만듦. 제어 루프가 그대로 받음.

    직전 행동을 들고 있음 -- 관찰에 들어가기 때문임. 첫 주기에는 0 임.

    **`t` 는 이 동작이 시작한 시점부터임.** 상태 기계가 상태별 경과 시간을 넘기므로
    뛰는 위상이 상태에 들어간 순간부터 셈.
    """
    last_action = [0.0] * ACTION_DIM

    def motion(t: float, observation: Dict[str, Any]) -> Optional[Dict[str, float]]:
        vector = observation_vector(
            observation, imu.read(), last_action, spec=spec, t=t
        )
        action = model(vector)
        last_action[:] = [float(v) for v in action]
        return joint_targets(last_action, spec=spec)

    return motion
