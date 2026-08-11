"""발목 2모터 링키지 — 관절각 <-> 모터각.

순수 계산임. 모터도 CAN 도 모르고 숫자만 다룸.


## 왜 필요한가

발목은 모터 하나가 관절 하나를 돌리는 구조가 아님. **모터 두 개가 로드 두 개로
발판 하나를 밀고 당김.**

    a1 ↑  a2 ↑     ->  발끝이 내려감 (pitch)
    a1 ↑  a2 ↓     ->  발이 옆으로 기움 (roll)

그래서 "pitch 를 10도" 라는 명령이 모터 각도 두 개로 풀려야 하고, 반대로 모터 각도
두 개를 읽으면 지금 pitch/roll 이 얼마인지 계산해야 함.

    solve_ik    (pitch, roll)  ->  (a1, a2)      역기구학
    solve_fk    (a1, a2)       ->  (pitch, roll) 순기구학


## 두 방향의 비용이 다름

`solve_ik` 는 닫힌 해임 — 삼각함수 몇 번으로 끝남.

`solve_fk` 는 **뉴턴 반복**임. 로드 길이 두 개가 만족되는 (pitch, roll) 을 수치로
찾아감. 제어 루프에서 매 주기 부르면 비쌈.

명령은 IK 로 내려가고, FK 는 사람에게 보여줄 때만 쓰는 것이 기본임.


## 각도 규약

두 모터 각도를 **모두 `[-180, 180)`** 으로 냄. 모터가 `zero_sta = 1` 로 그 범위를
보고하기 때문임. 한쪽만 `[0, 360)` 으로 두면 IK 가 340도를 돌려주고 모터는 -20도를
보고해 **360도 차이**가 남 (이슈 #1).


## 기하값의 출처

아래 좌표는 **실물 발목 하나를 실측한 값**임. 어느 쪽 다리인지 확인되지 않았고,
반대쪽은 좌우 대칭을 가정한 거울상임 — 실측이 아님 (이슈 #13).

단위는 mm, 원점은 다리 상단 기준임.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Tuple

import numpy as np

Vec3 = np.ndarray

FK_VERIFY_TOL_DEG = 1e-3
"""FK 결과를 IK 로 되짚었을 때 허용하는 오차. 수치 반복의 잔차보다 넉넉함."""

SINGULAR_EPS = 1e-9
"""야코비안 분모가 이보다 작으면 특이점으로 봄.

분모가 0이 되는 것은 **로드가 크랭크 원에 접하는 자세**임. 그 자세에서는 모터를
아무리 돌려도 발판이 그 방향으로 움직이지 않으므로, 필요한 토크가 무한대로 감.
"""

MAX_JACOBIAN_CONDITION = 50.0
"""`joint_torque_to_motor` 가 거부하는 조건수.

조건수가 크면 **자세 측정 오차가 토크로 증폭됨.** 엔코더 잡음 수준과 모터 토크
한계를 보고 정할 값이고, 50 은 잰 값이 아니라 출발점임.

지금 기하에서는 도달 범위 안 최대가 12 정도라 걸리지 않음 -- 그보다 먼저 로드 해가
없어짐. 기하가 바뀌면 그때 의미가 생김.
"""


class AnkleUnreachableError(ValueError):
    """요청한 (pitch, roll) 에 대응하는 모터 각도가 없음.

    조용히 자르지 않고 던지는 이유: 발목은 두 모터가 물려 있어 **한쪽만 잘리면
    관절이 비틀림.** 무엇을 보낼지 상위가 정해야 함.
    """


@dataclass(frozen=True)
class AnkleGeometry:
    """실측 좌표. 코드가 아니라 데이터임.

    `A1`, `A2`   모터 회전축의 위치
    `origin`     발목 관절의 회전 중심
    `C1`, `C2`   로드가 발판에 붙는 지점 (중립 자세에서)
    `crank_r`    모터 디스크 반지름
    `crank_t`    디스크가 축에서 x 방향으로 떨어진 거리

    모터 회전축은 x 축이고 디스크는 y-z 평면에 놓임.
    """

    a1: Tuple[float, float, float] = (-26.10, 74.00, -255.00)
    a2: Tuple[float, float, float] = (-80.90, 74.00, -313.00)
    origin: Tuple[float, float, float] = (-53.50, 74.00, -393.00)
    c1: Tuple[float, float, float] = (-8.10, 111.343, -398.00)
    c2: Tuple[float, float, float] = (-98.90, 111.343, -398.00)

    crank_r: float = 40.0
    crank_t: float = 18.0

    offset_sign_1: float = 1.0
    offset_sign_2: float = -1.0
    """디스크가 축의 어느 쪽에 붙어 있는지."""

    rotation_sign_1: float = 1.0
    rotation_sign_2: float = -1.0
    """모터 회전 방향. 두 모터가 마주 보게 달려 있어 부호가 반대임."""

    def mirrored(self) -> "AnkleGeometry":
        """x 를 뒤집은 거울상. **실측이 아니라 좌우 대칭 가정임** (이슈 #13).

        좌표만 뒤집으면 안 됨. `crank_t` 는 x 방향 오프셋이라 부호가 같이 뒤집혀야
        하고, 모터 회전축이 x 축이므로 yz 평면 반사에서 회전 방향도 뒤집힘.

        제대로 뒤집으면 다음이 성립함.

            거울상.solve_ik(pitch, -roll) == -원본.solve_ik(pitch, roll)

        좌표만 뒤집으면 이 관계가 근사로만 맞아 좌우 다리가 미묘하게 다르게 움직임.
        """
        flip = lambda p: (-p[0], p[1], p[2])  # noqa: E731
        return AnkleGeometry(
            a1=flip(self.a1),
            a2=flip(self.a2),
            origin=flip(self.origin),
            c1=flip(self.c1),
            c2=flip(self.c2),
            crank_r=self.crank_r,
            crank_t=self.crank_t,
            offset_sign_1=-self.offset_sign_1,
            offset_sign_2=-self.offset_sign_2,
            rotation_sign_1=-self.rotation_sign_1,
            rotation_sign_2=-self.rotation_sign_2,
        )


@dataclass(frozen=True)
class AnkleEnvelope:
    """소프트웨어 시험 범위. 링키지가 닿는 범위와는 별개임.

    링키지는 더 넓게 움직일 수 있지만, 브링업 단계에서는 좁게 잡아 둠. 넓히려면
    이 값을 올림.
    """

    roll_deg: Tuple[float, float] = (-25.0, 25.0)
    pitch_deg: Tuple[float, float] = (-40.0, 40.0)

    def check(self, pitch_deg: float, roll_deg: float) -> None:
        if not self.roll_deg[0] <= roll_deg <= self.roll_deg[1]:
            raise AnkleUnreachableError(
                f"roll={roll_deg:.2f} 가 시험 범위 {self.roll_deg} 밖임"
            )
        if not self.pitch_deg[0] <= pitch_deg <= self.pitch_deg[1]:
            raise AnkleUnreachableError(
                f"pitch={pitch_deg:.2f} 가 시험 범위 {self.pitch_deg} 밖임"
            )


def _wrap180(deg: float) -> float:
    """`[-180, 180)` 으로 접음.

    두 모터에 **같은 규약**을 적용해야 함. 한쪽만 `[0, 360)` 이면 IK 가 340도를
    돌려주고 모터는 -20도를 보고해 360도 차이가 남 (이슈 #1).
    """
    return (float(deg) + 180.0) % 360.0 - 180.0


def _rot_roll(phi: float) -> np.ndarray:
    """y 축 회전."""
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_pitch(theta: float) -> np.ndarray:
    """x 축 회전."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _d_rot_roll(phi: float) -> np.ndarray:
    """`_rot_roll` 의 phi 미분."""
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[-s, 0.0, c], [0.0, 0.0, 0.0], [-c, 0.0, -s]])


def _d_rot_pitch(theta: float) -> np.ndarray:
    """`_rot_pitch` 의 theta 미분."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[0.0, 0.0, 0.0], [0.0, -s, -c], [0.0, c, -s]])


class AnkleKinematics:
    """발목 링키지 하나. 기하값을 받아 IK/FK 를 풂.

    상태를 갖지 않음 — 같은 입력에 항상 같은 출력임. 다리마다 하나씩 만들어 두고
    계속 재사용하면 됨.
    """

    def __init__(
        self,
        geometry: AnkleGeometry = None,
        *,
        envelope: AnkleEnvelope = None,
    ) -> None:
        g = geometry if geometry is not None else AnkleGeometry()
        self.geometry = g
        self.envelope = envelope if envelope is not None else AnkleEnvelope()

        self._A1 = np.array(g.a1, dtype=float)
        self._A2 = np.array(g.a2, dtype=float)
        self._O = np.array(g.origin, dtype=float)

        # 중립 자세(모터각 0)에서의 로드 끝점. 여기서 로드 길이가 나옴.
        b1 = self._crank_point(self._A1, 0.0, g.offset_sign_1, g.rotation_sign_1)
        b2 = self._crank_point(self._A2, 0.0, g.offset_sign_2, g.rotation_sign_2)
        self._L1 = float(np.linalg.norm(b1 - np.array(g.c1, dtype=float)))
        self._L2 = float(np.linalg.norm(b2 - np.array(g.c2, dtype=float)))

        # 발판이 돌면 C 도 같이 도므로, 회전 중심 기준 상대 위치로 들고 있음.
        self._C1_local = np.array(g.c1, dtype=float) - self._O
        self._C2_local = np.array(g.c2, dtype=float) - self._O

    def __repr__(self) -> str:
        return f"AnkleKinematics(L1={self._L1:.2f}, L2={self._L2:.2f})"

    @property
    def rod_lengths(self) -> Tuple[float, float]:
        """로드 길이 (mm). 기하값에서 계산된 것이라 바뀌지 않음."""
        return self._L1, self._L2

    def _crank_point(
        self, axis: Vec3, angle_rad: float, offset_sign: float, rotation_sign: float
    ) -> Vec3:
        """모터각에 대응하는 디스크 위 점. 로드의 모터 쪽 끝임."""
        g = self.geometry
        a = rotation_sign * angle_rad
        return np.array([
            axis[0] + offset_sign * g.crank_t,
            axis[1] + g.crank_r * math.cos(a),
            axis[2] + g.crank_r * math.sin(a),
        ])

    def _foot_points(self, pitch_rad: float, roll_rad: float) -> Tuple[Vec3, Vec3]:
        """발판이 돌았을 때 로드가 붙는 두 지점.

        roll 을 먼저 걸고 pitch 를 나중에 검. 순서가 바뀌면 다른 자세가 나옴.
        """
        after_roll_1 = self._O + _rot_roll(roll_rad) @ self._C1_local
        after_roll_2 = self._O + _rot_roll(roll_rad) @ self._C2_local
        p = _rot_pitch(pitch_rad)
        return (
            self._O + p @ (after_roll_1 - self._O),
            self._O + p @ (after_roll_2 - self._O),
        )

    def _solve_motor_angle(
        self, delta: Vec3, rod_len: float, offset_sign: float, rotation_sign: float
    ) -> Tuple[float, bool]:
        """로드 끝점까지의 거리가 `rod_len` 이 되는 모터각을 찾음.

        닫힌 해임. `A*cos + B*sin = K` 꼴로 정리하면 해가 둘 나오는데, 중립(0도)에
        가까운 쪽을 고름 — 링키지가 뒤집힌 자세로 넘어가는 것을 막으려는 것임.

        반환: (모터각 rad, 해가 존재하는지)
        """
        g = self.geometry
        a_trig = 2.0 * g.crank_r * delta[1]
        b_trig = 2.0 * g.crank_r * delta[2]
        d_sq = (delta[0] - offset_sign * g.crank_t) ** 2 + delta[1] ** 2 + delta[2] ** 2
        k = d_sq + g.crank_r ** 2 - rod_len ** 2

        rho = math.hypot(a_trig, b_trig)
        if rho == 0.0:
            return 0.0, False

        ratio = k / rho
        feasible = abs(ratio) <= 1.0        # 넘으면 로드가 닿지 않음

        psi = math.atan2(a_trig, b_trig)
        asin_val = math.asin(max(-1.0, min(1.0, ratio)))
        candidates = (-psi + asin_val, -psi + math.pi - asin_val)

        # 중립(0)에 가까운 해. 2pi 주기를 맞춘 뒤 비교함.
        def near_zero(a: float) -> float:
            return a - 2.0 * math.pi * round(a / (2.0 * math.pi))

        chosen = min((near_zero(c) for c in candidates), key=abs)
        return rotation_sign * chosen, feasible

    # ---- 역기구학 ---------------------------------------------------------
    def solve_ik(
        self, pitch_deg: float, roll_deg: float, *, enforce_envelope: bool = True
    ) -> Tuple[float, float]:
        """(pitch, roll) -> (a1, a2). 둘 다 도 단위, `[-180, 180)`.

        닫힌 해라 제어 루프에서 매 주기 불러도 됨.

        닿지 않는 자세면 `AnkleUnreachableError` 를 던짐. 자르지 않는 이유: 한쪽
        모터만 잘리면 두 로드가 서로 다른 자세를 요구해 관절이 비틀림.
        """
        if enforce_envelope:
            self.envelope.check(pitch_deg, roll_deg)

        c1, c2 = self._foot_points(math.radians(pitch_deg), math.radians(roll_deg))
        g = self.geometry

        a1, ok1 = self._solve_motor_angle(
            c1 - self._A1, self._L1, g.offset_sign_1, g.rotation_sign_1
        )
        a2, ok2 = self._solve_motor_angle(
            c2 - self._A2, self._L2, g.offset_sign_2, g.rotation_sign_2
        )
        if not (ok1 and ok2):
            raise AnkleUnreachableError(
                f"pitch={pitch_deg:.2f}, roll={roll_deg:.2f} 에 로드 해가 없음 "
                f"(a1 가능={ok1}, a2 가능={ok2})"
            )

        return _wrap180(math.degrees(a1)), _wrap180(math.degrees(a2))

    def is_reachable(self, pitch_deg: float, roll_deg: float) -> bool:
        """시험 범위를 무시하고 링키지가 닿는지만 봄."""
        try:
            self.solve_ik(pitch_deg, roll_deg, enforce_envelope=False)
            return True
        except AnkleUnreachableError:
            return False

    # ---- 야코비안과 토크 ---------------------------------------------------
    def jacobian(self, pitch_deg: float, roll_deg: float) -> np.ndarray:
        """`d(a1, a2) / d(pitch, roll)`. 2x2 행렬.

        **닫힌 해임.** 수치 미분이 아니라 제약식을 직접 미분한 것이라, eps 를 고를
        일도 wrap 불연속에 걸릴 일도 없음.

        **단위가 없음.** 분자와 분모가 둘 다 각도라 도/도 와 rad/rad 가 같은 값임.
        그래서 호출부가 어느 단위를 쓰든 그대로 쓸 수 있음.

        유도: 모터각은 로드 길이 제약에서 나옴.

            |B_i(b_i) - C_i|^2 = L_i^2,     b_i = rotation_sign_i * a_i

        이것을 `D_i = C_i - A_i` 로 음함수 미분하면

            d(b_i)/d(D_i) = (C_i - B_i) / (R * (D_i.z*cos(b_i) - D_i.y*sin(b_i)))

        분자는 **로드 벡터**(크기가 곧 로드 길이)이고, 분모는 그 로드의 특이점에서
        정확히 0이 되는 값임 -- `_solve_motor_angle` 의 `|ratio| -> 1` 과 같은 조건임.
        여기에 `C_i(pitch, roll)` 의 회전 미분을 연쇄법칙으로 이으면 됨.

        특이점이거나 로드 해가 없으면 `AnkleUnreachableError` 를 던짐. 그 자세에서는
        모터를 돌려도 발판이 그 방향으로 안 움직이므로 토크가 발산함.
        """
        g = self.geometry
        theta, phi = math.radians(pitch_deg), math.radians(roll_deg)

        roll_m, d_roll_m = _rot_roll(phi), _d_rot_roll(phi)
        pitch_m, d_pitch_m = _rot_pitch(theta), _d_rot_pitch(theta)

        rods = (
            (self._C1_local, self._A1, self._L1, g.offset_sign_1, g.rotation_sign_1),
            (self._C2_local, self._A2, self._L2, g.offset_sign_2, g.rotation_sign_2),
        )

        jac = np.zeros((2, 2))
        for i, (c_local, axis, rod_len, offset_sign, rotation_sign) in enumerate(rods):
            rolled = roll_m @ c_local
            c_point = self._O + pitch_m @ rolled
            dc_dpitch = d_pitch_m @ rolled
            dc_droll = pitch_m @ (d_roll_m @ c_local)

            delta = c_point - axis
            angle, ok = self._solve_motor_angle(delta, rod_len, offset_sign, rotation_sign)
            # `_solve_motor_angle` 이 고른 것과 같은 가지를 써야 함.
            beta = rotation_sign * angle
            cos_b, sin_b = math.cos(beta), math.sin(beta)
            denom = g.crank_r * (delta[2] * cos_b - delta[1] * sin_b)

            if not ok or abs(denom) < SINGULAR_EPS:
                raise AnkleUnreachableError(
                    f"로드 {i + 1} 이 특이점에 있음 "
                    f"(pitch={pitch_deg:.2f}, roll={roll_deg:.2f}, 분모={denom:.3e}). "
                    f"이 자세에서는 모터를 돌려도 발판이 그 방향으로 움직이지 않음"
                )

            # C_i - B_i. 유효한 해에서는 크기가 로드 길이임.
            rod_vec = np.array([
                delta[0] - offset_sign * g.crank_t,
                delta[1] - g.crank_r * cos_b,
                delta[2] - g.crank_r * sin_b,
            ])
            grad = rod_vec / denom

            jac[i, 0] = rotation_sign * float(grad @ dc_dpitch)
            jac[i, 1] = rotation_sign * float(grad @ dc_droll)

        return jac

    def joint_torque_to_motor(
        self,
        pitch_deg: float,
        roll_deg: float,
        tau_pitch: float,
        tau_roll: float,
        *,
        max_condition: float = MAX_JACOBIAN_CONDITION,
    ) -> Tuple[float, float]:
        """관절 토크 (pitch, roll) -> 모터 토크 (a1, a2). 단위는 Nm.

        전달이 손실 없다고 보면 가상일이 보존됨.

            tau_a . da = tau_pr . dpr,     da = J dpr
            => tau_a = (J^T)^-1 tau_pr

        **지금 자세에서 선형화함.** 목표 자세가 아니라 실측 자세를 넣을 것 -- 링키지가
        지금 실제로 놓인 기하가 토크를 정함.

        `pitch_deg`/`roll_deg` 는 도, 토크는 Nm 임. 야코비안이 단위 없는 값이라
        각도 단위가 토크에 섞이지 않음.

        조건수가 `max_condition` 을 넘으면 거부함. 그런 자세에서는 **자세를 조금
        잘못 재기만 해도 토크가 크게 튐** -- 링키지의 실제 한계이지 계산 문제가
        아님.
        """
        jac = self.jacobian(pitch_deg, roll_deg)

        cond = float(np.linalg.cond(jac.T))
        if not math.isfinite(cond) or cond > max_condition:
            raise AnkleUnreachableError(
                f"pitch={pitch_deg:.2f}, roll={roll_deg:.2f} 에서 야코비안 조건수가 "
                f"{cond:.1f} 임 (한계 {max_condition}). 특이점에 가까워 자세 측정 "
                f"오차가 토크로 증폭됨"
            )

        try:
            tau = np.linalg.solve(jac.T, np.array([tau_pitch, tau_roll], dtype=float))
        except np.linalg.LinAlgError as e:
            raise AnkleUnreachableError(
                f"pitch={pitch_deg:.2f}, roll={roll_deg:.2f} 에서 야코비안을 풀 수 없음: {e}"
            ) from e

        return float(tau[0]), float(tau[1])

    def mit_torque(
        self,
        target: Tuple[float, float],
        current: Tuple[float, float],
        *,
        target_velocity: Tuple[float, float] = (0.0, 0.0),
        current_velocity: Tuple[float, float] = (0.0, 0.0),
        kp: Tuple[float, float] = (0.0, 0.0),
        kd: Tuple[float, float] = (0.0, 0.0),
        feedforward: Tuple[float, float] = (0.0, 0.0),
    ) -> Tuple[float, float]:
        """관절 목표 -> 모터 토크 (a1, a2). 단위는 Nm.

        모든 짝은 `(pitch, roll)` 순서임.

            target, current              도
            target_velocity, ...         도/초
            kp                           Nm/rad
            kd                           Nm/(rad/s)
            feedforward                  Nm

        각도는 도, 게인은 라디안 기준임. **둘 다 이 저장소의 기존 규약임** --
        `solve_ik`/`solve_fk` 가 도를 쓰고, `robot.yaml` 의 `kp` 는 변환 없이 모터로
        나가는데 모터는 프레임의 라디안 각도로 계산함. 오차를 안에서 라디안으로
        바꾸므로 `kp` 값을 그대로 쓸 수 있음.

        모터 펌웨어가 하던 PD 를 여기서 함.

            tau_pr = kp*(목표 - 실측) + kd*(목표속도 - 실측속도) + feedforward
            tau_a  = (J^T)^-1 tau_pr

        **지금 자세에서 선형화함.** 목표가 아니라 실측을 씀 -- 링키지가 지금 놓인
        기하가 토크를 정함.

        나온 값은 `MitCommand(kp=0, kd=0, torque_nm=...)` 로 보낼 것. 모터 쪽 PD 를
        끄지 않으면 두 PD 가 겹침.
        """
        tau_pitch = (
            kp[0] * math.radians(target[0] - current[0])
            + kd[0] * math.radians(target_velocity[0] - current_velocity[0])
            + feedforward[0]
        )
        tau_roll = (
            kp[1] * math.radians(target[1] - current[1])
            + kd[1] * math.radians(target_velocity[1] - current_velocity[1])
            + feedforward[1]
        )
        return self.joint_torque_to_motor(current[0], current[1], tau_pitch, tau_roll)

    # ---- 순기구학 ---------------------------------------------------------
    def solve_fk(
        self,
        a1_deg: float,
        a2_deg: float,
        *,
        guess_pitch_deg: float = 0.0,
        guess_roll_deg: float = 0.0,
        max_iter: int = 120,
        tol: float = 1e-7,
    ) -> Tuple[float, float]:
        """(a1, a2) -> (pitch, roll). 도 단위.

        **뉴턴 반복임.** 두 로드 길이가 동시에 맞는 (pitch, roll) 을 수치로 찾아감.
        `solve_ik` 보다 훨씬 비싸므로 제어 루프에서 매 주기 부르지 말 것 — 사람에게
        보여줄 때만 씀.

        **초기 추정이 중요함.** 같은 모터각 조합에 대응하는 자세가 여럿이라, 추정이
        멀면 다른 자세로 수렴함. 직전 결과를 `guess_*` 로 넣을 것 — 제어 루프에서는
        한 주기 전 자세가 늘 가까움.

        수렴한 답이 정말 그 모터각에 대응하는지 **IK 로 되짚어 확인함.** IK 는 닫힌
        해라 싸고, 이 검사가 없으면 다른 자세로 수렴한 값을 조용히 돌려주게 됨.
        틀린 답이 나오는 것이 답이 없는 것보다 나쁨.

        수렴하지 않거나 되짚기가 어긋나면 `AnkleUnreachableError` 를 던짐. 발산한
        값을 그대로 돌려주면 NaN 이 상위로 흘러가는데, 그건 `safety.guards` 가 잡기
        전까지 조용함.
        """
        g = self.geometry
        b1 = self._crank_point(
            self._A1, math.radians(a1_deg), g.offset_sign_1, g.rotation_sign_1
        )
        b2 = self._crank_point(
            self._A2, math.radians(a2_deg), g.offset_sign_2, g.rotation_sign_2
        )

        phi = math.radians(guess_roll_deg)
        theta = math.radians(guess_pitch_deg)
        eps = 1e-6

        def residual(p: float, t: float) -> np.ndarray:
            """로드 길이 오차 두 개. 둘 다 0이면 그 자세가 답임."""
            c1, c2 = self._foot_points(t, p)
            return np.array([
                float(np.sum((c1 - b1) ** 2)) - self._L1 ** 2,
                float(np.sum((c2 - b2) ** 2)) - self._L2 ** 2,
            ])

        converged = False
        for _ in range(max_iter):
            f = residual(phi, theta)
            if float(np.linalg.norm(f)) < tol:
                converged = True
                break

            # 해석적 미분 대신 수치 미분. 기하가 바뀌어도 따라오게 하려는 것임.
            d_phi = (residual(phi + eps, theta) - f) / eps
            d_theta = (residual(phi, theta + eps) - f) / eps
            jac = np.column_stack([d_phi, d_theta])
            try:
                step = np.linalg.solve(jac + np.eye(2) * 1e-11, -f)
            except np.linalg.LinAlgError:
                break
            if not np.all(np.isfinite(step)):
                break
            phi += float(step[0])
            theta += float(step[1])

        if not converged:
            raise AnkleUnreachableError(
                f"a1={a1_deg:.2f}, a2={a2_deg:.2f} 에서 FK 가 수렴하지 않음. "
                f"모터 각도가 실제로 도달 불가능한 조합이거나 초기 추정이 멀었을 수 있음"
            )

        pitch_deg, roll_deg = math.degrees(theta), math.degrees(phi)

        # 되짚기. 다른 자세로 수렴했으면 여기서 걸림.
        try:
            back = self.solve_ik(pitch_deg, roll_deg, enforce_envelope=False)
        except AnkleUnreachableError:
            back = None
        if back is None or not all(
            abs(_wrap180(got - want)) < FK_VERIFY_TOL_DEG
            for got, want in zip(back, (_wrap180(a1_deg), _wrap180(a2_deg)))
        ):
            raise AnkleUnreachableError(
                f"a1={a1_deg:.2f}, a2={a2_deg:.2f} 에서 FK 가 다른 자세로 수렴함 "
                f"(찾은 자세 pitch={pitch_deg:.2f}, roll={roll_deg:.2f} 를 IK 로 되짚으면 "
                f"{'해 없음' if back is None else f'({back[0]:.2f}, {back[1]:.2f})'}). "
                f"guess_pitch_deg / guess_roll_deg 를 실제 자세에 가깝게 줄 것"
            )

        return pitch_deg, roll_deg
