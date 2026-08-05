"""
ankle_kinematics.py

Per-side wrapper around the corrected 2-motor ankle IK/FK (axis fixes +
roll/pitch swap confirmed against hardware earlier in this project):
  - motor rotation axis = X (disc lies in the Y-Z plane)
  - roll(phi)  -> rotation about Y
  - pitch(theta) -> rotation about X

The measured geometry constants below (A1, A2, O, C1_init, C2_init) were
taken from one physical ankle (the one kinematics.py/kinematics_fixed.py was
built from). Which leg that is (left or right) has NOT been confirmed here.

*** TODO: confirm before use ***
  1. Which leg (left/right) do these measured points actually belong to?
  2. The other leg's version below is a MIRROR (x -> -x) of these points,
     assuming perfect bilateral symmetry. This is an assumption, not a
     measurement -- verify against the real robot before trusting it.
  3. Confirm the real CAN motor IDs for both ankle actuators on both legs.
     (kinematics.py's own comments call them "motor7"/"motor8", which does
     NOT match the placeholder mapping in robot_constant.py where 7/8 are
     the RIGHT leg's hipz/hipx. This mismatch must be resolved before
     wiring real CAN ids in single_leg_controller.py.)
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass
from typing import Tuple

# Software test envelope requested for the ankle (on top of whatever the
# linkage can kinematically reach -- see FEASIBLE checks below).
ROLL_LIMIT_DEG: Tuple[float, float] = (-25.0, 25.0)
PITCH_LIMIT_DEG: Tuple[float, float] = (-40.0, 40.0)


class AnkleUnreachableError(ValueError):
    """Raised when a requested (pitch, roll) has no valid motor solution."""


class AnkleKinematics:
    def __init__(self, side: str = "right"):
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        self.side = side
        mirror = -1.0 if side == "left" else 1.0  # TODO: verify bilateral symmetry assumption

        # Measured geometry (see module docstring: confirmed for ONE leg only).
        self.A1 = np.array([-26.10, 74.00, -255.00]) * np.array([mirror, 1, 1])
        self.A2 = np.array([-80.90, 74.00, -313.00]) * np.array([mirror, 1, 1])
        self.O = np.array([-53.50, 74.00, -393.00]) * np.array([mirror, 1, 1])
        C1_init = np.array([-8.10, 111.343, -398.00]) * np.array([mirror, 1, 1])
        C2_init = np.array([-98.90, 111.343, -398.00]) * np.array([mirror, 1, 1])

        self.R = 40.0
        self.t = 18.0
        self.OFFSET_SIGN_1 = 1.0
        self.OFFSET_SIGN_2 = -1.0
        # TODO: verify rotation handedness on real hardware for this side.
        self.ROTATION_SIGN_1 = 1.0
        self.ROTATION_SIGN_2 = -1.0

        self.ALPHA1_START = 0.0
        self.ALPHA2_START = 0.0

        a1m = self.ROTATION_SIGN_1 * self.ALPHA1_START
        a2m = self.ROTATION_SIGN_2 * self.ALPHA2_START
        self.B1_init = np.array([
            self.A1[0] + self.OFFSET_SIGN_1 * self.t,
            self.A1[1] + self.R * np.cos(a1m),
            self.A1[2] + self.R * np.sin(a1m),
        ])
        self.B2_init = np.array([
            self.A2[0] + self.OFFSET_SIGN_2 * self.t,
            self.A2[1] + self.R * np.cos(a2m),
            self.A2[2] + self.R * np.sin(a2m),
        ])

        self.L1 = float(np.linalg.norm(self.B1_init - C1_init))
        self.L2 = float(np.linalg.norm(self.B2_init - C2_init))
        self.C1_local = C1_init - self.O
        self.C2_local = C2_init - self.O

    def _solve_motor_angle(self, D, R, L, target_angle, offset_sign, rotation_sign):
        A_trig = 2 * R * D[1]
        B_trig = 2 * R * D[2]
        D_sq = (D[0] - offset_sign * self.t) ** 2 + D[1] ** 2 + D[2] ** 2
        K = D_sq + R ** 2 - L ** 2
        rho = np.sqrt(A_trig ** 2 + B_trig ** 2)
        ratio = K / rho
        feasible = abs(ratio) <= 1.0
        psi = np.arctan2(A_trig, B_trig)
        asin_val = np.arcsin(np.clip(ratio, -1.0, 1.0))
        cand1 = -psi + asin_val
        cand2 = -psi + np.pi - asin_val
        target_math = rotation_sign * target_angle

        def wrap_near(a, tgt):
            return a + 2 * np.pi * np.round((tgt - a) / (2 * np.pi))

        cand1 = wrap_near(cand1, target_math)
        cand2 = wrap_near(cand2, target_math)
        chosen = cand1 if abs(cand1 - target_math) <= abs(cand2 - target_math) else cand2
        return rotation_sign * chosen, feasible, ratio

    def solve_ik(self, pitch_deg: float, roll_deg: float, *, enforce_test_envelope: bool = True):
        """
        Returns (alpha1_deg, alpha2_deg). Raises AnkleUnreachableError if the
        target has no valid motor solution (instead of silently clipping),
        or if enforce_test_envelope=True and the target falls outside
        ROLL_LIMIT_DEG / PITCH_LIMIT_DEG.
        """
        if enforce_test_envelope:
            if not (ROLL_LIMIT_DEG[0] <= roll_deg <= ROLL_LIMIT_DEG[1]):
                raise AnkleUnreachableError(
                    f"roll={roll_deg:.2f} outside test envelope {ROLL_LIMIT_DEG}"
                )
            if not (PITCH_LIMIT_DEG[0] <= pitch_deg <= PITCH_LIMIT_DEG[1]):
                raise AnkleUnreachableError(
                    f"pitch={pitch_deg:.2f} outside test envelope {PITCH_LIMIT_DEG}"
                )

        phi, theta = np.radians(roll_deg), np.radians(pitch_deg)
        c_p, s_p = np.cos(phi), np.sin(phi)
        c_t, s_t = np.cos(theta), np.sin(theta)

        R_roll = np.array([[c_p, 0.0, s_p], [0.0, 1.0, 0.0], [-s_p, 0.0, c_p]])
        C1p = self.O + R_roll @ self.C1_local
        C2p = self.O + R_roll @ self.C2_local
        R_pitch = np.array([[1.0, 0.0, 0.0], [0.0, c_t, -s_t], [0.0, s_t, c_t]])
        C1f = self.O + R_pitch @ (C1p - self.O)
        C2f = self.O + R_pitch @ (C2p - self.O)

        a1, feasible1, _ = self._solve_motor_angle(
            C1f - self.A1, self.R, self.L1, self.ALPHA1_START,
            self.OFFSET_SIGN_1, self.ROTATION_SIGN_1,
        )
        a2, feasible2, _ = self._solve_motor_angle(
            C2f - self.A2, self.R, self.L2, self.ALPHA2_START,
            self.OFFSET_SIGN_2, self.ROTATION_SIGN_2,
        )
        if not (feasible1 and feasible2):
            raise AnkleUnreachableError(
                f"pitch={pitch_deg:.2f}, roll={roll_deg:.2f} has no valid rod solution "
                f"(motor1 feasible={feasible1}, motor2 feasible={feasible2})"
            )

        a1 = (a1 + np.pi) % (2 * np.pi) - np.pi
        if a1 < 0:
            a1 += 2 * np.pi
        a2 = (a2 + np.pi) % (2 * np.pi) - np.pi
        return np.degrees(a1), np.degrees(a2)

    def solve_fk(self, alpha1_deg: float, alpha2_deg: float, *, guess_pitch_deg=0.0,
                 guess_roll_deg=0.0, max_iter=120, tol=1e-7):
        a1, a2 = np.radians(alpha1_deg), np.radians(alpha2_deg)
        a1m = self.ROTATION_SIGN_1 * a1
        a2m = self.ROTATION_SIGN_2 * a2
        B1 = np.array([self.A1[0] + self.OFFSET_SIGN_1 * self.t,
                       self.A1[1] + self.R * np.cos(a1m), self.A1[2] + self.R * np.sin(a1m)])
        B2 = np.array([self.A2[0] + self.OFFSET_SIGN_2 * self.t,
                       self.A2[1] + self.R * np.cos(a2m), self.A2[2] + self.R * np.sin(a2m)])

        phi, theta = np.radians(guess_roll_deg), np.radians(guess_pitch_deg)
        eps = 1e-6
        for _ in range(max_iter):
            def geom(p, t):
                c_p, s_p = np.cos(p), np.sin(p)
                c_t, s_t = np.cos(t), np.sin(t)
                R_roll = np.array([[c_p, 0, s_p], [0, 1, 0], [-s_p, 0, c_p]])
                R_pitch = np.array([[1, 0, 0], [0, c_t, -s_t], [0, s_t, c_t]])
                C1p = self.O + R_roll @ self.C1_local
                C2p = self.O + R_roll @ self.C2_local
                return self.O + R_pitch @ (C1p - self.O), self.O + R_pitch @ (C2p - self.O)

            C1, C2 = geom(phi, theta)
            f1 = np.sum((C1 - B1) ** 2) - self.L1 ** 2
            f2 = np.sum((C2 - B2) ** 2) - self.L2 ** 2
            F = np.array([f1, f2])
            if np.linalg.norm(F) < tol:
                break
            C1p_, C2p_ = geom(phi + eps, theta)
            df1_dphi = (np.sum((C1p_ - B1) ** 2) - self.L1 ** 2 - f1) / eps
            df2_dphi = (np.sum((C2p_ - B2) ** 2) - self.L2 ** 2 - f2) / eps
            C1t_, C2t_ = geom(phi, theta + eps)
            df1_dtheta = (np.sum((C1t_ - B1) ** 2) - self.L1 ** 2 - f1) / eps
            df2_dtheta = (np.sum((C2t_ - B2) ** 2) - self.L2 ** 2 - f2) / eps
            J = np.array([[df1_dphi, df1_dtheta], [df2_dphi, df2_dtheta]])
            delta = np.linalg.solve(J + np.eye(2) * 1e-11, -F)
            phi += delta[0]
            theta += delta[1]

        return np.degrees(theta), np.degrees(phi)  # (pitch, roll)

    def is_feasible(self, pitch_deg: float, roll_deg: float) -> bool:
        try:
            self.solve_ik(pitch_deg, roll_deg, enforce_test_envelope=False)
            return True
        except AnkleUnreachableError:
            return False
