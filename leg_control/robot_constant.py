"""
robot_constant.py

Shared hardware constants for the bipedal robot's leg actuators.

*** IMPORTANT — READ BEFORE ENABLING TORQUE ON REAL HARDWARE ***
The values below (MOTOR_SIGN, MOTOR_OFFSET_DEG, JOINT_LIMITS_DEG,
DEFAULT_GAINS, ANKLE_COUPLING_CALIBRATION_*, MOTORS specs) are PLACEHOLDERS.
They are wired up so BipedalRobotController / SingleLegController import and
run, but they are NOT calibrated to any physical robot. Every line marked
"# TODO: calibrate" must be replaced with a value measured on your actual
hardware before you run `mode="control"` (i.e. before commanding torque).
Wrong sign/offset/limit values can drive a joint straight into its
mechanical hard-stop.

CALIBRATED below is a guard the controllers check at construction time —
leave it False until you've actually done that calibration.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple

# Flip to True only after every "# TODO: calibrate" value below has been
# measured/verified on the physical robot.
CALIBRATED = False

# -------------------------------------------------------------------------
# Motor identity / topology
# -------------------------------------------------------------------------
# Left leg:  1=hipz 2=hipx 3=hipy 4=knee 5=ankle_a1 6=ankle_a2
# Right leg: 7=hipz 8=hipx 9=hipy 10=knee 11=ankle_a1 12=ankle_a2
LEFT_LEG_MOTOR_IDS: Tuple[int, ...] = (1, 2, 3, 4, 5, 6)
RIGHT_LEG_MOTOR_IDS: Tuple[int, ...] = (7, 8, 9, 10, 11, 12)
MOTOR_IDS: Tuple[int, ...] = LEFT_LEG_MOTOR_IDS + RIGHT_LEG_MOTOR_IDS

# One leg <-> one CAN bus (matches BipedalRobotController's dual-bus layout).
CAN0_MOTOR_IDS = LEFT_LEG_MOTOR_IDS
CAN1_MOTOR_IDS = RIGHT_LEG_MOTOR_IDS

# Joint-space order used everywhere (matches BipedalRobotController's model order).
LEG_JOINT_NAMES: Tuple[str, ...] = ("hipy", "hipx", "hipz", "knee", "ankle_pitch", "ankle_roll")

# Per-side motor ids, in LEG_JOINT_NAMES order except the ankle pair, which is
# the two coupled actuators (a1, a2) rather than pitch/roll directly.
LEG_MOTOR_IDS_BY_SIDE: Dict[str, Tuple[int, ...]] = {
    "left": LEFT_LEG_MOTOR_IDS,
    "right": RIGHT_LEG_MOTOR_IDS,
}

# -------------------------------------------------------------------------
# Actuator specs (MIT-mode encoding range), keyed by motor id.
# -------------------------------------------------------------------------
@dataclass(frozen=True)
class MotorSpec:
    name: str
    pmax_rad: float   # position encoding range used by pack_mit_command / decode_state_frame
    vmax_rad_s: float
    tmax_nm: float


def _make_leg_specs(prefix: str, ids: Tuple[int, ...]) -> Dict[int, MotorSpec]:
    joint_order = ["hipy", "hipx", "hipz", "knee", "ankle_a1", "ankle_a2"]
    # TODO: calibrate -- replace with your actuators' real datasheet ranges.
    # (pmax_rad, vmax_rad_s, tmax_nm)
    specs_by_joint = {
        "hipz": (12.57, 44.0, 17.0),
        "hipx": (12.57, 44.0, 17.0),
        "hipy": (12.57, 44.0, 17.0),
        "knee": (12.57, 44.0, 17.0),
        "ankle_a1": (12.57, 33.0, 14.0),
        "ankle_a2": (12.57, 33.0, 14.0),
    }
    return {mid: MotorSpec(f"{prefix}_{jn}", *specs_by_joint[jn]) for mid, jn in zip(ids, joint_order)}


MOTORS: Dict[int, MotorSpec] = {
    **_make_leg_specs("left", LEFT_LEG_MOTOR_IDS),
    **_make_leg_specs("right", RIGHT_LEG_MOTOR_IDS),
}

# -------------------------------------------------------------------------
# Calibration: encoder sign + zero offset.
#   calibrated_deg = sign * raw_deg + offset_deg
# -------------------------------------------------------------------------
# TODO: calibrate -- verify sign against the real encoder rotation direction.
MOTOR_SIGN: Dict[int, float] = {mid: 1.0 for mid in MOTOR_IDS}
# TODO: calibrate -- zero each motor at a known joint pose so calibrated_deg
# reads 0 (or your chosen reference) at that pose.
MOTOR_OFFSET_DEG: Dict[int, float] = {mid: 0.0 for mid in MOTOR_IDS}

# -------------------------------------------------------------------------
# Joint limits, in the same space MOTOR_SIGN/OFFSET map into ("raw" degrees
# by default -- see the controller's `_limit_source`).
# -------------------------------------------------------------------------
# TODO: calibrate -- set to the true mechanical range of motion (deg), with
# margin before the physical hard-stop.
JOINT_LIMITS_DEG: Dict[int, Tuple[float, float]] = {
    1: (-45.0, 45.0),    # left hipz
    2: (-30.0, 30.0),    # left hipx
    3: (-100.0, 30.0),   # left hipy
    4: (0.0, 120.0),     # left knee
    5: (-45.0, 45.0),    # left ankle a1
    6: (-45.0, 45.0),    # left ankle a2
    7: (-117.07, 21.07),    # right hipz
    8: (-5.51, 79.64),    # right hipx
    9: (-41.90, 31.09),   # right hipy
    10: (-20.65, 74.79),    # right knee
    11: (-79.77, 43.16),   # right ankle a1
    12: (-12.50, 126.66),   # right ankle a2
}

# -------------------------------------------------------------------------
# Coupled-ankle kinematics (2 actuators -> pitch/roll):
#   pitch = sign_p * (a1 - a2) / 2 + offset_p
#   roll  = sign_r * (a1 + a2) / 2 + offset_r
# -------------------------------------------------------------------------
# TODO: calibrate -- verify sign/offset against the physical ankle linkage.
ANKLE_COUPLING_CALIBRATION_LEFT: Dict[str, Dict[str, float]] = {
    "pitch": {"sign": 1.0, "offset_deg": 0.0},
    "roll": {"sign": 1.0, "offset_deg": 0.0},
}
ANKLE_COUPLING_CALIBRATION_RIGHT: Dict[str, Dict[str, float]] = {
    "pitch": {"sign": 1.0, "offset_deg": 0.0},
    "roll": {"sign": 1.0, "offset_deg": 0.0},
}

# -------------------------------------------------------------------------
# Default MIT-mode gains per motor: (kp, kd).
# -------------------------------------------------------------------------
# TODO: calibrate -- start low and raise gradually with the leg supported
# off the ground (e.g. on a stand) before trusting these on a standing robot.
DEFAULT_GAINS: Dict[int, Tuple[float, float]] = {mid: (1.0, 0.3) for mid in MOTOR_IDS}

# -------------------------------------------------------------------------
# Safety margins / behavior.
# -------------------------------------------------------------------------
COMMAND_MARGIN_DEG = 3.0     # keep commands this far inside JOINT_LIMITS_DEG
STATE_MARGIN_DEG = 5.0       # trip E-STOP if measured state exceeds limits by more than this
NEAR_STOP_MARGIN_DEG = 8.0   # switch to damping-only once this close to a limit
DAMPING_KD = 1.0             # kd used for damping-only (kp=0) commands

# -------------------------------------------------------------------------
# CAN protocol command bytes (MIT-mode actuator convention).
# TODO: verify against your actuator's datasheet -- these bytes vary by vendor.
# -------------------------------------------------------------------------
CAN_CMD_ENABLE = 0xFC
CAN_CMD_DISABLE = 0xFD
CAN_CMD_ZERO = 0xFE
CAN_CMD_CLEAR_FAULT = 0xFB

# -------------------------------------------------------------------------
# URDF / visualization (optional — only needed if you attach a meshcat
# visualizer via BipedalRobotController; SingleLegController doesn't use these).
# -------------------------------------------------------------------------
MODEL_URDF_DIR = Path(__file__).resolve().parent / "urdf"
MODEL_URDF_PATH = MODEL_URDF_DIR / "robit.urdf"
