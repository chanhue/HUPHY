"""
single_leg_controller.py

Controller for a SINGLE leg (6 motors on ONE CAN bus). It mirrors the safety
machinery in `BipedalRobotController` (E-STOP on out-of-limit measured state,
command-jump guard, command-limit clamping, near-stop damping) but is scoped
to one leg so it can be driven standalone -- e.g. by an external policy that
only knows about a single leg.

Intended usage (policy lives elsewhere, this class only does actuation/IO):

    from single_leg_controller import SingleLegController

    leg = SingleLegController(side="left", channel="can0", allow_uncalibrated=True)
    leg.start(mode="control", auto_enable=True)

    while running:
        joint_deg = leg.get_joint_state()      # {"hipz": .., "knee": .., ...}
        target = my_policy.infer(joint_deg)    # dict with whichever joints it wants to move
        leg.set_leg_action(**target)           # sent at leg.control_hz by the background loop
        time.sleep(1.0 / policy_hz)

    leg.stop()

`set_leg_action` only updates the *target*; a background thread sends MIT
commands at `control_hz` using the most recent target (zero-order hold), so
the policy can run at a different (e.g. slower or non-deterministic) rate
than the CAN control loop.

NOTE: robot_constant.py ships with placeholder calibration values. Do not
pass allow_uncalibrated=True against real hardware until MOTOR_SIGN,
MOTOR_OFFSET_DEG, JOINT_LIMITS_DEG, DEFAULT_GAINS, and
ANKLE_COUPLING_CALIBRATION_* have actually been calibrated -- see the
comments in robot_constant.py.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Union
import csv
import logging
import threading
import time

import can
import numpy as np

# This standalone leg-control package ships mit_codec inside its own utils/
# subpackage (leg_control/utils/mit_codec.py), so add THIS file's directory to
# sys.path and import it as `utils.mit_codec`.
import sys
from pathlib import Path as _Path
_PKG_DIR = _Path(__file__).resolve().parent
if str(_PKG_DIR) not in sys.path:
    sys.path.insert(0, str(_PKG_DIR))

from utils.mit_codec import MotorState, decode_state_frame, pack_mit_command
from robot_constant import (
    CALIBRATED,
    LEG_MOTOR_IDS_BY_SIDE,
    LEG_JOINT_NAMES,
    MOTORS,
    MOTOR_SIGN,
    MOTOR_OFFSET_DEG,
    JOINT_LIMITS_DEG,
    DEFAULT_GAINS,
    ANKLE_COUPLING_CALIBRATION_LEFT,
    ANKLE_COUPLING_CALIBRATION_RIGHT,
    COMMAND_MARGIN_DEG,
    STATE_MARGIN_DEG,
    NEAR_STOP_MARGIN_DEG,
    DAMPING_KD,
    CAN_CMD_CLEAR_FAULT,
    CAN_CMD_ZERO,
    CAN_CMD_ENABLE,
    CAN_CMD_DISABLE,
)
from ankle_kinematics import AnkleKinematics, AnkleUnreachableError, ROLL_LIMIT_DEG, PITCH_LIMIT_DEG

logger = logging.getLogger(__name__)

# Temporary safety switch, mirrors BipedalRobotController: keep the MIT
# feedforward torque field pinned at 0 Nm until it's explicitly needed.
ENABLE_MIT_FEEDFORWARD_TORQUE = False


@dataclass
class JointGains:
    kp: float
    kd: float


@dataclass
class MotorCommand:
    position_deg: float = 0.0
    velocity_deg_s: float = 0.0
    torque_nm: float = 0.0
    kp: Optional[float] = None
    kd: Optional[float] = None


class SingleLegController:
    """Controls the 6 motors of ONE leg over a single CAN bus."""

    MODES = ("state_only", "control")
    RX_FLUSH_MAX_MSGS = 4096

    def __init__(
        self,
        *,
        side: str,
        interface: str = "socketcan",
        channel: Optional[str] = None,
        bus: Optional[Any] = None,
        control_hz: float = 100.0,
        recv_timeout_s: float = 0.001,
        log_path: Optional[Union[str, Path]] = None,
        allow_uncalibrated: bool = False,
    ):
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        if not CALIBRATED and not allow_uncalibrated:
            raise RuntimeError(
                "robot_constant.CALIBRATED is False -- motor signs, offsets, joint "
                "limits, gains, and ankle-coupling values are placeholders. Calibrate "
                "them for the real hardware first, or pass allow_uncalibrated=True to "
                "proceed anyway at your own risk (e.g. against a simulator)."
            )

        # Confirmed: left leg <-> can0, right leg <-> can1 (matches
        # robot_constant.py's CAN0_MOTOR_IDS/CAN1_MOTOR_IDS). Still overridable
        # via the channel= argument if your setup differs.
        if channel is None:
            channel = "can0" if side == "left" else "can1"

        self.side = side
        self.motor_ids = LEG_MOTOR_IDS_BY_SIDE[side]     # e.g. (1,2,3,4,5,6)
        self.hip_knee_ids = self.motor_ids[:4]           # hipz, hipx, hipy, knee
        self.ankle_ids = self.motor_ids[4:]              # (a1, a2)
        # NOTE: ankle geometry/calibration now lives entirely in AnkleKinematics
        # (validated nonlinear 2-motor IK/FK), replacing the old linear
        # ANKLE_COUPLING_CALIBRATION_* placeholder. For a1_id/a2_id specifically,
        # "raw degrees" ARE AnkleKinematics' alpha1_deg/alpha2_deg directly --
        # MOTOR_SIGN/MOTOR_OFFSET_DEG are NOT applied a second time for these two
        # motors (ROTATION_SIGN_*/OFFSET_SIGN_* inside AnkleKinematics already
        # play that role). See ankle_kinematics.py's module docstring for the
        # still-unconfirmed assumptions (which leg the measured geometry is
        # for, and the mirrored-symmetry assumption for the other leg).
        self.ankle_kin = AnkleKinematics(side)
        self._last_ankle_pitch_deg = 0.0
        self._last_ankle_roll_deg = 0.0

        self.bus = bus if bus is not None else can.interface.Bus(interface=interface, channel=channel)
        self.control_hz = float(control_hz)
        self.recv_timeout_s = float(recv_timeout_s)
        self.mode = "state_only"

        self.state: Dict[int, MotorState] = {mid: MotorState() for mid in self.motor_ids}
        self.gains: Dict[int, JointGains] = {
            mid: JointGains(*DEFAULT_GAINS.get(mid, (5.0, 0.5))) for mid in self.motor_ids
        }
        self.motor_sign: Dict[int, float] = {mid: float(MOTOR_SIGN[mid]) for mid in self.motor_ids}
        self.motor_offset_deg: Dict[int, float] = {mid: float(MOTOR_OFFSET_DEG[mid]) for mid in self.motor_ids}
        self.joint_limits_deg: Dict[int, tuple[float, float]] = {
            mid: tuple(JOINT_LIMITS_DEG[mid]) for mid in self.motor_ids if mid in JOINT_LIMITS_DEG
        }
        self.command_limits_cal_deg: Dict[int, tuple[float, float]] = {}
        self.action: Dict[int, MotorCommand] = {mid: MotorCommand() for mid in self.motor_ids}

        self._state_lock = threading.Lock()
        self._action_lock = threading.Lock()
        self._tx_lock = threading.Lock()
        self._rx_lock = threading.Lock()

        self._loop_thread: Optional[threading.Thread] = None
        self._running = False
        self._estop = False
        self._estop_reason = ""
        self._damping_active = False
        self._action_initialized = False
        self._state_only_enforced = False
        self._cycle_idx = 0

        self._limit_source = "raw"
        self.max_cmd_delta_deg = 50.0
        self.enforce_command_limits = True
        self._last_jump_warn_s: Dict[int, float] = {mid: 0.0 for mid in self.motor_ids}
        self.reset_estop_each_state_cycle = True
        self._recompute_command_limits()

        # ROM scale: fraction (0, 1] of each joint's configured range that
        # run_joint_max_rom_sweep / run_full_rom_diagnostic actually sweeps to,
        # measured from the midpoint of (lo, hi). Conservative by default (30%
        # of range) for first-time bring-up; raise toward 1.0 across
        # successive test rounds via set_rom_scale().
        self.rom_scale = 0.3

        self.log_path = Path(log_path) if log_path is not None else None
        self._log_fp = None
        self._log_writer = None

    # -------------------------
    # Public API
    # -------------------------
    def set_mode(self, mode: str) -> None:
        if mode not in self.MODES:
            raise ValueError(f"Unsupported mode '{mode}', expected one of {self.MODES}")
        if mode == "control" and self.mode != "control":
            # Re-latch the next measured state as initial hold target.
            self._action_initialized = False
            self._state_only_enforced = False
        if mode == "state_only" and self.mode != "state_only":
            self._state_only_enforced = False
        self.mode = mode

    def set_joint_gains(self, motor_id: int, *, kp: Optional[float] = None, kd: Optional[float] = None) -> None:
        g = self.gains[motor_id]
        self.gains[motor_id] = JointGains(kp if kp is not None else g.kp, kd if kd is not None else g.kd)

    def set_max_command_delta(self, delta_deg: float) -> None:
        self.max_cmd_delta_deg = float(delta_deg)

    def set_rom_scale(self, scale: float) -> None:
        """
        Set how much of each joint's configured range run_joint_max_rom_sweep /
        run_full_rom_diagnostic actually sweep to (0 < scale <= 1.0), measured
        from the midpoint of that joint's range. 0.3 (default) sweeps only 30%
        of the way to each limit; 1.0 sweeps to the full configured range.
        """
        if not (0.0 < scale <= 1.0):
            raise ValueError("scale must be in (0, 1.0]")
        self.rom_scale = float(scale)

    def clear_estop(self) -> None:
        self._estop = False
        self._estop_reason = ""

    def get_estop_reason(self) -> str:
        return str(self._estop_reason)

    def set_leg_action(
        self,
        *,
        hipz: Optional[float] = None,
        hipx: Optional[float] = None,
        hipy: Optional[float] = None,
        knee: Optional[float] = None,
        ankle_pitch: Optional[float] = None,
        ankle_roll: Optional[float] = None,
        velocity_deg_s: Union[float, int] = 0.0,
        torque_nm: Union[float, int] = 0.0,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
    ) -> Dict[int, float]:
        """
        Update this leg's command target in joint space (degrees, calibrated).
        Only joints passed with a value are updated; the rest hold their last
        commanded target. `torque_nm` is accepted for API symmetry with
        BipedalRobotController.set_action but, like there, only takes effect
        if ENABLE_MIT_FEEDFORWARD_TORQUE is flipped on.
        Returns the effective raw motor-space send targets (deg), keyed by
        motor id, after clamp/wrap resolution.
        """
        requested = {
            "hipz": hipz, "hipx": hipx, "hipy": hipy, "knee": knee,
            "ankle_pitch": ankle_pitch, "ankle_roll": ankle_roll,
        }
        with self._action_lock:
            proposed = {mid: cmd for mid, cmd in self.action.items()}

            cur_cmd_raw = {mid: float(proposed[mid].position_deg) for mid in self.motor_ids}
            q_cmd_deg = self._motor_to_joint(cur_cmd_raw)
            for name, value in requested.items():
                if value is not None:
                    q_cmd_deg[name] = float(value)

            qd_cmd_deg_s = {name: float(velocity_deg_s) for name in LEG_JOINT_NAMES}
            tau_cmd_nm = {name: 0.0 for name in LEG_JOINT_NAMES}
            if ENABLE_MIT_FEEDFORWARD_TORQUE:
                tau_cmd_nm = {name: float(torque_nm) for name in LEG_JOINT_NAMES}

            try:
                target_raw = self._joint_to_motor(q_cmd_deg, output_space="raw")
            except AnkleUnreachableError as e:
                print(f"[WARN] ankle action update rejected: {e}")
                target_raw = None
            vel_raw, tau_raw = self._joint_vel_tau_to_motor_raw(qd_cmd_deg_s, tau_cmd_nm)

            if target_raw is None or not self._passes_action_update_jump_guard(target_raw):
                req_raw = {mid: float(proposed[mid].position_deg) for mid in self.motor_ids}
            else:
                for mid in self.motor_ids:
                    proposed[mid] = MotorCommand(
                        position_deg=float(target_raw[mid]),
                        velocity_deg_s=float(vel_raw[mid]),
                        torque_nm=float(tau_raw[mid]),
                        kp=proposed[mid].kp if kp is None else kp,
                        kd=proposed[mid].kd if kd is None else kd,
                    )
                self.action = proposed
                req_raw = {mid: float(proposed[mid].position_deg) for mid in self.motor_ids}
        return self._effective_send_raw_map(req_raw)

    def get_joint_state(self, *, output_radians: bool = False) -> Dict[str, float]:
        """Return this leg's measured joint state, keyed by LEG_JOINT_NAMES."""
        with self._state_lock:
            snapshot = {mid: st for mid, st in self.state.items()}
        raw = {mid: float(snapshot[mid].position_deg) for mid in self.motor_ids}
        q_deg = self._motor_to_joint(raw)
        if output_radians:
            return {k: float(np.radians(v)) for k, v in q_deg.items()}
        return q_deg

    def get_state_snapshot(self) -> Dict[int, MotorState]:
        with self._state_lock:
            return {mid: st for mid, st in self.state.items()}

    def start(self, *, mode: str = "state_only", auto_enable: bool = False) -> None:
        self.set_mode(mode)
        self._estop = False
        self._estop_reason = ""
        self._state_only_enforced = False
        self._open_log()
        if auto_enable and mode == "control":
            self.enable_all()
        if self._loop_thread is None:
            self._running = True
            self._loop_thread = threading.Thread(target=self._run_loop, daemon=True)
            self._loop_thread.start()

    def stop(self, *, disable_motors: bool = True) -> None:
        self._running = False
        if self._loop_thread is not None:
            self._loop_thread.join(timeout=2.0)
        self._loop_thread = None
        if disable_motors:
            self.disable_all()
        self._close_log()
        try:
            self.bus.shutdown()
        except Exception:
            pass

    # -------------------------
    # CAN protocol wrappers
    # -------------------------
    def enable(self, motor_id: int) -> bool:
        if self._estop:
            return False
        self._send_cmd_ff(motor_id, CAN_CMD_ENABLE)
        return self._recv_motor_reply(motor_id, self.recv_timeout_s) is not None

    def disable(self, motor_id: int) -> bool:
        self._send_cmd_ff(motor_id, CAN_CMD_DISABLE)
        return self._recv_motor_reply(motor_id, self.recv_timeout_s) is not None

    def set_zero(self, motor_id: int, *, wait_s: float = 0.5):
        if self._estop:
            return None
        data = [0xFF] * 7 + [CAN_CMD_ZERO]
        self._send8(motor_id, data)
        return self._recv_motor_reply(motor_id, wait_s)

    def enable_all(self) -> None:
        for mid in self.motor_ids:
            self.enable(mid)

    def disable_all(self) -> None:
        for mid in self.motor_ids:
            self.disable(mid)

    def set_zero_all(self) -> None:
        for mid in self.motor_ids:
            self.set_zero(mid)

    def zero_here(self, *, settle_s: float = 0.2) -> Dict[str, float]:
        """Define the leg's CURRENT physical pose as the zero/home reference.

        Sends the RobStride 'set mechanical zero' command (0xFE) to every motor
        on this leg, so wherever each joint is sitting right now becomes 0. The
        motor stores this zero, so it persists across power cycles until you
        re-zero -- do it once at assembly and you don't have to repeat it every
        run (or pass zero_on_start=True to verify_and_start to repeat it).

        SAFETY: only call this with the leg physically placed at the intended
        home pose (e.g. against an assembly jig / hard-stop). It does NOT move
        anything, but it redefines what '0' means for every joint limit and
        command -- zeroing at a random pose makes all of them reference that
        random pose. Motors are disabled first so they aren't holding torque
        while re-referencing. Call this before start()/verify_and_start switches
        the leg into control mode (so the background CAN loop isn't contending
        for the bus). Returns the measured joint state right after zeroing
        (each joint should now read ~0).
        """
        self.disable_all()
        time.sleep(settle_s)
        self.set_zero_all()
        time.sleep(settle_s)
        # Pull fresh state so the new ~0 readings are visible to callers/checks.
        self._request_state(self.motor_ids, settle_s=settle_s)
        q = self.get_joint_state()
        print(
            f"[{self.side}] zeroed at current pose -> joint state now: "
            f"{ {k: round(float(v), 2) for k, v in q.items()} }"
        )
        return q

    # -------------------------
    # Diagnostics: connectivity check + max-ROM sweep tests
    # -------------------------
    def check_connectivity(self, *, timeout_s: float = 0.3, attempts: int = 3) -> Dict[int, bool]:
        """
        Read-only check: request state from every motor on this leg and report
        which ones replied. Safe to call any time (does not enable/move
        anything). Intended to be run in mode="state_only".
        """
        responded: Dict[int, bool] = {mid: False for mid in self.motor_ids}
        for _ in range(attempts):
            missing = self._request_state(self.motor_ids, settle_s=timeout_s)
            for mid in self.motor_ids:
                if mid not in missing:
                    responded[mid] = True
            if all(responded.values()):
                break
        return responded

    def verify_and_start(self, *, auto_enable: bool = True, timeout_s: float = 0.3, attempts: int = 3, zero_on_start: bool = False) -> Dict[str, Any]:
        """
        Safety-gated startup:
          1. check_connectivity() -- every motor on this leg must respond.
          1b. If zero_on_start=True: zero_here() -- redefine the leg's CURRENT
             pose as home (0) before the range check and before enabling. Only
             use this when the leg is physically placed at its intended home
             pose; see zero_here()'s safety note. Defaults to False so an
             already-stored zero (set once at assembly) is respected.
          2. Read each joint's CURRENT angle and check it's within its
             configured range (JOINT_LIMITS_DEG for hip/knee, ROLL_LIMIT_DEG/
             PITCH_LIMIT_DEG for the ankle) -- i.e. the leg must already be
             sitting somewhere sane before control starts, not just after.
          3. Only if every motor responded AND every joint is in-range,
             switch to mode="control" (via start()) and optionally enable
             motors. Otherwise raises RuntimeError and does NOT start control.

        Returns a report dict: {"connectivity": {...}, "joint_angles_deg": {...},
        "in_range": {...}}.
        """
        connectivity = self.check_connectivity(timeout_s=timeout_s, attempts=attempts)
        report: Dict[str, Any] = {"connectivity": connectivity}

        if not all(connectivity.values()):
            missing = [mid for mid, ok in connectivity.items() if not ok]
            raise RuntimeError(
                f"[{self.side}] verify_and_start aborted: motor(s) {missing} did not respond. "
                f"Full report: {connectivity}"
            )

        if zero_on_start:
            # Redefine the current pose as home (0) before the range check and
            # before enabling. Assumes the leg is sitting at its intended home
            # pose right now -- see zero_here()'s safety note.
            self.zero_here()

        with self._state_lock:
            snapshot = {mid: st for mid, st in self.state.items()}
        q_deg = self.get_joint_state()
        m_hipy, m_hipx, m_hipz, m_knee = self.hip_knee_ids
        joint_to_motor = {"hipy": m_hipy, "hipx": m_hipx, "hipz": m_hipz, "knee": m_knee}

        in_range: Dict[str, bool] = {}
        problems = []
        display_angles: Dict[str, float] = {}
        for name in ("hipz", "hipx", "hipy", "knee"):
            mid = joint_to_motor[name]
            lim = self.command_limits_cal_deg.get(mid)
            raw = float(snapshot[mid].position_deg)
            if lim is None:
                wrapped = self._raw_to_cal(mid, raw)
                ok = True
            else:
                # Same wrap-nearest-to-range logic _state_in_bounds uses --
                # a raw angle read as e.g. 354 deg (a wrapped -6 deg) must be
                # compared against its representative inside/near the
                # configured interval, not the raw sign*raw+offset value.
                wrapped = self._raw_to_cal_near_interval(mid, raw, lim[0], lim[1])
                ok = lim[0] <= wrapped <= lim[1]
            display_angles[name] = wrapped
            in_range[name] = ok
            if not ok:
                problems.append(f"{name} (m{mid}) = {wrapped:.2f} deg (raw={raw:.2f}), expected within {lim}")

        display_angles["ankle_pitch"] = q_deg["ankle_pitch"]
        display_angles["ankle_roll"] = q_deg["ankle_roll"]
        pitch_ok = PITCH_LIMIT_DEG[0] <= q_deg["ankle_pitch"] <= PITCH_LIMIT_DEG[1]
        roll_ok = ROLL_LIMIT_DEG[0] <= q_deg["ankle_roll"] <= ROLL_LIMIT_DEG[1]
        in_range["ankle_pitch"] = pitch_ok
        in_range["ankle_roll"] = roll_ok
        if not pitch_ok:
            problems.append(f"ankle_pitch = {q_deg['ankle_pitch']:.2f} deg, expected within {PITCH_LIMIT_DEG}")
        if not roll_ok:
            problems.append(f"ankle_roll = {q_deg['ankle_roll']:.2f} deg, expected within {ROLL_LIMIT_DEG}")

        report["joint_angles_deg"] = {k: round(float(v), 3) for k, v in display_angles.items()}
        report["in_range"] = in_range

        print(f"[{self.side}] connectivity: {connectivity}")
        print(f"[{self.side}] current joint angles (deg): {report['joint_angles_deg']}")
        print(f"[{self.side}] in-range: {in_range}")

        if problems:
            raise RuntimeError(
                f"[{self.side}] verify_and_start aborted: current pose out of range:\n  "
                + "\n  ".join(problems)
            )

        self.start(mode="control", auto_enable=auto_enable)
        print(f"[{self.side}] all checks passed -- control started (auto_enable={auto_enable}).")
        return report

    def run_joint_max_rom_sweep(
        self,
        joint_name: str,
        *,
        repeats: int = 3,
        rom_scale: Optional[float] = None,
        step_deg: Optional[float] = None,
        settle_s: float = 0.05,
        hold_s: float = 0.5,
    ) -> None:
        """
        Sweep ONE joint (by LEG_JOINT_NAMES name) between its configured
        limits, `repeats` times, ramping in small steps (respecting
        max_cmd_delta_deg / command limits / the ankle's kinematic envelope
        via set_leg_action's existing guards). Blocks until done. Requires
        mode="control" and the loop already running.

        rom_scale (default self.rom_scale, conservatively 0.3) shrinks the
        swept range toward the midpoint of the joint's configured limits --
        e.g. 0.3 only travels 30% of the way to each limit. Raise it toward
        1.0 across successive test rounds via set_rom_scale().

        step_deg (default 3.0 deg/step) sets how big each ramp step is;
        every step still passes through set_leg_action's existing guards
        (max_cmd_delta_deg, command limits, ankle kinematic envelope).
        """
        if joint_name not in LEG_JOINT_NAMES:
            raise ValueError(f"joint_name must be one of {LEG_JOINT_NAMES}")
        if self.mode != "control":
            raise RuntimeError("run_joint_max_rom_sweep requires mode='control' to already be running")

        if rom_scale is None:
            rom_scale = self.rom_scale
        if not (0.0 < rom_scale <= 1.0):
            raise ValueError("rom_scale must be in (0, 1.0]")

        if step_deg is None:
            step_deg = 3.0

        if joint_name in ("ankle_pitch", "ankle_roll"):
            lo, hi = (PITCH_LIMIT_DEG if joint_name == "ankle_pitch" else ROLL_LIMIT_DEG)
        else:
            m_hipy, m_hipx, m_hipz, m_knee = self.hip_knee_ids
            mid = {"hipz": m_hipz, "hipx": m_hipx, "hipy": m_hipy, "knee": m_knee}[joint_name]
            cal_lim = self.command_limits_cal_deg.get(mid)
            if cal_lim is None:
                raise RuntimeError(f"No JOINT_LIMITS_DEG configured for motor {mid} ({joint_name})")
            lo, hi = cal_lim
            lo += COMMAND_MARGIN_DEG
            hi -= COMMAND_MARGIN_DEG

        mid_deg = (lo + hi) / 2.0
        lo = mid_deg + (lo - mid_deg) * rom_scale
        hi = mid_deg + (hi - mid_deg) * rom_scale

        def ramp_to(target_deg: float):
            current = self.get_joint_state()[joint_name]
            n_steps = max(1, int(abs(target_deg - current) / step_deg))
            for i in range(1, n_steps + 1):
                val = current + (target_deg - current) * (i / n_steps)
                self.set_leg_action(**{joint_name: val})
                time.sleep(settle_s)

        try:
            for i in range(repeats):
                print(f"[ROM TEST] {self.side}/{joint_name} sweep {i+1}/{repeats}: -> {hi:.1f}")
                ramp_to(hi)
                time.sleep(hold_s)
                print(f"[ROM TEST] {self.side}/{joint_name} sweep {i+1}/{repeats}: -> {lo:.1f}")
                ramp_to(lo)
                time.sleep(hold_s)
            ramp_to(0.0)
        except BaseException as e:
            # Covers KeyboardInterrupt (Ctrl+C) as well as any runtime error
            # mid-sweep: disable this leg immediately rather than leaving it
            # holding a stale target, then re-raise so the caller sees it.
            print(f"[ROM TEST] {self.side}/{joint_name} interrupted/failed ({type(e).__name__}: {e}) -- disabling")
            try:
                self.disable_all()
            except Exception as disable_err:
                print(f"[WARN] {self.side}: error while disabling: {disable_err}")
            raise

    def run_ankle_trajectory(
        self,
        waypoints: list[tuple[float, float]],
        *,
        step_deg: float = 1.0,
        settle_s: float = 0.05,
        hold_s: float = 0.0,
    ) -> None:
        """
        Move the ankle through a sequence of (pitch_deg, roll_deg) waypoints,
        linearly interpolating between each consecutive pair in `step_deg`
        increments (e.g. (0,0) -> (-1,0) -> (-2,0) -> ... -> (-20,0) ->
        (-20,-1) -> ... reproduces the classic axis-then-axis "L-shaped"
        sweep -- pass waypoints=[(0,0), (-20,0), (-20,-B)] for that).

        Every intermediate point is sent through set_leg_action, so it still
        goes through the existing guards (max_cmd_delta_deg, command limits,
        and AnkleKinematics' feasibility/ROLL_LIMIT_DEG/PITCH_LIMIT_DEG check
        -- a waypoint outside the allowed envelope stops the trajectory with
        a clear rejection message rather than silently skipping it).

        Requires mode="control" already running.
        """
        if self.mode != "control":
            raise RuntimeError("run_ankle_trajectory requires mode='control' to already be running")
        if len(waypoints) < 2:
            raise ValueError("waypoints needs at least 2 points (start, end)")

        a1_id, a2_id = self.ankle_ids
        try:
            for seg_idx in range(len(waypoints) - 1):
                p0, r0 = waypoints[seg_idx]
                p1, r1 = waypoints[seg_idx + 1]
                n_steps = max(1, int(max(abs(p1 - p0), abs(r1 - r0)) / step_deg))
                print(f"[ANKLE TRAJ] {self.side} segment {seg_idx+1}/{len(waypoints)-1}: "
                      f"({p0:.1f},{r0:.1f}) -> ({p1:.1f},{r1:.1f}) in {n_steps} steps")
                for i in range(1, n_steps + 1):
                    frac = i / n_steps
                    pitch = p0 + (p1 - p0) * frac
                    roll = r0 + (r1 - r0) * frac
                    before = (self.action[a1_id].position_deg, self.action[a2_id].position_deg)
                    self.set_leg_action(ankle_pitch=pitch, ankle_roll=roll)
                    after = (self.action[a1_id].position_deg, self.action[a2_id].position_deg)
                    if after == before:
                        # A no-op result partway through a segment means this point
                        # was rejected (out of envelope / jump guard / limits) --
                        # set_leg_action already printed the [WARN] reason.
                        print(f"[WARN] {self.side}: trajectory point ({pitch:.2f},{roll:.2f}) rejected, stopping")
                        return
                    time.sleep(settle_s)
                if hold_s > 0:
                    time.sleep(hold_s)
        except BaseException as e:
            print(f"[ANKLE TRAJ] {self.side} interrupted/failed ({type(e).__name__}: {e}) -- disabling")
            try:
                self.disable_all()
            except Exception as disable_err:
                print(f"[WARN] {self.side}: error while disabling: {disable_err}")
            raise

    # -------------------------
    # Internal loop
    # -------------------------
    def _run_loop(self) -> None:
        period = 1.0 / max(1.0, self.control_hz)
        next_tick = time.perf_counter()

        while self._running:
            self._cycle_idx += 1
            missing: list[int] = []

            if self.mode == "state_only":
                if self.reset_estop_each_state_cycle:
                    self.clear_estop()
                missing = self._request_state(self.motor_ids)
                if not self._state_only_enforced:
                    # Hard safety: in state-only mode, keep actuators disabled.
                    self.disable_all()
                    self._state_only_enforced = True
                self._damping_active = False
                self._action_initialized = False
            elif not self._estop:
                self._state_only_enforced = False
                if not self._has_any_valid_state():
                    missing = self._request_state(self.motor_ids)
                elif (self._cycle_idx % 50) == 0:
                    stale = self._stale_motor_ids(stale_s=0.5)
                    if stale:
                        missing = self._request_state(stale)

                if not self._action_initialized:
                    # Safe startup: hold current measured pose before accepting motion updates.
                    self._latch_action_from_state()
                    self._action_initialized = True

                if self._is_near_stop():
                    self._damping_active = True
                    self._send_damping_all()
                else:
                    self._damping_active = False
                    self._send_action_all()

            self._log_state(missing)

            next_tick += period
            sleep_s = next_tick - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.perf_counter()

    def _has_any_valid_state(self) -> bool:
        with self._state_lock:
            return any(st.stamp > 0.0 for st in self.state.values())

    def _stale_motor_ids(self, *, stale_s: float = 0.5) -> list[int]:
        now = time.time()
        stale: list[int] = []
        with self._state_lock:
            snapshot = {mid: st for mid, st in self.state.items()}
        for mid in self.motor_ids:
            st = snapshot[mid]
            if st.stamp <= 0.0 or (now - st.stamp) > float(stale_s):
                stale.append(mid)
        return stale

    def _latch_action_from_state(self) -> None:
        with self._state_lock:
            snapshot = {mid: st for mid, st in self.state.items()}
        with self._action_lock:
            for mid in self.motor_ids:
                prev = self.action[mid]
                st = snapshot[mid]
                self.action[mid] = MotorCommand(
                    position_deg=float(st.position_deg),
                    velocity_deg_s=0.0,
                    torque_nm=0.0,
                    kp=prev.kp,
                    kd=prev.kd,
                )

    # -------------------------
    # State request / decode
    # -------------------------
    def _request_state(self, motor_ids, *, settle_s: float = 0.001) -> list[int]:
        ids = list(motor_ids)
        touched: set[int] = set()
        with self._tx_lock:
            for mid in ids:
                data = [0xFF] * 7 + [CAN_CMD_CLEAR_FAULT]
                msg = can.Message(arbitration_id=int(mid), data=data, is_extended_id=False)
                self.bus.send(msg)
        if settle_s > 0.0:
            time.sleep(settle_s)
        touched.update(self._drain_rx_states(timeout_s=self.recv_timeout_s, max_msgs=self.RX_FLUSH_MAX_MSGS))
        return [mid for mid in ids if mid not in touched]

    def _try_update_state_from_msg(self, msg: "can.Message") -> Optional[int]:
        try:
            raw = bytes(msg.data)
            mid = int(raw[0])
            if mid not in self.motor_ids:
                return None
            spec = MOTORS.get(mid)
            if spec is None:
                return None
            motor_id, st = decode_state_frame(raw, pmax=spec.pmax_rad, vmax=spec.vmax_rad_s, tmax=spec.tmax_nm)
            st.stamp = time.time()
            with self._state_lock:
                self.state[motor_id] = st

            if not self._state_in_bounds(motor_id, st.position_deg):
                reason = (
                    f"State out of limits on motor {motor_id}: {st.position_deg:.2f} deg, "
                    f"limits={self.joint_limits_deg.get(motor_id)}, margin={STATE_MARGIN_DEG}"
                )
                if not self._estop:
                    print(f"[WARN] E-STOP triggered: {reason}")
                self._estop = True
                self._estop_reason = reason
                self.disable_all()
            return motor_id
        except Exception:
            return None

    def _drain_rx_states(self, *, timeout_s: float, max_msgs: int) -> list[int]:
        touched: list[int] = []
        drained = 0
        timeout = max(0.0, float(timeout_s))
        while drained < max_msgs:
            with self._rx_lock:
                rx = self.bus.recv(timeout)
            if rx is None:
                break
            mid = self._try_update_state_from_msg(rx)
            if mid is not None:
                touched.append(mid)
            drained += 1
        return touched

    def _recv_motor_reply(self, motor_id: int, timeout_s: float):
        timeout = max(0.0, float(timeout_s))
        deadline = time.perf_counter() + timeout
        while True:
            remaining = deadline - time.perf_counter()
            if remaining <= 0.0:
                return None
            with self._rx_lock:
                rx = self.bus.recv(remaining)
            if rx is None:
                return None
            mid = self._try_update_state_from_msg(rx)
            if mid == int(motor_id):
                return rx

    # -------------------------
    # MIT control / damping
    # -------------------------
    def _send_action_all(self) -> None:
        with self._action_lock:
            commands = {mid: cmd for mid, cmd in self.action.items()}
        send_raw = self._build_send_raw_from_joint_error(commands)
        for mid in self.motor_ids:
            cmd = commands[mid]
            cmd_eff = MotorCommand(
                position_deg=float(send_raw[mid]),
                velocity_deg_s=float(cmd.velocity_deg_s),
                torque_nm=float(cmd.torque_nm),
                kp=cmd.kp,
                kd=cmd.kd,
            )
            payload = self._prepare_mit_payload(mid, cmd_eff, clamp=False)
            if payload is None:
                continue
            self._send8(mid, payload)
        self._drain_rx_states(timeout_s=self.recv_timeout_s, max_msgs=self.RX_FLUSH_MAX_MSGS)

    def _send_damping_all(self) -> None:
        with self._state_lock:
            snapshot = {mid: st for mid, st in self.state.items()}
        for mid in self.motor_ids:
            st = snapshot[mid]
            damping_cmd = MotorCommand(
                position_deg=float(st.position_deg),
                velocity_deg_s=0.0,
                torque_nm=0.0,
                kp=0.0,
                kd=DAMPING_KD,
            )
            payload = self._prepare_mit_payload(mid, damping_cmd, clamp=False)
            if payload is None:
                continue
            self._send8(mid, payload)
        self._drain_rx_states(timeout_s=self.recv_timeout_s, max_msgs=self.RX_FLUSH_MAX_MSGS)

    def _build_send_raw_from_joint_error(self, commands: Dict[int, MotorCommand]) -> Dict[int, float]:
        """
        Build raw motor targets from joint-space error (current measured vs.
        requested command), same approach as BipedalRobotController.
        """
        with self._state_lock:
            snapshot = {mid: st for mid, st in self.state.items()}
        cur_raw = {mid: float(snapshot[mid].position_deg) for mid in self.motor_ids}
        req_raw = {mid: float(commands[mid].position_deg) for mid in self.motor_ids}

        q_cur = self._motor_to_joint(cur_raw)
        q_req = self._motor_to_joint(req_raw)
        q_tgt = {name: q_cur[name] + (q_req[name] - q_cur[name]) for name in LEG_JOINT_NAMES}

        try:
            tgt_raw = self._joint_to_motor(q_tgt, output_space="raw")
        except AnkleUnreachableError as e:
            print(f"[WARN] ankle target became unreachable mid-loop ({e}); holding current pose")
            tgt_raw = cur_raw
        out: Dict[int, float] = {}
        for mid in self.motor_ids:
            st = snapshot[mid]
            raw = float(tgt_raw[mid])
            if st.stamp > 0.0:
                raw = self._resolve_raw_target_near_current(mid, raw, float(st.position_deg))
            out[mid] = float(raw)
        return out

    def _prepare_mit_payload(self, motor_id: int, cmd: MotorCommand, *, clamp: bool) -> Optional[bytes]:
        if self._estop:
            return None
        spec = MOTORS[motor_id]
        gains = self.gains[motor_id]
        kp = gains.kp if cmd.kp is None else float(cmd.kp)
        kd = gains.kd if cmd.kd is None else float(cmd.kd)
        req_raw = float(cmd.position_deg)

        with self._state_lock:
            st = self.state[motor_id]
        if st.stamp <= 0.0:
            now = time.time()
            if now - self._last_jump_warn_s[motor_id] > 0.5:
                print(f"[WARN] m{motor_id}: no valid state yet, skipping command send")
                self._last_jump_warn_s[motor_id] = now
            return None

        cur_raw = float(st.position_deg)
        pos_raw = self._resolve_raw_target_near_current(motor_id, req_raw, cur_raw)
        if clamp and not self._raw_command_in_limits(motor_id, pos_raw, ref_raw_deg=cur_raw):
            now = time.time()
            if now - self._last_jump_warn_s[motor_id] > 0.5:
                lo, hi = self._raw_limits_near_ref(motor_id, self._raw_command_limits().get(motor_id, (-np.inf, np.inf)), cur_raw)
                print(
                    f"[WARN] m{motor_id}: command out of limits, req_raw={req_raw:.1f}, tgt_raw={pos_raw:.1f}, "
                    f"allowed=({lo + COMMAND_MARGIN_DEG:.1f}, {hi - COMMAND_MARGIN_DEG:.1f}), skipping send"
                )
                self._last_jump_warn_s[motor_id] = now
            return None
        delta = abs(pos_raw - cur_raw)
        if delta > self.max_cmd_delta_deg:
            now = time.time()
            if now - self._last_jump_warn_s[motor_id] > 0.5:
                print(
                    f"[WARN] m{motor_id}: command jump too large ({delta:.1f} deg > {self.max_cmd_delta_deg:.1f} deg), "
                    f"cur_raw={cur_raw:.1f}, req_raw={req_raw:.1f}, tgt_raw={pos_raw:.1f}, skipping send"
                )
                self._last_jump_warn_s[motor_id] = now
            return None

        return pack_mit_command(
            position_deg=pos_raw,
            velocity_deg_s=float(cmd.velocity_deg_s),
            kp=kp,
            kd=kd,
            torque_nm=float(cmd.torque_nm),
            pmax=spec.pmax_rad,
            vmax=spec.vmax_rad_s,
            tmax=spec.tmax_nm,
        )

    def _passes_action_update_jump_guard(self, target_raw: Dict[int, float]) -> bool:
        with self._state_lock:
            snapshot = {mid: st for mid, st in self.state.items()}

        for mid in self.motor_ids:
            st = snapshot[mid]
            if st.stamp <= 0.0:
                continue
            cur_raw = float(st.position_deg)
            tgt_raw = self._resolve_raw_target_near_current(mid, float(target_raw[mid]), cur_raw)
            if self.enforce_command_limits and not self._raw_command_in_limits(mid, tgt_raw, ref_raw_deg=cur_raw):
                lo, hi = self._raw_limits_near_ref(mid, self._raw_command_limits().get(mid, (-np.inf, np.inf)), cur_raw)
                print(
                    f"[WARN] m{mid}: action update out of limits, tgt_raw={tgt_raw:.1f}, "
                    f"allowed=({lo + COMMAND_MARGIN_DEG:.1f}, {hi - COMMAND_MARGIN_DEG:.1f}), rejecting action update"
                )
                return False
            delta = abs(tgt_raw - cur_raw)
            if delta > self.max_cmd_delta_deg:
                print(
                    f"[WARN] m{mid}: action update jump too large ({delta:.1f} deg > {self.max_cmd_delta_deg:.1f} deg), "
                    f"cur_raw={cur_raw:.1f}, tgt_raw={tgt_raw:.1f}, rejecting action update"
                )
                return False
        return True

    def _effective_send_raw_map(self, req_raw_map: Dict[int, float]) -> Dict[int, float]:
        out: Dict[int, float] = {}
        with self._state_lock:
            snapshot = {mid: st for mid, st in self.state.items()}
        for mid in self.motor_ids:
            req_raw = float(req_raw_map[mid])
            st = snapshot[mid]
            tgt_raw = req_raw
            if st.stamp > 0.0:
                tgt_raw = self._resolve_raw_target_near_current(mid, tgt_raw, float(st.position_deg))
            out[mid] = float(tgt_raw)
        return out

    # -------------------------
    # Safety
    # -------------------------
    def _state_in_bounds(self, motor_id: int, pos_deg: float) -> bool:
        lim = self.command_limits_cal_deg.get(motor_id)
        if lim is None:
            return True
        lo, hi = lim
        value = self._raw_to_cal_near_interval(motor_id, float(pos_deg), lo, hi)
        return (lo - STATE_MARGIN_DEG) <= value <= (hi + STATE_MARGIN_DEG)

    def _raw_command_limits(self) -> Dict[int, tuple[float, float]]:
        out: Dict[int, tuple[float, float]] = {}
        for mid, lim in self.joint_limits_deg.items():
            lo, hi = float(lim[0]), float(lim[1])
            if self._limit_source == "raw":
                out[mid] = (min(lo, hi), max(lo, hi))
            else:
                lo_raw, hi_raw = self._cal_to_raw(mid, lo), self._cal_to_raw(mid, hi)
                out[mid] = (min(lo_raw, hi_raw), max(lo_raw, hi_raw))
        return out

    def _raw_limits_near_ref(self, motor_id: int, lim_raw: tuple[float, float], ref_raw_deg: Optional[float]) -> tuple[float, float]:
        lo, hi = float(lim_raw[0]), float(lim_raw[1])
        if ref_raw_deg is None:
            return lo, hi
        ref = float(ref_raw_deg)
        best = (lo, hi)
        best_dist = float("inf")
        for k in range(-4, 5):
            cand_lo, cand_hi = lo + 360.0 * k, hi + 360.0 * k
            center = 0.5 * (cand_lo + cand_hi)
            dist = abs(center - ref)
            if dist < best_dist:
                best, best_dist = (cand_lo, cand_hi), dist
        return best

    def _raw_command_in_limits(self, motor_id: int, pos_raw_deg: float, ref_raw_deg: Optional[float] = None) -> bool:
        lim_raw = self._raw_command_limits().get(int(motor_id))
        if lim_raw is None:
            return True
        lo, hi = self._raw_limits_near_ref(int(motor_id), lim_raw, ref_raw_deg)
        lo_safe, hi_safe = lo + COMMAND_MARGIN_DEG, hi - COMMAND_MARGIN_DEG
        if lo_safe > hi_safe:
            lo_safe, hi_safe = lo, hi
        return lo_safe <= float(pos_raw_deg) <= hi_safe

    def _recompute_command_limits(self) -> None:
        out: Dict[int, tuple[float, float]] = {}
        for mid, lim in self.joint_limits_deg.items():
            lo, hi = float(lim[0]), float(lim[1])
            if self._limit_source == "raw":
                lo_cal, hi_cal = self._raw_to_cal(mid, lo), self._raw_to_cal(mid, hi)
                out[mid] = (min(lo_cal, hi_cal), max(lo_cal, hi_cal))
            else:
                out[mid] = (min(lo, hi), max(lo, hi))
        self.command_limits_cal_deg = out

    def _is_near_stop(self) -> bool:
        with self._state_lock:
            snapshot = {mid: st for mid, st in self.state.items()}
        for mid, st in snapshot.items():
            lim = self.command_limits_cal_deg.get(mid)
            if lim is None:
                continue
            lo, hi = lim
            value = self._raw_to_cal_near_interval(mid, float(st.position_deg), lo, hi)
            if value <= lo + NEAR_STOP_MARGIN_DEG or value >= hi - NEAR_STOP_MARGIN_DEG:
                return True
        return False

    # -------------------------
    # Joint-space <-> motor-space conversions (single leg, with ankle coupling)
    # -------------------------
    def _motor_to_joint(self, motor_raw_deg: Dict[int, float]) -> Dict[str, float]:
        m_hipy, m_hipx, m_hipz, m_knee = self.hip_knee_ids
        a1_id, a2_id = self.ankle_ids
        # Report each hip/knee angle wrapped into the representative nearest its
        # configured limit interval (the same wrap logic _state_in_bounds /
        # verify_and_start already use). Without this, a raw angle read as e.g.
        # 350 deg -- a wrapped -10 deg -- is reported as 350, so any caller that
        # plans motion from get_joint_state() (ramp_to, an external policy) sees
        # it as out of range and travels the long way around (350 -> 0 as -350
        # deg instead of +10 deg). Limit checks and the CAN send path already
        # wrap; only this state-reporting path was missing it.
        out: Dict[str, float] = {}
        for name, mid in (("hipz", m_hipz), ("hipx", m_hipx), ("hipy", m_hipy), ("knee", m_knee)):
            lim = self.command_limits_cal_deg.get(mid)
            if lim is None:
                out[name] = self._raw_to_cal(mid, motor_raw_deg[mid])
            else:
                out[name] = self._raw_to_cal_near_interval(mid, float(motor_raw_deg[mid]), lim[0], lim[1])
        # a1/a2 raw degrees ARE alpha1/alpha2 for AnkleKinematics (see __init__ note).
        pitch_deg, roll_deg = self.ankle_kin.solve_fk(
            motor_raw_deg[a1_id], motor_raw_deg[a2_id],
            guess_pitch_deg=self._last_ankle_pitch_deg, guess_roll_deg=self._last_ankle_roll_deg,
        )
        self._last_ankle_pitch_deg, self._last_ankle_roll_deg = pitch_deg, roll_deg
        out["ankle_pitch"], out["ankle_roll"] = pitch_deg, roll_deg
        return out

    def _joint_to_motor(self, q_deg: Dict[str, float], *, output_space: str = "raw") -> Dict[int, float]:
        if output_space not in ("calibrated", "raw"):
            raise ValueError("output_space must be 'calibrated' or 'raw'")
        m_hipy, m_hipx, m_hipz, m_knee = self.hip_knee_ids
        a1_id, a2_id = self.ankle_ids
        out_cal = {
            m_hipz: float(q_deg["hipz"]),
            m_hipx: float(q_deg["hipx"]),
            m_hipy: float(q_deg["hipy"]),
            m_knee: float(q_deg["knee"]),
        }
        # solve_ik raises AnkleUnreachableError if outside the software test
        # envelope (ROLL_LIMIT_DEG/PITCH_LIMIT_DEG) or outside what the
        # linkage can physically reach -- callers must handle this the same
        # way they handle a rejected jump/limit (see set_leg_action).
        a1, a2 = self.ankle_kin.solve_ik(float(q_deg["ankle_pitch"]), float(q_deg["ankle_roll"]))
        if output_space == "calibrated":
            # ankle motors have no separate "calibrated" space of their own now
            # (AnkleKinematics IS the calibration) -- return alpha directly.
            out_cal[a1_id] = a1
            out_cal[a2_id] = a2
            return out_cal
        out_raw = {mid: self._cal_to_raw(mid, cal_deg) for mid, cal_deg in out_cal.items()}
        out_raw[a1_id] = a1
        out_raw[a2_id] = a2
        return out_raw

    def _joint_vel_tau_to_motor_raw(self, qd_deg_s: Dict[str, float], tau_nm: Dict[str, float]) -> tuple[Dict[int, float], Dict[int, float]]:
        m_hipy, m_hipx, m_hipz, m_knee = self.hip_knee_ids
        a1_id, a2_id = self.ankle_ids
        qd_raw: Dict[int, float] = {}
        tau_raw: Dict[int, float] = {}

        for mid, name in ((m_hipz, "hipz"), (m_hipx, "hipx"), (m_hipy, "hipy"), (m_knee, "knee")):
            s = float(self.motor_sign[mid])
            if abs(s) < 1e-9:
                raise ValueError(f"Invalid motor sign for m{mid}: {s}")
            qd_raw[mid] = float(qd_deg_s[name] / s)
            tau_raw[mid] = float(tau_nm[name] * s)

        # NOTE: ankle velocity/torque feedforward through the nonlinear IK
        # would need a proper Jacobian; until that's added, ankle motors get
        # zero feedforward velocity/torque here (position/kp,kd still drive
        # them correctly via _build_send_raw_from_joint_error).
        qd_raw[a1_id] = 0.0
        qd_raw[a2_id] = 0.0
        tau_raw[a1_id] = 0.0
        tau_raw[a2_id] = 0.0
        return qd_raw, tau_raw

    # -------------------------
    # Raw <-> calibrated helpers (per motor)
    # -------------------------
    def _raw_to_cal(self, motor_id: int, raw_deg: float) -> float:
        return float(self.motor_sign[motor_id] * float(raw_deg) + self.motor_offset_deg[motor_id])

    def _cal_to_raw(self, motor_id: int, cal_deg: float) -> float:
        sign = float(self.motor_sign[motor_id])
        if abs(sign) < 1e-9:
            raise ValueError(f"Invalid MOTOR_SIGN for motor {motor_id}: {sign}")
        return float((float(cal_deg) - self.motor_offset_deg[motor_id]) / sign)

    def _raw_to_cal_near_interval(self, motor_id: int, raw_deg: float, lo: float, hi: float) -> float:
        base = self._raw_to_cal(motor_id, raw_deg)
        step = 360.0 * float(self.motor_sign[motor_id])
        center = 0.5 * (float(lo) + float(hi))
        best, best_dist, best_center = base, float("inf"), float("inf")
        for k in range(-4, 5):
            cand = base + step * k
            if cand < lo:
                dist = lo - cand
            elif cand > hi:
                dist = cand - hi
            else:
                dist = 0.0
            cdist = abs(cand - center)
            if (dist < best_dist) or (dist == best_dist and cdist < best_center):
                best, best_dist, best_center = cand, dist, cdist
        return float(best)

    def _resolve_raw_target_near_current(self, motor_id: int, raw_target_deg: float, raw_current_deg: float) -> float:
        spec = MOTORS[motor_id]
        pmax_deg = float(np.degrees(spec.pmax_rad))
        cur, tgt = float(raw_current_deg), float(raw_target_deg)
        best, best_dist = tgt, abs(tgt - cur)
        found_in_range = (-pmax_deg <= tgt <= pmax_deg)
        for k in range(-4, 5):
            cand = tgt + 360.0 * k
            dist = abs(cand - cur)
            in_range = (-pmax_deg <= cand <= pmax_deg)
            if found_in_range:
                if in_range and dist < best_dist:
                    best, best_dist = cand, dist
            else:
                if in_range:
                    best, best_dist, found_in_range = cand, dist, True
                elif dist < best_dist:
                    best, best_dist = cand, dist
        return float(best)

    # -------------------------
    # Low-level CAN send
    # -------------------------
    def _send8(self, motor_id: int, data8) -> None:
        msg = can.Message(arbitration_id=int(motor_id), data=list(data8), is_extended_id=False)
        with self._tx_lock:
            self.bus.send(msg)

    def _send_cmd_ff(self, motor_id: int, cmd: int) -> None:
        data = [0xFF] * 8
        data[7] = int(cmd)
        self._send8(motor_id, data)

    # -------------------------
    # Logging (optional -- only enabled if log_path is provided)
    # -------------------------
    def _open_log(self) -> None:
        if self.log_path is None or self._log_fp is not None:
            return
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log_fp = self.log_path.open("w", newline="")
        self._log_writer = csv.writer(self._log_fp)
        header = ["time_s", "mode", "estop", "estop_reason", "damping_active", "missing_ids"]
        for mid in self.motor_ids:
            header += [f"m{mid}_pos_deg", f"m{mid}_target_pos_deg", f"m{mid}_vel_deg_s", f"m{mid}_tau_nm"]
        for name in LEG_JOINT_NAMES:
            header.append(f"{self.side}_{name}_pos_deg")
        self._log_writer.writerow(header)
        self._log_fp.flush()

    def _log_state(self, missing_ids) -> None:
        if self._log_writer is None:
            return
        with self._state_lock:
            snapshot = {mid: st for mid, st in self.state.items()}
        with self._action_lock:
            action_snapshot = {mid: cmd for mid, cmd in self.action.items()}
        row = [
            time.time(), self.mode, int(self._estop), self._estop_reason,
            int(self._damping_active), ",".join(map(str, missing_ids)),
        ]
        for mid in self.motor_ids:
            st = snapshot[mid]
            row += [st.position_deg, action_snapshot[mid].position_deg, st.velocity_deg_s, st.torque_nm]
        raw = {mid: float(snapshot[mid].position_deg) for mid in self.motor_ids}
        q_deg = self._motor_to_joint(raw)
        row += [q_deg[name] for name in LEG_JOINT_NAMES]
        self._log_writer.writerow(row)
        self._log_fp.flush()

    def _close_log(self) -> None:
        if self._log_fp is not None:
            self._log_fp.close()
        self._log_fp = None
        self._log_writer = None


def run_full_rom_diagnostic(
    left_leg: Optional["SingleLegController"] = None,
    right_leg: Optional["SingleLegController"] = None,
    *,
    repeats: int = 3,
    rom_scale: float = 0.3,
    require_confirmed_motor_ids: bool = False,
) -> None:
    """
    Full test sequence:
      1. verify_and_start() on every leg passed in: connectivity check,
         current-angle range check, then switch to mode="control" -- only if
         everything checks out.
      2. For each leg, sweep each joint individually, ankle -> hip
         (ankle_pitch, ankle_roll, knee, hipy, hipx, hipz), `repeats` times
         at rom_scale x its full ROM, one joint moving at a time (others
         hold position).
      3. Both ankles' pitch+roll swept simultaneously, `repeats` times, same
         rom_scale -- ONLY if both left_leg and right_leg are provided (this
         phase is skipped for a single-leg run).

    Pass only the leg(s) you actually have connected -- e.g.
    run_full_rom_diagnostic(right_leg=right_leg) to test the right leg alone.
    At least one of left_leg/right_leg must be given.

    rom_scale (0, 1.0]: fraction of each joint's full configured range
    actually swept to, from the midpoint outward. Defaults to a conservative
    0.3 (30%) for first-time bring-up -- raise it toward 1.0 across
    successive test rounds once you trust the leg's behavior.

    Motor ID mapping confirmed: right leg hip/knee order is
    hipy(7), hipx(8), hipz(9), knee(10), ankle_a1/a2=11/12 on can1; left leg
    same joint order/numbering (1-6) on can0. require_confirmed_motor_ids is
    kept as a manual override in case that changes again -- pass True to
    re-block execution.
    """
    if left_leg is None and right_leg is None:
        raise ValueError("pass at least one of left_leg/right_leg")
    legs = [leg for leg in (left_leg, right_leg) if leg is not None]

    if require_confirmed_motor_ids:
        raise RuntimeError(
            "require_confirmed_motor_ids=True -- re-verify LEG_MOTOR_IDS_BY_SIDE / "
            "hip_knee_ids ordering against the real robot before proceeding."
        )
    if not (0.0 < rom_scale <= 1.0):
        raise ValueError("rom_scale must be in (0, 1.0]")

    print(f"=== 1. Connectivity + current-angle range check, then start control ({[l.side for l in legs]}) ===")
    for leg in legs:
        leg.verify_and_start(auto_enable=True)

    sweep_order = ["ankle_pitch", "ankle_roll", "knee", "hipy", "hipx", "hipz"]

    try:
        print(f"=== 2. Per-joint max-ROM sweeps (ankle -> hip), rom_scale={rom_scale} ===")
        for leg in legs:
            for joint_name in sweep_order:
                leg.run_joint_max_rom_sweep(joint_name, repeats=repeats, rom_scale=rom_scale)

        if left_leg is not None and right_leg is not None:
            print(f"=== 3. Both ankles simultaneously, rom_scale={rom_scale} ===")

            def ramp_both_to(pitch_target: float, roll_target: float, step_deg: float = 3.0, settle_s: float = 0.05):
                starts = {
                    leg.side: (leg.get_joint_state()["ankle_pitch"], leg.get_joint_state()["ankle_roll"])
                    for leg in (left_leg, right_leg)
                }
                n_steps = max(
                    1,
                    int(max(
                        abs(pitch_target - p) for p, r in starts.values()
                    ) / step_deg),
                    int(max(
                        abs(roll_target - r) for p, r in starts.values()
                    ) / step_deg),
                )
                for i in range(1, n_steps + 1):
                    frac = i / n_steps
                    for leg in (left_leg, right_leg):
                        p0, r0 = starts[leg.side]
                        leg.set_leg_action(
                            ankle_pitch=p0 + (pitch_target - p0) * frac,
                            ankle_roll=r0 + (roll_target - r0) * frac,
                        )
                    time.sleep(settle_s)

            pitch_hi = PITCH_LIMIT_DEG[1] * rom_scale
            pitch_lo = PITCH_LIMIT_DEG[0] * rom_scale
            roll_hi = ROLL_LIMIT_DEG[1] * rom_scale
            roll_lo = ROLL_LIMIT_DEG[0] * rom_scale
            for i in range(repeats):
                print(f"[ROM TEST] both ankles sweep {i+1}/{repeats}")
                ramp_both_to(pitch_hi, roll_hi)
                time.sleep(1.0)
                ramp_both_to(pitch_lo, roll_lo)
                time.sleep(1.0)
            ramp_both_to(0.0, 0.0)
        else:
            print(f"=== 3. Skipped (both-ankles-simultaneous needs both legs; only {legs[0].side} was given) ===")
    except BaseException as e:
        # Covers KeyboardInterrupt (Ctrl+C) as well as any runtime error mid-sweep:
        # disable every leg's motors immediately rather than leaving them holding
        # a stale target. Re-raised so the caller still sees what happened.
        print(f"[ROM TEST] interrupted/failed ({type(e).__name__}: {e}) -- disabling {[l.side for l in legs]} now")
        for leg in legs:
            try:
                leg.stop(disable_motors=True)
            except Exception as stop_err:
                print(f"[WARN] {leg.side}: error while stopping: {stop_err}")
        raise


# ---------------------------------------------------------------------------
# Interactive bring-up menu (used when this file is run directly)
# ---------------------------------------------------------------------------
def _prompt_int(prompt: str) -> Optional[int]:
    raw = input(prompt).strip()
    if raw == "":
        return None
    try:
        return int(raw)
    except ValueError:
        print("  Please enter a number.")
        return None


def _wrap_near_center(x: float, center: float) -> float:
    """Return x + 360*k that lands closest to `center` (collapse 360-deg wrap)."""
    best, best_dist = x, abs(x - center)
    for k in range(-4, 5):
        c = x + 360.0 * k
        d = abs(c - center)
        if d < best_dist:
            best, best_dist = c, d
    return best


def _read_motor_pos(leg: "SingleLegController", mid: int, *, settle_s: float = 0.1) -> Optional[float]:
    """Request one fresh state frame for `mid`, return its raw position (deg)."""
    leg.clear_estop()
    missing = leg._request_state([mid], settle_s=settle_s)
    with leg._state_lock:
        st = leg.state[mid]
    if mid in missing or st.stamp <= 0.0:
        return None
    return float(st.position_deg)


def _motor_label(mid: int) -> str:
    """Motor id -> joint name (MOTORS[mid].name with the leg prefix stripped)."""
    spec = MOTORS.get(mid)
    if spec is None:
        return str(mid)
    return spec.name.split("_", 1)[1] if "_" in spec.name else spec.name


def _print_motor_map(leg: "SingleLegController") -> None:
    """Print this leg's 'id = joint' mapping."""
    mapping = "  ".join(f"{mid}={_motor_label(mid)}" for mid in leg.motor_ids)
    print(f"  [motor id = joint]  {mapping}")


# --- Hardware zero: set the motor's OWN mechanical zero (MIT 0xFE), persistent ---
# Option 1 sends set_zero (0xFE) so the motor itself stores "current pose = 0"
# (confirmed to survive a power cycle on this firmware). There is no file / no
# software offset any more: after zeroing, raw 0 IS the home pose, so
# JOINT_LIMITS_DEG apply directly in raw space and option 2 just drives every
# motor back to raw 0.
def _step_motor_toward_raw(leg: "SingleLegController", mid: int, target_raw: float, step_deg: float) -> bool:
    """Move motor `mid` one step toward the absolute raw target (shortest way).
    Returns True once reached."""
    leg._request_state([mid], settle_s=0.003)
    with leg._state_lock:
        st = leg.state[mid]
    if st.stamp <= 0.0:
        return False
    cur = float(st.position_deg)
    goal = _wrap_near_center(target_raw, cur)  # shortest-way target from current pos
    if abs(goal - cur) < 1.0:
        return True
    nxt = cur + max(-step_deg, min(step_deg, goal - cur))
    payload = leg._prepare_mit_payload(mid, MotorCommand(position_deg=nxt), clamp=True)
    if payload is not None:
        leg._send8(mid, payload)
    leg._drain_rx_states(timeout_s=0.003, max_msgs=64)
    return False


def _menu_set_zero(leg: "SingleLegController") -> None:
    """Option 1: enter a motor id -> set the motor's OWN mechanical zero here via
    MIT set-zero (0xFE). The motor stores it (persists across power cycle)."""
    _print_motor_map(leg)
    mid = _prompt_int(f"  motor id to zero {list(leg.motor_ids)}: ")
    if mid is None:
        return
    if mid not in leg.motor_ids:
        print(f"  not a motor on this leg ({leg.side}): {list(leg.motor_ids)}")
        return
    leg.disable(mid)          # relax so the motor isn't holding torque while re-zeroing
    time.sleep(0.2)
    before = _read_motor_pos(leg, mid)
    leg.clear_estop()         # allow zeroing even if the pre-zero pose read out of range
    ack = leg.set_zero(mid) is not None   # 0xFE: set current pose as mechanical zero
    time.sleep(0.2)
    after = _read_motor_pos(leg, mid)
    b = f"{before:.2f}" if before is not None else "N/A"
    a = f"{after:.2f}" if after is not None else "N/A"
    print(f"  motor {mid}: mechanical zero set (0xFE) ack={ack}. was raw {b} deg -> now {a} deg")


def _menu_go_home(leg: "SingleLegController", *, step_deg: float = 3.0, timeout_s: float = 15.0) -> None:
    """Option 2: move every motor to its (motor) zero = raw 0."""
    print(f"  moving motors {list(leg.motor_ids)} to zero (0)...")
    leg.clear_estop()
    leg.enable_all()
    time.sleep(0.1)
    deadline = time.time() + timeout_s
    reached = False
    try:
        while time.time() < deadline and not leg._estop:
            done = True
            for mid in leg.motor_ids:
                if not _step_motor_toward_raw(leg, mid, 0.0, step_deg):
                    done = False
            if done:
                reached = True
                print("  done: all motors at zero. (holding)")
                break
            time.sleep(0.02)
    except Exception as e:
        print(f"  [ERROR] {type(e).__name__}: {e} -- disabling motors")
        leg.disable_all()
        return
    if leg._estop:
        print(f"  [ABORT] E-STOP: {leg.get_estop_reason()} -- disabling motors")
        leg.disable_all()
    elif not reached:
        print("  timeout: some motors did not reach zero.")


def _menu_control(leg: "SingleLegController", *, step_deg: float = 2.0) -> None:
    """Option 3: enter a motor id -> sweep its range (JOINT_LIMITS_DEG, referenced to
    the motor zero). Press 'q'+Enter to stop."""
    _print_motor_map(leg)
    mid = _prompt_int(f"  motor id to control {list(leg.motor_ids)}: ")
    if mid is None:
        return
    if mid not in leg.motor_ids:
        print(f"  not a motor on this leg ({leg.side}): {list(leg.motor_ids)}")
        return
    lim = JOINT_LIMITS_DEG.get(mid)
    if lim is None:
        print(f"  motor {mid} has no limits (JOINT_LIMITS_DEG).")
        return
    lo_raw = min(lim) + COMMAND_MARGIN_DEG   # absolute raw (motor zero = 0)
    hi_raw = max(lim) - COMMAND_MARGIN_DEG
    if lo_raw >= hi_raw:
        print(f"  motor {mid} has no valid range (limit={lim}, margin={COMMAND_MARGIN_DEG}).")
        return

    leg.clear_estop()
    leg.enable(mid)
    time.sleep(0.1)

    stop = threading.Event()

    def _wait_q() -> None:
        while not stop.is_set():
            try:
                if input().strip().lower() == "q":
                    stop.set()
                    return
            except EOFError:
                stop.set()
                return

    threading.Thread(target=_wait_q, daemon=True).start()
    print(f"  motor {mid} sweeping range [{lo_raw:.1f}, {hi_raw:.1f}] deg. Press 'q' + Enter to stop.")

    targets = [hi_raw, lo_raw]
    idx = 0
    try:
        while not stop.is_set() and not leg._estop:
            while not stop.is_set() and not leg._estop:
                if _step_motor_toward_raw(leg, mid, targets[idx], step_deg):
                    break
                time.sleep(0.02)
            idx = 1 - idx
    finally:
        stop.set()
        if leg._estop:
            print(f"  [ABORT] E-STOP: {leg.get_estop_reason()}")
        leg.disable(mid)
        print(f"  motor {mid} control stopped and disabled.")


if __name__ == "__main__":
    # Requires robot_constant.py, ankle_kinematics.py, utils/mit_codec.py
    # (one directory up) alongside this file, a real (or virtual) CAN
    # interface on can1, and CALIBRATED=True in robot_constant.py (or
    # allow_uncalibrated=True for a bench/simulator run).
    #
    # Right leg only for now -- left leg isn't connected yet.
    leg = SingleLegController(side="right", allow_uncalibrated=True)

    try:
        while True:
            print(f"\n==== Single-leg control menu (side={leg.side}, motors={list(leg.motor_ids)}) ====")
            print("  1) Set zero      - enter a motor id -> set the motor's mechanical zero here (0xFE)")
            print("  2) Go to zero    - move all motors to their zero (0)")
            print("  3) Control motor - enter a motor id -> sweep its range ('q' to stop)")
            print("  0) Quit")
            choice = input("select> ").strip().lower()
            if choice == "1":
                _menu_set_zero(leg)
            elif choice == "2":
                _menu_go_home(leg)
            elif choice == "3":
                _menu_control(leg)
            elif choice in ("0", "q", "exit"):
                break
            else:
                print("  Choose 1, 2, 3, or 0.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            leg.disable_all()
        except Exception:
            pass
        leg.stop()