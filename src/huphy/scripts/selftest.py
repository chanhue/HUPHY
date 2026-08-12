"""동작 확인 — 다리를 정해진 패턴으로 계속 움직여 봄.

    huphy-test --limb right_leg zero
    huphy-test --limb right_leg range

두 가지임.

    zero     관절 전부를 0도로 두고 붙잡음
    range    관절마다 최소~최대를 오감

둘 다 **Ctrl-Q 를 누를 때까지** 계속함. 끝나면 자세를 붙잡은 채로 토크를 끊음.


## 무엇을 보는 것인가

`zero` 는 **자세가 유지되는가**를 봄. 처지면 게인이 부족하고, 부르르 떨면 과함.
관절 전부가 0도이므로 눈으로 봐도 어긋난 관절이 바로 보임.

`range` 는 **끝까지 가는가**를 봄. 설정한 한계까지 실제로 도달하는지, 도중에
걸리는 데가 없는지, 양 끝에서 부딪히는 소리가 나지 않는지.


## 시작할 때 반드시 천천히 감

지금 자세가 어디든 목표까지 **`--approach` 초에 걸쳐** 옮긴 뒤에 패턴을 시작함.
토크를 넣는 순간 목표가 멀리 있으면 관절이 튐 — 점프 가드가 자르기는 하지만 그
전에 큰 토크가 한 번 나감.


## 한계에서 여유를 둠

`range` 는 설정한 한계에서 `--margin` 만큼 안쪽까지만 감. 한계는 하드스톱을 재서
넣은 값이라 그대로 명령하면 스톱에 부딪힘.
"""

from __future__ import annotations

import argparse
import logging
import math
import select
import sys
import termios
import threading
import tty
from typing import Any, Dict, Optional, Tuple

from ..config import ConfigError, LimbConfig, load_robot
from ..control import ControlLoop, Mode, motions
from ..robots.leg import ANKLE_JOINTS, SINGLE_JOINTS, Leg
from .bringup import build_leg
from . import table
from .commission import CONFIG_NAME, _find_config, _pick_limb

logger = logging.getLogger("huphy.selftest")

QUIT_KEY = "\x11"
"""Ctrl-Q. 터미널이 흘려보내는 문자라 raw 모드에서만 읽힘."""

DEFAULT_APPROACH_S = 3.0
DEFAULT_MARGIN_DEG = 5.0
DEFAULT_PERIOD_S = 6.0


# ===========================================================================
# 관절 한계
# ===========================================================================
def joint_limits(leg: Leg) -> Dict[str, Tuple[float, float]]:
    """관절 이름 -> (최소, 최대). 관절 좌표계임.

    출처가 둘임.

        hip_pitch hip_roll hip_yaw knee     robot.yaml 의 limits_deg. 모터와 관절이 1:1 임
        ankle_pitch ankle_roll  AnkleEnvelope. 모터 한계가 아니라 시험 범위임

    발목은 모터 두 개가 로드로 물려 있어 **모터 한계를 관절 한계로 옮길 수 없음.**
    한 모터의 최대각이 다른 모터의 자세에 따라 달라짐. 그래서 발목만 기구학 쪽의
    시험 범위를 씀.

    한계가 없는 관절은 빠짐 -- 아직 안 잰 것이라 어디까지 가도 되는지 모름.
    """
    out: Dict[str, Tuple[float, float]] = {}
    for name in SINGLE_JOINTS:
        limits = leg.config.motors[name].limits_deg
        if limits is not None:
            out[name] = (float(limits[0]), float(limits[1]))

    envelope = leg.kinematics.envelope
    out["ankle_pitch"] = tuple(float(v) for v in envelope.pitch_deg)
    out["ankle_roll"] = tuple(float(v) for v in envelope.roll_deg)
    return out


def _inset(limits: Tuple[float, float], margin: float) -> Tuple[float, float]:
    """한계에서 안쪽으로 `margin` 만큼 들어옴.

    폭이 여유의 두 배도 안 되면 가운데로 접음 -- 좁은 관절에서 최소가 최대보다
    커지는 것을 막음.
    """
    lo, hi = limits
    if hi - lo <= 2.0 * margin:
        middle = 0.5 * (lo + hi)
        return (middle, middle)
    return (lo + margin, hi - margin)


# ===========================================================================
# 동작
# ===========================================================================
def approach(targets: Dict[str, float], start: Dict[str, float], seconds: float):
    """지금 자세에서 목표까지 관절 전부를 같이 옮김.

    `motions.ramp` 은 관절 하나짜리라 여기서 여러 관절을 함께 다룸. 시작값은
    루프를 돌리기 전에 읽어 둔 관찰임 -- 매 주기 다시 읽으면 목표가 따라 움직여
    영영 도착하지 않음.
    """
    if seconds <= 0:
        raise ValueError(f"seconds 는 0보다 커야 함 (받은 값 {seconds})")

    def motion(t: float, observation: Dict[str, Any]) -> Dict[str, float]:
        fraction = min(1.0, max(0.0, t / seconds))
        return {
            name: start.get(name, target) + (target - start.get(name, target)) * fraction
            for name, target in targets.items()
        }

    return motion


def cycle(limits: Dict[str, Tuple[float, float]], *, period_s: float):
    """관절마다 최소~최대를 오감. 전부 같은 위상으로 움직임.

    사인파를 씀 -- 삼각파는 양 끝에서 속도가 꺾이면서 점프 가드에 걸리고, 실제로도
    관절이 한 번 튐.

    시작이 가운데라 `approach` 가 데려다 놓은 자리에서 이어짐.
    """
    plan = {
        name: (0.5 * (lo + hi), 0.5 * (hi - lo)) for name, (lo, hi) in limits.items()
    }
    omega = 2.0 * math.pi / float(period_s)

    def motion(t: float, observation: Dict[str, Any]) -> Dict[str, float]:
        return {
            name: center + amplitude * math.sin(omega * t)
            for name, (center, amplitude) in plan.items()
        }

    return motion


def midpoints(limits: Dict[str, Tuple[float, float]]) -> Dict[str, float]:
    return {name: 0.5 * (lo + hi) for name, (lo, hi) in limits.items()}


def then(first, seconds: float, second):
    """`first` 를 `seconds` 만큼 돌린 뒤 `second` 로 넘어감. 뒤는 끝이 없음.

    `motions.chain` 은 구간마다 길이를 받고 마지막이 끝나면 명령을 멈춤. 여기는
    Ctrl-Q 까지 계속해야 해서 뒤 구간에 길이를 두지 않음.

    `second` 는 **자기 구간의 0초부터** 시작하는 시간을 받음 -- 사인파가 가운데에서
    시작해야 접근이 데려다 놓은 자리와 이어짐.
    """

    def motion(t: float, observation: Dict[str, Any]):
        if t < seconds:
            return first(t, observation)
        return second(t - seconds, observation)

    return motion


# ===========================================================================
# Ctrl-Q
# ===========================================================================
class QuitWatcher:
    """Ctrl-Q 가 눌리면 루프를 멈춤.

    별도 스레드에서 키를 봄 -- 루프는 주기를 지켜야 해서 입력을 기다릴 틈이 없음.
    `loop.stop()` 은 다음 주기에 빠져나오게만 하므로 스레드에서 불러도 됨.

    **터미널을 raw 모드로 바꿈.** Ctrl-Q 는 원래 흐름 제어(XON)로 먹히는 문자라,
    보통 모드에서는 프로그램까지 오지 않음. 빠져나올 때 원래 설정으로 되돌림.

    화면이 아니면 아무것도 하지 않음 -- 그때는 Ctrl-C 로 끊음.
    """

    def __init__(self, loop: ControlLoop) -> None:
        self.loop = loop
        self.armed = sys.stdin.isatty()
        self._saved = None
        self._thread: Optional[threading.Thread] = None
        self._done = threading.Event()

    def __enter__(self) -> "QuitWatcher":
        if not self.armed:
            return self
        self._saved = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        self._thread = threading.Thread(target=self._watch, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *exc) -> None:
        self._done.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._saved is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._saved)
        return None

    def _watch(self) -> None:
        while not self._done.is_set():
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue
            key = sys.stdin.read(1)
            if key == QUIT_KEY:
                logger.info("Ctrl-Q -- 멈춤")
                self.loop.stop()
                return


# ===========================================================================
# 명령
# ===========================================================================
def _read_pose(leg: Leg, loop: ControlLoop, joints) -> Dict[str, float]:
    """지금 관절 각도를 읽음. 루프를 한 주기 돌려 상태를 받아 옴.

    MIT 모드에는 읽기 전용 명령이 없어서, 힘이 나가지 않는 명령을 보내고 그 응답을
    받는 것임.

    관찰은 **모터 공간**임 (`knee.pos`). 발목만 관절 각도가 관찰에 없어서 FK 를
    따로 부름 -- 모터가 보고하는 값이 아니라 FK 로 푸는 값임.
    """
    loop.step(None, t=0.0)
    observation = leg.get_observation()

    out: Dict[str, float] = {}
    for name in joints:
        if name in SINGLE_JOINTS:
            value = observation.get(f"{name}.pos")
            if value is not None:
                out[name] = float(value)

    if any(name in ANKLE_JOINTS for name in joints):
        pose = leg.ankle_pose()
        if pose is not None:
            for name, value in zip(ANKLE_JOINTS, pose):
                if name in joints:
                    out[name] = float(value)
    return out


def _run(leg: Leg, loop: ControlLoop, targets, motion, *, approach_s: float) -> int:
    """접근 -> 패턴. Ctrl-Q 까지 돎."""
    start = _read_pose(leg, loop, targets)
    missing = [name for name in targets if name not in start]
    if missing:
        print(f"  상태를 못 읽은 관절: {missing}. 배선과 CAN id 를 확인할 것.")
        return 1

    print(f"\n  {approach_s:.0f}초에 걸쳐 시작 자세로 옮긴 뒤 시작합니다.")
    print(f"  멈추려면 Ctrl-Q.\n")

    loop.mode = Mode.CONTROL
    plan = then(approach(targets, start, approach_s), approach_s, motion)

    with QuitWatcher(loop):
        stats = loop.run(plan)

    print(
        f"\n  {stats.cycles}주기 {stats.total_s:.1f}초, 평균 {stats.mean_hz:.1f}Hz "
        f"(목표 {stats.target_hz:.0f}Hz)"
    )
    if stats.missing_cycles:
        print(f"  응답이 빠진 주기: {stats.missing_cycles}")
    if not stats.kept_up:
        print("  주기를 못 지킴. 게인을 만지기 전에 이것부터 볼 것.")
        return 1
    return 0


def cmd_zero(args, leg: Leg, loop: ControlLoop) -> int:
    """관절 전부를 0도로 두고 붙잡음."""
    joints = list(SINGLE_JOINTS) + list(ANKLE_JOINTS)
    targets = {name: 0.0 for name in joints}

    print(f"\n{leg.id} 영자세 유지 -- {len(joints)}개 관절을 0도로")
    print("  관절 전부가 0도이므로 어긋난 관절이 눈으로 보임.")
    print("  처지면 kp 가 부족하고, 부르르 떨면 kp 가 과함.")

    return _run(leg, loop, targets, motions.hold(targets), approach_s=args.approach)


def cmd_range(args, leg: Leg, loop: ControlLoop) -> int:
    """관절마다 최소~최대를 오감."""
    limits = joint_limits(leg)
    inset = {name: _inset(span, args.margin) for name, span in limits.items()}

    print(f"\n{leg.id} 가동 범위 왕복 -- 주기 {args.period:.1f}초")
    print(f"  한계에서 {args.margin:.1f}도 안쪽까지만 감.\n")
    print("  " + table.header(("관절", 12, "<"), ("최소", 9), ("최대", 9)))
    for name, (lo, hi) in inset.items():
        print(f"  {name:<12} {lo:9.2f} {hi:9.2f}")

    skipped = [n for n in SINGLE_JOINTS if n not in limits]
    if skipped:
        print(
            f"\n  한계가 없어 빼는 관절: {skipped}.\n"
            f"  huphy-commission --limb {leg.id} sweep 으로 먼저 잴 것."
        )

    flat = [name for name, (lo, hi) in inset.items() if hi - lo < 1.0]
    if flat:
        print(f"\n  움직일 폭이 없는 관절: {flat}. 한계가 좁거나 여유가 큼.")

    return _run(
        leg,
        loop,
        midpoints(inset),
        cycle(inset, period_s=args.period),
        approach_s=args.approach,
    )


COMMANDS = {"zero": cmd_zero, "range": cmd_range}


# ===========================================================================
# 진입점
# ===========================================================================
def _add_common(parser, *, suppress: bool) -> None:
    """어느 자리에서도 받는 옵션. 최상위와 서브명령 양쪽에 붙임.

    argparse 는 최상위 플래그를 서브명령 뒤에 받지 않음. `zero --approach 5` 라고
    쓰는 것이 자연스러우므로 서브명령에도 같은 옵션을 둠.

    서브명령 쪽은 기본값이 `SUPPRESS` 임. 그러지 않으면 `--limb r zero` 처럼 앞에
    적은 값을 **서브명령의 기본값이 덮어씀** -- 안 준 옵션은 이름공간에 아예 넣지
    않아야 앞의 값이 살아남음.
    """
    default = (lambda v: argparse.SUPPRESS if suppress else v)
    parser.add_argument(
        "--config",
        default=default(None),
        help=f"기본값: 위로 올라가며 {CONFIG_NAME} 을 찾음",
    )
    parser.add_argument(
        "--limb", default=default(None), help="팔다리 이름. 하나뿐이면 생략 가능"
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=default(None),
        help="제어 주기. 기본은 설정의 control_hz",
    )
    parser.add_argument(
        "--gain-scale",
        type=float,
        default=default(1.0),
        help="게인을 낮춰 시작함. 기본 1.0",
    )
    parser.add_argument(
        "--approach",
        type=float,
        default=default(DEFAULT_APPROACH_S),
        help=f"시작 자세까지 옮기는 시간. 기본 {DEFAULT_APPROACH_S:.0f}초",
    )
    parser.add_argument(
        "--allow-uncalibrated",
        action="store_true",
        default=default(False),
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", default=default(False)
    )


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="huphy-test",
        description="다리를 정해진 패턴으로 계속 움직여 봄. Ctrl-Q 로 멈춤.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "예시:\n"
            "  --limb right_leg zero\n"
            "  --limb right_leg range --period 10 --margin 8\n"
        ),
    )
    _add_common(p, suppress=False)

    sub = p.add_subparsers(dest="command", required=True)

    z = sub.add_parser("zero", help="관절 전부를 0도로 두고 붙잡음")
    _add_common(z, suppress=True)

    r = sub.add_parser("range", help="관절마다 최소~최대를 오감")
    _add_common(r, suppress=True)
    r.add_argument(
        "--period",
        type=float,
        default=DEFAULT_PERIOD_S,
        help=f"한 번 왕복하는 데 걸리는 시간. 기본 {DEFAULT_PERIOD_S:.0f}초",
    )
    r.add_argument(
        "--margin",
        type=float,
        default=DEFAULT_MARGIN_DEG,
        help=f"한계에서 남길 여유. 기본 {DEFAULT_MARGIN_DEG:.0f}도",
    )
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(message)s",
    )

    from pathlib import Path

    path = Path(args.config) if args.config else _find_config()
    if path is None or not path.is_file():
        raise SystemExit(f"설정 파일이 없음: {path or CONFIG_NAME}")
    try:
        robot = load_robot(path)
    except ConfigError as e:
        raise SystemExit(str(e)) from None

    limb: LimbConfig = _pick_limb(robot, args.limb)
    leg = build_leg(
        robot,
        limb,
        gain_scale=args.gain_scale,
        allow_uncalibrated=args.allow_uncalibrated,
    )
    loop = ControlLoop(
        leg,
        hz=args.hz if args.hz else limb.control_hz,
        mode=Mode.OBSERVE,
    )

    try:
        with leg:
            return COMMANDS[args.command](args, leg, loop)
    except ConnectionError as e:
        raise SystemExit(
            f"{e}\n채널이 올라와 있는지 확인할 것:\n"
            f"  sudo ip link set {limb.channel} up type can bitrate 1000000"
        ) from e
    except KeyboardInterrupt:
        print()
        return 130


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
