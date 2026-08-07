"""커미셔닝 진입점 — 조립할 때 한 번 하는 조작을 터미널에서 부름.

    python -m huphy.scripts.commission --limb right_leg scan
    python -m huphy.scripts.commission --limb right_leg nudge knee
    python -m huphy.scripts.commission --limb right_leg zero knee --note "다리 편 상태"

설정 파일에서 모터 목록을 읽으므로 모터 id 를 손으로 적지 않음.


## 관절 이름으로 말함

    nudge knee          O
    nudge 10            X

사람은 관절로 생각하고, 모터 id 는 배선이 바뀌면 달라짐. 설정에 적힌 이름을 그대로 씀.


## 어느 팔다리인지 반드시 지정함

`--limb` 이 없으면 멈춤 (팔다리가 하나뿐이면 그것을 씀). 다리가 둘이고 각각 다른
CAN 채널에 있으므로, 잘못 고르면 **엉뚱한 다리가 움직임.**


## 되돌리기 어려운 것은 --yes 를 요구함

    zero        기계 영점. 전원 재투입 후에도 남음
    can-id      CAN id 변경
    protocol    프로토콜 전환. 전원 재투입 필요

실수로 방향키를 눌러 실행되는 자리에 두지 않으려는 것임.
"""

from __future__ import annotations

import argparse
import logging
import select
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

from .. import calibration as cal
from ..config import ConfigError, LimbConfig, load_robot
from ..config.schema import RobotConfig
from ..motors.base import MotorCalibration
from ..motors.canbus import CanBus
from ..motors.robstride import commissioning as C
from ..motors.robstride import tables
from ..motors.robstride.bus import RobStrideBus

CONFIG_NAME = "config/robot.yaml"

DANGEROUS = {
    "zero": "기계 영점을 지금 자세로 잡음. 모터에 저장되고 좌표계가 통째로 옮겨감.",
    "can-id": "CAN id 를 바꿈. 바꾼 뒤 robot.yaml 도 고쳐야 함.",
    "protocol": "프로토콜을 바꿈. 전원을 재투입해야 적용되고 그 뒤에는 프레임 포맷이 달라짐.",
}
"""되돌리기 어려운 조작. `--yes` 없이는 버스를 열지도 않음.

`--yes` 를 최상위가 아니라 이 명령들에만 두는 이유: argparse 는 최상위 플래그를
서브명령 뒤에 받지 않음. `zero knee --yes` 라고 쓰는 것이 자연스러운데, 최상위에
두면 `--yes zero knee` 로 써야 함.
"""

YES_HELP = "되돌리기 어려운 조작임을 확인함"


def _find_config() -> Optional[Path]:
    """현재 폴더부터 위로 올라가며 `config/robot.yaml` 을 찾음.

    저장소 어디서 실행하든 같은 파일을 쓰게 하려는 것임.
    """
    here = Path.cwd().resolve()
    for folder in (here, *here.parents):
        candidate = folder / CONFIG_NAME
        if candidate.is_file():
            return candidate
    return None


def _pick_limb(robot: RobotConfig, name: Optional[str]) -> LimbConfig:
    if name:
        try:
            return robot.limb(name)
        except KeyError as e:
            # 사람이 이름을 잘못 친 것이라 스택 트레이스가 도움이 되지 않음.
            raise SystemExit(str(e).strip('"')) from None
    if len(robot.limbs) == 1:
        return next(iter(robot.limbs.values()))
    raise SystemExit(
        f"--limb 을 지정할 것 (가용: {sorted(robot.limbs)}). "
        f"팔다리마다 CAN 채널이 달라 잘못 고르면 엉뚱한 쪽이 움직임"
    )


def _joint_id(limb: LimbConfig, joint: str) -> int:
    if joint not in limb.motors:
        raise SystemExit(
            f"{limb.name} 에 {joint!r} 관절이 없음 (가용: {sorted(limb.motors)})"
        )
    return limb.motors[joint].id


def choose_joints(
    limb: LimbConfig,
    joint: Optional[str],
    *,
    allow_all: bool,
    what: str,
) -> List[str]:
    """무엇을 대상으로 할지 정함. 인자가 없으면 **물어봄.**

    관절 이름을 외우지 않아도 되게 하려는 것임. 목록을 보여주고 번호나 이름을 받음.

    화면이 아니면(파이프, 스크립트) 묻지 않음 -- 입력이 없는 곳에서 멈추면 안 됨.
    그때는 `allow_all` 이면 전부, 아니면 에러임.
    """
    if joint:
        _joint_id(limb, joint)
        return [joint]

    names = list(limb.motors)

    if not sys.stdin.isatty():
        if allow_all:
            return names
        raise SystemExit(
            f"관절을 지정할 것 (가용: {names}). 화면이 아니라 물어볼 수 없음"
        )

    print(f"\n  {limb.name} -- 무엇을 {what}?\n")
    for index, name in enumerate(names, start=1):
        motor = limb.motors[name]
        print(f"    {index}) {name:<10} id={motor.id:<3} {motor.model}")
    if allow_all:
        print(f"    a) 전부")
    print()

    default = "a" if allow_all else ""
    raw = input(f"  선택{' [a]' if allow_all else ''}: ").strip().lower() or default

    if allow_all and raw in ("a", "all", "전부"):
        return names
    if raw.isdigit() and 1 <= int(raw) <= len(names):
        return [names[int(raw) - 1]]
    if raw in names:
        return [raw]

    raise SystemExit(f"{raw!r} 는 고를 수 없음 (가용: {names})")


# ===========================================================================
# 옵션 고르기
# ===========================================================================
@dataclass(frozen=True)
class Option:
    """대화형으로 받을 수 있는 옵션 하나.

    `default` 가 `None` 이면 **반드시 받아야 하는 값**임 -- 대신 정해 줄 수 없는 것들
    (어느 자세에서 영점을 잡았는지, 어느 번호로 바꿀지) 이 여기에 해당함.
    """

    name: str
    default: Any
    parse: Callable[[str], Any]
    note: str
    choices: Tuple[str, ...] = ()

    def shown(self) -> str:
        return "(필수)" if self.default is None else str(self.default)


OPTIONS: Dict[str, Tuple[Option, ...]] = {
    "nudge": (
        Option("delta", 5.0, float, "몇 도 움직였다 되돌릴지. 20도까지"),
        Option("kp", 5.0, float, "위치 게인. 안 움직이면 조금씩 올릴 것"),
        Option("kd", 0.5, float, "속도 게인"),
    ),
    "sweep": (Option("hz", 20.0, float, "몇 번 재는지. 초당"),),
    "zero": (Option("note", None, str, "어느 자세에서 잡는지. 나중에 재현하려면 필요함"),),
    "mode": (
        Option(
            "to",
            "mit",
            str,
            "제어 모드",
            tuple(x.name.lower() for x in tables.ControlMode),
        ),
    ),
    "can-id": (Option("to", None, int, "바꿀 CAN id. 1..127"),),
    "protocol": (
        Option(
            "to",
            None,
            str,
            "프레임 포맷. 바꾸면 전원 재투입이 필요함",
            tuple(x.name.lower() for x in tables.Protocol),
        ),
    ),
}
"""명령별 옵션 목록. 표시 순서가 곧 쉼표 순서임.

여기 적힌 것이 **기본값의 유일한 출처**임. argparse 쪽 기본값은 `None` 으로 두어
"안 줬음" 과 "기본값을 줬음" 을 구분함 -- 안 준 것만 물어보기 위함임.
"""


def choose_options(command: str, args, *, asked: bool) -> None:
    """빠진 옵션을 채움. 대화형이면 **한 줄로 몰아서** 받음.

    관절을 명령줄에 적었으면(`asked` 가 거짓) 플래그로 다 지정한 것으로 보고 묻지
    않음. 관절을 생략해 목록에서 고른 경우에만, 이어서 옵션도 한 번에 보여주고 받음.

    입력은 쉼표로 나눔. 빈 칸은 기본값임. 옵션이 하나뿐인 명령(`zero --note` 등)은
    쉼표로 나누지 않고 줄 전체를 값으로 씀 -- 메모에 쉼표가 들어가기 때문임.
    """
    options = OPTIONS.get(command, ())
    if not options:
        return

    missing = [o for o in options if getattr(args, o.name, None) is None]

    if missing and asked and sys.stdin.isatty():
        print(f"\n  옵션 -- 쉼표로 구분, 비우면 기본값\n")
        for index, option in enumerate(options, start=1):
            tail = f"  {'/'.join(option.choices)}" if option.choices else ""
            print(f"    {index}) {option.name:<7} {option.shown():<8} {option.note}{tail}")
        print()

        line = ", ".join(o.shown() for o in options)
        raw = input(f"  입력 [{line}]: ").strip()
        if raw:
            _apply_options(options, args, raw)

    for option in options:
        if getattr(args, option.name, None) is not None:
            continue
        if option.default is None:
            raise SystemExit(
                f"--{option.name} 을 지정할 것 ({option.note})"
            )
        setattr(args, option.name, option.default)

    for option in options:
        value = getattr(args, option.name)
        if option.choices and str(value).lower() not in option.choices:
            raise SystemExit(
                f"--{option.name} 은 {list(option.choices)} 중 하나여야 함 (받은 값: {value!r})"
            )


def _apply_options(options: Tuple[Option, ...], args, raw: str) -> None:
    """한 줄로 받은 답을 인자에 옮김."""
    if len(options) == 1:
        tokens = [raw]
    else:
        tokens = [t.strip() for t in raw.split(",")]
        if len(tokens) > len(options):
            raise SystemExit(
                f"옵션은 {len(options)}개인데 {len(tokens)}개를 받음: {raw!r}"
            )

    for token, option in zip(tokens, options):
        if not token:
            continue
        try:
            setattr(args, option.name, option.parse(token))
        except ValueError as e:
            raise SystemExit(f"{option.name} 값을 읽지 못함: {token!r} ({e})") from e


def echo_command(limb: LimbConfig, command: str, joint: str, args) -> None:
    """방금 고른 것을 명령줄 형태로 냄. 다음부터는 이대로 바로 칠 수 있음."""
    head = f"huphy-commission --limb {limb.name} {command}"
    parts = [f"{head} {joint}" if joint else head]
    for option in OPTIONS.get(command, ()):
        value = getattr(args, option.name)
        text = f'"{value}"' if isinstance(value, str) and " " in value else value
        parts.append(f"--{option.name} {text}")
    if getattr(args, "yes", False):
        parts.append("--yes")
    print(f"\n  실행: {' '.join(str(p) for p in parts)}\n")


def _open(limb: LimbConfig) -> RobStrideBus:
    bus = RobStrideBus(
        CanBus(limb.channel, interface=limb.interface), limb.motors_by_id()
    )
    try:
        bus.connect()
    except ImportError as e:
        raise SystemExit(f"{e}") from e
    except ConnectionError as e:
        raise SystemExit(
            f"{e}\n채널이 올라와 있는지 확인할 것:\n"
            f"  sudo ip link set {limb.channel} up type can bitrate 1000000\n"
            f"  ip -details link show {limb.channel}"
        ) from e
    return bus


# ===========================================================================
# 명령
# ===========================================================================
def cmd_scan(args, limb: LimbConfig, bus: RobStrideBus) -> int:
    found = C.scan(bus)
    print(f"{limb.name}  {limb.channel}  모터 {len(limb.motors)}개\n")
    for joint, motor in limb.motors.items():
        mark = "응답" if motor.id in found else "----"
        print(f"  {joint:10} id={motor.id:<3} {motor.model:6} {mark}")

    missing = [j for j, m in limb.motors.items() if m.id not in found]
    if missing:
        print(
            f"\n응답 없음: {missing}\n"
            f"  배선, 전원, CAN id, 프로토콜 모드가 후보임.\n"
            f"  이 넷은 여기서 구분되지 않음 -- 전부 조용히 빠짐 (이슈 #11)."
        )
        return 1
    print("\n전부 응답함.")
    return 0


def cmd_state(args, limb: LimbConfig, bus: RobStrideBus) -> int:
    missing = bus.refresh_states()
    calib = None
    if limb.calibration_path and limb.calibration_path.is_file():
        calib = cal.attach(cal.load(limb.calibration_path), limb.motors)

    print(f"{limb.name}  {limb.channel}\n")
    print(f"  {'관절':<10} {'raw':>9} {'cal':>9} {'속도':>9} {'토크':>8} {'온도':>6}")
    for joint, motor in limb.motors.items():
        st = bus.state(motor.id)
        if not st.is_valid:
            print(f"  {joint:<10} {'응답 없음':>9}")
            continue
        cal_deg = calib[motor.id].raw_to_cal(st.position_deg) if calib else st.position_deg
        print(
            f"  {joint:<10} {st.position_deg:9.2f} {cal_deg:9.2f} "
            f"{st.velocity_deg_s:9.2f} {st.torque_nm:8.2f} {st.temp_c:6.1f}"
        )

    if calib is None:
        print("\n  캘리브레이션 파일이 없어 cal 이 raw 와 같음.")
    elif cal.unmeasured(cal.load(limb.calibration_path)):
        print("\n  미실측 상태라 cal 이 raw 와 같음.")
    return 1 if missing else 0


def cmd_fault(args, limb: LimbConfig, bus: RobStrideBus) -> int:
    print(f"{limb.name}  {limb.channel}\n")
    bad = 0
    for joint, motor in limb.motors.items():
        fault = bus.read_fault(motor.id)
        if fault is None:
            print(f"  {joint:10} 응답 없음")
            bad += 1
        elif fault.ok:
            print(f"  {joint:10} 정상")
        else:
            print(f"  {joint:10} 0x{fault.raw:08X}  {', '.join(fault.active())}")
            bad += 1
    return 1 if bad else 0


def cmd_clear_fault(args, limb: LimbConfig, bus: RobStrideBus) -> int:
    joints = choose_joints(limb, args.joint, allow_all=True, what="지울까요")
    bus.clear_fault([limb.motors[j].id for j in joints])
    print(f"\n  고장 상태를 지웠음: {', '.join(joints)}")
    print("  원인이 남아 있으면 다시 뜸.")
    return 0


def _enter_pressed() -> bool:
    """Enter 가 눌렸는지. **기다리지 않음.**

    입력을 기다리면 그동안 상태를 못 읽어 최대·최소를 놓침.
    """
    if not sys.stdin.isatty():
        return False
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return False
    sys.stdin.readline()
    return True


def cmd_sweep(args, limb: LimbConfig, bus: RobStrideBus) -> int:
    """토크를 끄고 손으로 미는 동안 가동 범위를 기록함."""
    asked = args.joint is None
    joints = choose_joints(limb, args.joint, allow_all=True, what="잴까요")
    choose_options(args.command, args, asked=asked)
    if asked:
        echo_command(limb, args.command, args.joint or "", args)
    ids = [limb.motors[j].id for j in joints]
    names = {motor.id: name for name, motor in limb.motors.items()}

    print(
        f"\n{limb.name} 가동 범위 측정\n\n"
        f"  토크를 끕니다. 관절을 손으로 **양쪽 끝까지** 천천히 미세요.\n"
        f"  하드스톱에 닿는 느낌을 확인할 것 -- 끝까지 안 밀면 그만큼 좁게 나옵니다.\n"
        f"  발목은 발을 잡고 움직이면 두 모터가 같이 따라옵니다.\n\n"
        f"  끝나면 Enter.\n"
    )

    lines = [0]

    def show(results, positions):
        if lines[0]:
            print(f"\033[{lines[0]}A", end="")
        out = [f"  {'관절':<10} {'최소':>9} {'지금':>9} {'최대':>9} {'범위':>9}"]
        for mid, r in results.items():
            now = positions.get(mid)
            now_text = f"{now:9.2f}" if now is not None else f"{'--':>9}"
            out.append(
                f"  {names[mid]:<10} {r.lo_deg:9.2f} {now_text} "
                f"{r.hi_deg:9.2f} {r.span_deg:9.2f}"
            )
        print("\n".join(out))
        lines[0] = len(out)

    results = C.sweep(
        bus, ids, should_stop=lambda: _enter_pressed(), on_update=show, hz=args.hz
    )

    print(f"\n  robot.yaml 의 {limb.name}.motors 에 적을 것 (raw 공간):\n")
    for mid, r in results.items():
        motor = limb.motors[names[mid]]
        print(
            f"      {names[mid]+':':<10} {{id: {motor.id}, model: {motor.model}, "
            f"limits_deg: [{r.lo_deg:.2f}, {r.hi_deg:.2f}], "
            f"kp: {motor.gains.kp}, kd: {motor.gains.kd}}}"
        )

    print(
        f"\n  한 번 더 돌려 같은 값이 나오는지 확인할 것.\n"
        f"  영점을 다시 잡으면 이 값도 다시 재야 함 -- raw 는 영점에 매달려 있음."
    )

    thin = [names[mid] for mid, r in results.items() if r.span_deg < 5.0]
    if thin:
        print(f"\n  범위가 5도도 안 되는 관절: {thin}. 끝까지 밀었는지 확인할 것.")
        return 1
    return 0


def cmd_nudge(args, limb: LimbConfig, bus: RobStrideBus) -> int:
    asked = args.joint is None
    joint = choose_joints(limb, args.joint, allow_all=False, what="움직일까요")[0]
    choose_options(args.command, args, asked=asked)
    args.joint = joint
    motor_id = limb.motors[joint].id
    if asked:
        echo_command(limb, args.command, joint, args)
    print(
        f"{limb.name}.{args.joint} (id={motor_id}) 를 {args.delta:+.1f}도 움직였다 되돌림.\n"
        f"  게인 kp={args.kp} kd={args.kd}\n"
        f"  다리를 받쳐 두고, 실제로 어느 관절이 움직이는지 볼 것.\n"
    )
    result = C.nudge(
        bus, motor_id, delta_deg=args.delta, kp=args.kp, kd=args.kd
    )
    print(
        f"  시작 {result.start_deg:8.2f}\n"
        f"  최대 {result.peak_deg:8.2f}   (움직인 양 {result.moved_deg:+.2f})\n"
        f"  끝   {result.end_deg:8.2f}"
    )
    if abs(result.moved_deg) < abs(args.delta) * 0.3:
        print(
            f"\n  명령한 만큼 안 움직였음. 게인이 낮거나, 중력을 못 이기거나, 걸린 것임.\n"
            f"  받침대에 올린 상태에서 --kp 를 조금씩 올려볼 것."
        )
    return 0


def cmd_zero(args, limb: LimbConfig, bus: RobStrideBus) -> int:
    """지금 자세를 기계 영점으로 잡음.

    관절 이름을 생략하면 **전부** 잡음. 토크가 꺼진 상태로 자세를 유지해야 하는데,
    명령을 여섯 번 나눠 치면 그동안 다리가 무너짐.
    """
    asked = args.joint is None
    joints = choose_joints(limb, args.joint, allow_all=True, what="영점 잡을까요")
    choose_options(args.command, args, asked=asked)
    if asked:
        echo_command(limb, args.command, args.joint or "", args)

    print(f"\n{limb.name} 영점: {', '.join(joints)}")
    print(f'  자세: "{args.note}"\n')

    bus.disable_torque([limb.motors[j].id for j in joints])

    done = []
    failed = []
    for joint in joints:
        try:
            C.set_zero(bus, limb.motors[joint].id, zero_reference=args.note)
        except C.CommissioningError as e:
            failed.append(joint)
            print(f"  {joint:10} 실패 -- {e}")
        else:
            done.append(joint)
            print(f"  {joint:10} 잡음")

    if done and limb.calibration_path:
        path = limb.calibration_path
        entries = cal.load(path) if path.is_file() else cal.identity(limb.motors)
        for joint in done:
            previous = entries.get(joint, MotorCalibration(motor_id=-1))
            entries[joint] = MotorCalibration(
                motor_id=-1,
                sign=previous.sign,
                offset_deg=previous.offset_deg,
                zero_reference=args.note,
            )
        cal.save(path, entries, limb=limb.name)
        print(f"\n  메모를 {path} 에 저장했음 ({len(done)}개).")
    elif done:
        print("\n  캘리브레이션 파일이 설정되어 있지 않아 메모를 저장하지 못함.")

    if failed:
        print(f"\n  실패한 관절: {failed}. 배선과 CAN id 를 확인하고 다시 할 것.")
        return 1

    print(
        f"\n  다음: 자세를 그대로 두고 가동 범위를 잼.\n"
        f"    huphy-commission --limb {limb.name} sweep"
    )
    return 0


def cmd_mode(args, limb: LimbConfig, bus: RobStrideBus) -> int:
    asked = args.joint is None
    args.joint = choose_joints(limb, args.joint, allow_all=False, what="바꿀까요")[0]
    choose_options(args.command, args, asked=asked)
    if asked:
        echo_command(limb, args.command, args.joint, args)
    mode = tables.ControlMode[args.to.upper()]
    motor_id = limb.motors[args.joint].id
    C.set_control_mode(bus, motor_id, mode)
    print(f"{limb.name}.{args.joint} 제어 모드 -> {mode.name}")
    return 0


def cmd_can_id(args, limb: LimbConfig, bus: RobStrideBus) -> int:
    asked = args.joint is None
    args.joint = choose_joints(limb, args.joint, allow_all=False, what="바꿀까요")[0]
    choose_options(args.command, args, asked=asked)
    if asked:
        echo_command(limb, args.command, args.joint, args)
    motor_id = limb.motors[args.joint].id
    C.set_can_id(bus, motor_id, args.to)
    print(
        f"{limb.name}.{args.joint}: {motor_id} -> {args.to}\n"
        f"  robot.yaml 의 {limb.name}.motors.{args.joint}.id 를 {args.to} 로 고칠 것."
    )
    return 0


def cmd_protocol(args, limb: LimbConfig, bus: RobStrideBus) -> int:
    asked = args.joint is None
    args.joint = choose_joints(limb, args.joint, allow_all=False, what="바꿀까요")[0]
    choose_options(args.command, args, asked=asked)
    if asked:
        echo_command(limb, args.command, args.joint, args)
    protocol = tables.Protocol[args.to.upper()]
    motor_id = limb.motors[args.joint].id
    C.set_protocol(bus, motor_id, protocol)
    print(
        f"{limb.name}.{args.joint} 프로토콜 -> {protocol.name}\n"
        f"  전원을 재투입할 것. 그 전까지는 옛 포맷으로 통신함."
    )
    return 0


# ===========================================================================
# 진입점
# ===========================================================================
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="python -m huphy.scripts.commission",
        description="조립할 때 한 번 하는 조작. 관절 이름으로 지정함.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "예시:\n"
            "  --limb right_leg scan\n"
            "  --limb right_leg state\n"
            '  --limb right_leg zero --note "다리 편 상태" --yes\n'
            "  --limb right_leg sweep\n"
            "  --limb right_leg nudge knee --delta 5\n"
            '  --limb right_leg zero knee --note "다리 편 상태" --yes\n'
        ),
    )
    p.add_argument("--config", type=Path, help=f"기본값: 위로 올라가며 {CONFIG_NAME} 을 찾음")
    p.add_argument("--limb", help="팔다리 이름. 하나뿐이면 생략 가능")
    p.add_argument("-v", "--verbose", action="store_true")

    sub = p.add_subparsers(dest="command", required=True)

    sub.add_parser("scan", help="어느 모터가 응답하는지")
    sub.add_parser("state", help="현재 각도·속도·토크·온도")
    sub.add_parser("fault", help="고장 상태 조회")

    c = sub.add_parser("clear-fault", help="고장 상태 지우기")
    c.add_argument("joint", nargs="?", help="생략하면 물어봄 (기본 전부)")

    s = sub.add_parser("sweep", help="토크를 끄고 손으로 밀어 가동 범위 측정")
    s.add_argument("joint", nargs="?", help="생략하면 물어봄 (기본 전부)")
    s.add_argument("--hz", type=float, help="기본 20")

    n = sub.add_parser("nudge", help="조금 움직였다 되돌림. 어느 관절인지 확인용")
    n.add_argument("joint", nargs="?", help="생략하면 물어봄")
    n.add_argument("--delta", type=float, help="기본 5도, 최대 20도")
    n.add_argument("--kp", type=float, help="기본 5.0")
    n.add_argument("--kd", type=float, help="기본 0.5")

    z = sub.add_parser("zero", help="[영구] 지금 자세를 기계 영점으로")
    z.add_argument("joint", nargs="?", help="생략하면 물어봄 (기본 전부)")
    z.add_argument("--note", help="어느 자세에서 잡는지. 나중에 재현하려면 필요함")
    z.add_argument("--yes", action="store_true", help=YES_HELP)

    m = sub.add_parser("mode", help="제어 모드 변경")
    m.add_argument("joint", nargs="?", help="생략하면 물어봄")
    m.add_argument("--to", choices=[x.name.lower() for x in tables.ControlMode], help="기본 mit")

    i = sub.add_parser("can-id", help="[영구] CAN id 변경")
    i.add_argument("joint", nargs="?", help="생략하면 물어봄")
    i.add_argument("--to", type=int, help="바꿀 CAN id. 생략하면 물어봄")
    i.add_argument("--yes", action="store_true", help=YES_HELP)

    pr = sub.add_parser("protocol", help="[영구] 프로토콜 전환. 전원 재투입 필요")
    pr.add_argument("joint", nargs="?", help="생략하면 물어봄")
    pr.add_argument("--to", choices=[x.name.lower() for x in tables.Protocol], help="생략하면 물어봄")
    pr.add_argument("--yes", action="store_true", help=YES_HELP)

    return p


HANDLERS = {
    "scan": cmd_scan,
    "state": cmd_state,
    "fault": cmd_fault,
    "clear-fault": cmd_clear_fault,
    "sweep": cmd_sweep,
    "nudge": cmd_nudge,
    "zero": cmd_zero,
    "mode": cmd_mode,
    "can-id": cmd_can_id,
    "protocol": cmd_protocol,
}


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(message)s",
    )

    path = args.config or _find_config()
    if path is None:
        raise SystemExit(
            f"{CONFIG_NAME} 을 찾지 못했음. 저장소 안에서 실행하거나 --config 로 지정할 것"
        )

    try:
        robot = load_robot(path)
    except ConfigError as e:
        raise SystemExit(f"{e}") from e

    limb = _pick_limb(robot, args.limb)

    # 승인 확인을 버스보다 먼저 함. 모터에 아무것도 보내지 않은 상태에서 멈추게 하려는 것임.
    if args.command in DANGEROUS and not getattr(args, "yes", False):
        raise SystemExit(
            f"{limb.name}.{getattr(args, 'joint', '')}: {DANGEROUS[args.command]}\n"
            f"되돌리기 어려운 조작임. 확인했으면 --yes 를 붙일 것."
        )

    bus = _open(limb)
    try:
        return HANDLERS[args.command](args, limb, bus)
    except (C.CommissioningError, cal.CalibrationError) as e:
        sys.stdout.flush()      # 앞서 찍은 안내문 뒤에 나오도록
        print(f"\n실패: {e}", file=sys.stderr)
        return 1
    finally:
        bus.disconnect()


if __name__ == "__main__":
    sys.exit(main())
