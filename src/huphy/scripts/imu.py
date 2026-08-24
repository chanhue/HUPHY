"""IMU 커미셔닝 진입점 — 센서를 설정하고 확인함.

    python -m huphy.scripts.imu show
    python -m huphy.scripts.imu apply --yes
    python -m huphy.scripts.imu check
    python -m huphy.scripts.imu watch

`robot.yaml` 의 `imus` 항목을 읽으므로 포트나 보레이트를 손으로 적지 않음.
`scripts/commission.py` 가 모터에 대해 하는 일과 같은 자리임.


## 설정 파일이 기준임

센서에 저장된 설정을 읽어 `robot.yaml` 과 대조하고, 다르면 **센서를 파일에 맞춤.**
반대로 하지 않음 -- 센서에 물어보고 코드가 따라가면, 센서를 갈아 끼우거나 누가
MT Manager 로 설정을 바꿨을 때 동작이 조용히 달라짐.


## 되돌리기 어려운 것은 --yes 를 요구함

    apply     출력 항목과 주기를 바꿈. 센서 비휘발성 메모리에 저장됨

전원을 껐다 켜도 남고, 되돌리려면 반대 명령을 보내야 함. 실수로 방향키를 눌러
실행되는 자리에 두지 않으려는 것임.

`show` 와 `check` 는 읽기만 하므로 확인을 안 받음.


## 어느 IMU 인지

`--imu` 로 고름. 하나뿐이면 생략해도 됨. 여럿인데 안 고르면 멈춤 -- 엉뚱한 센서
설정을 덮어쓰는 것을 막으려는 것임.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from ..config import ConfigError, load_robot
from ..config.schema import ImuConfig, RobotConfig
from ..sensors import make_imu
from ..sensors.ebimu import commands as C
from ..sensors.ebimu import commissioning as M
from . import table

CONFIG_NAME = "config/robot.yaml"

DANGEROUS = {
    "apply": "센서 출력 설정을 바꿈. 비휘발성 메모리에 저장되어 전원을 껐다 켜도 남음.",
}
"""`--yes` 없이는 포트를 열지도 않는 명령."""

SUPPORTED = ("ebimu",)
"""이 도구가 다룰 수 있는 model. 다른 센서는 설정 프로토콜이 달라 여기서 못 만짐."""

WATCH_HZ = 10.0
"""`watch` 화면 갱신 주기. 센서 출력 주기와 무관함 -- 사람 눈에 맞춘 값임."""


def _find_config() -> Optional[Path]:
    """현재 폴더부터 위로 올라가며 `config/robot.yaml` 을 찾음."""
    here = Path.cwd().resolve()
    for folder in (here, *here.parents):
        candidate = folder / CONFIG_NAME
        if candidate.is_file():
            return candidate
    return None


def _pick_imu(robot: RobotConfig, name: Optional[str]) -> ImuConfig:
    """어느 IMU 를 다룰지 고름. 못 고르면 종료함."""
    if not robot.imus:
        sys.exit(f"[!] {CONFIG_NAME} 에 imus 항목이 없음")

    if name is None:
        if len(robot.imus) == 1:
            return next(iter(robot.imus.values()))
        sys.exit(
            f"[!] IMU 가 여럿임. --imu 로 고를 것 (가용: {sorted(robot.imus)})"
        )

    if name not in robot.imus:
        sys.exit(f"[!] 모르는 IMU {name!r} (가용: {sorted(robot.imus)})")
    return robot.imus[name]


def _require_supported(config: ImuConfig) -> None:
    if config.model not in SUPPORTED:
        sys.exit(
            f"[!] {config.name}: model {config.model!r} 은 이 도구로 설정할 수 없음 "
            f"(가용: {list(SUPPORTED)}). 센서마다 설정 프로토콜이 다름"
        )


def _wanted(config: ImuConfig) -> List[str]:
    """`robot.yaml` 이 요구하는 센서 설정 명령들."""
    try:
        return C.output_commands(
            config.output,
            accel_mode=config.accel_mode,
            dist_mode=config.dist_mode,
            rate_hz=config.rate_hz,
        )
    except ValueError as e:
        sys.exit(f"[!] {CONFIG_NAME} 의 imus.{config.name}: {e}")


def _open_serial(config: ImuConfig) -> Any:
    """설정용으로 포트를 엶. **읽기 스레드를 띄우지 않음.**

    `EbimuImu` 는 스레드로 계속 읽는데, 설정 명령의 응답을 받으려면 그 스레드가
    없어야 함 -- 응답을 스레드가 먼저 먹어 버림.
    """
    try:
        import serial  # noqa: PLC0415
    except ImportError:
        sys.exit("[!] pyserial 이 필요함.  pip install 'huphy[imu]'")

    baud = config.baudrate or C.BAUD_DEFAULT
    try:
        return serial.Serial(port=config.port, baudrate=baud, timeout=0.2)
    except Exception as e:
        sys.exit(
            f"[!] {config.port} 를 열 수 없음: {e}\n"
            f"    포트 확인   ls /dev/ttyUSB* /dev/ttyACM* /dev/serial*\n"
            f"    권한        sudo usermod -aG dialout $USER  (재로그인)"
        )


def _head(config: ImuConfig) -> None:
    baud = config.baudrate or C.BAUD_DEFAULT
    print(f"\n  {config.name}  {config.port} @ {baud}  ({config.model})")


# ===========================================================================
# 명령
# ===========================================================================
def cmd_show(args, config: ImuConfig) -> int:
    """센서에 저장된 설정을 읽어 `robot.yaml` 과 대조함. **아무것도 안 바꿈.**"""
    port = _open_serial(config)
    try:
        settings = M.read_settings(port)
    finally:
        port.close()

    _head(config)
    if not settings:
        print("\n  [!] <cfg> 에 답하지 않음. 보레이트가 센서 설정과 같은지 확인할 것\n")
        return 1

    print("\n  센서에 저장된 설정")
    for command, detail in C.describe(settings):
        print(f"    {command:<10} {detail}")

    on_sensor = C.output_from_config(settings)
    print(f"\n  센서가 내는 항목   {'+'.join(on_sensor)}  ({C.field_count(on_sensor)}개)")
    print(f"  robot.yaml        {'+'.join(config.output)}  "
          f"({C.field_count(config.output)}개)")

    mismatches = M.compare(settings, _wanted(config))
    if not mismatches:
        print("\n  일치함.\n")
        return 0

    print(f"\n  ** 어긋남 {len(mismatches)}개 **")
    print("  " + table.header(("명령", 10, "<"), ("지금", 26, "<"), ("설정 파일", 26, "<")))
    for item in mismatches:
        print(f"  {item.command:<10} {table.cell(item.now, 26, align='<')}"
              f"{table.cell(item.wanted, 26, align='<')}")
    print("\n  huphy-imu apply --yes 로 맞출 것\n")
    return 1


def cmd_apply(args, config: ImuConfig) -> int:
    """`robot.yaml` 대로 센서를 맞춤. **센서에 저장됨.**"""
    port = _open_serial(config)
    try:
        settings = M.read_settings(port)
        if not settings:
            print("\n  [!] <cfg> 에 답하지 않음. 보레이트를 확인할 것\n")
            return 1

        mismatches = M.compare(settings, _wanted(config))
        _head(config)
        if not mismatches:
            print("\n  이미 설정 파일과 같음. 아무것도 보내지 않음.\n")
            return 0

        print("\n  보낼 명령")
        for item in mismatches:
            print(f"    {item.command:<10} {item.now}  ->  {item.wanted}")

        results = M.apply(port, mismatches)
    finally:
        port.close()

    print("\n  응답")
    for command, response in results:
        print(f"    {command:<10} {response or '(없음)'}")
    print("\n  센서에 저장됨. 되돌리려면 반대 명령을 보낼 것.\n")
    return 0


def cmd_check(args, config: ImuConfig) -> int:
    """부착 방향을 가속도계와 대조함. **아무것도 안 바꿈.**"""
    print(
        "\n  센서를 **두 축으로 기울여** 가만히 둘 것.\n"
        "  한 축만 기울이면 못 잡는 어긋남이 있고, 움직이는 중이면 가속도계가\n"
        "  중력 말고 다른 것도 재서 결과가 의미 없음.\n"
    )

    imu = make_imu(config)
    imu.connect()
    try:
        state = M.sample(imu, seconds=2.0)
    finally:
        imu.disconnect()

    if state is None:
        print("  [!] 값이 안 들어옴.\n")
        return 1

    result = M.check_mount(state)
    _head(config)
    print()
    print("    자세에서 계산한 중력   ({:7.3f},{:7.3f},{:7.3f})".format(*result.from_attitude))
    print("    가속도계가 잰 중력     ({:7.3f},{:7.3f},{:7.3f})".format(*result.from_accel))
    print(f"    오차                   {result.error:.4f}")
    print(f"    기울기                 {result.tilt:.3f}")
    print(f"    가속도계 부호          {result.accel_sign:+d}")

    if not result.tilted_enough:
        print(
            f"\n  [!] 너무 수평임 (기울기 {result.tilt:.3f} < {M.MIN_TILT}). "
            f"**이 상태로는 부착이 틀려도 통과함** -- 기울여서 다시 잴 것\n"
        )
        return 1
    if not result.ok:
        print(
            f"\n  ** 어긋남 (허용 {M.LEVEL_TOLERANCE}) **\n"
            f"  센서가 예상과 다른 방향으로 붙어 있음. 어느 성분이 다른지 위에서\n"
            f"  볼 것 -- 부호만 반대면 축 방향, 자리가 바뀌었으면 축 순서임.\n"
        )
        return 1
    print("\n  부착 방향 정상.\n")
    return 0


def cmd_watch(args, config: ImuConfig) -> int:
    """들어오는 값을 계속 보여줌. **아무것도 안 바꿈.**"""
    imu = make_imu(config)
    imu.connect()
    period = 1.0 / WATCH_HZ
    lines = 0

    try:
        while True:
            time.sleep(period)
            state = imu.read()
            block = _watch_block(imu, config, state)
            if lines:
                sys.stdout.write(f"\033[{lines}A")
            sys.stdout.write("\n".join(block) + "\n")
            sys.stdout.flush()
            lines = len(block)
    except KeyboardInterrupt:
        pass
    finally:
        imu.disconnect()
    print("\n  종료했음.\n")
    return 0


def _watch_block(imu: Any, config: ImuConfig, state: Any) -> List[str]:
    baud = config.baudrate or C.BAUD_DEFAULT
    dropped = getattr(imu, "dropped", 0)
    out = [
        f"  {config.name}  {config.port} @ {baud}",
        f"  버린 줄 {dropped}    마지막 값 {_age(state)}",
        "  " + "-" * 52,
    ]
    if not state.is_valid:
        out.append("  값이 아직 없음")
        out.append("  " + "-" * 52)
        out.append("  Ctrl-C 로 종료")
        return out

    out.append("  중력    ({:7.3f},{:7.3f},{:7.3f})".format(*state.gravity))
    out.append("  각속도  ({:7.2f},{:7.2f},{:7.2f})  도/초".format(*state.gyro_dps))
    out.append("  가속도  ({:7.2f},{:7.2f},{:7.2f})  m/s^2".format(*state.accel_mps2))
    for field in getattr(imu, "extra_fields", ()):
        out.append(f"  {field:<8}{state.extra.get(field, 0.0):10.3f}")
    out.append("  " + "-" * 52)
    out.append("  Ctrl-C 로 종료")
    return out


def _age(state: Any) -> str:
    age = state.age_ms()
    return "없음" if age < 0 else f"{age:.0f}ms"


COMMANDS = {
    "show": cmd_show,
    "apply": cmd_apply,
    "check": cmd_check,
    "watch": cmd_watch,
}


# ===========================================================================
# 진입점
# ===========================================================================
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="huphy-imu",
        description="IMU 설정과 확인. robot.yaml 의 imus 항목을 씀",
    )
    p.add_argument("--config", help=f"기본: 위로 올라가며 {CONFIG_NAME} 을 찾음")
    p.add_argument("--imu", help="어느 IMU 인지. 하나뿐이면 생략 가능")
    p.add_argument("-v", "--verbose", action="store_true")

    sub = p.add_subparsers(dest="command", required=True)
    sub.add_parser("show", help="센서 설정을 읽어 robot.yaml 과 대조")
    a = sub.add_parser("apply", help="[영구] robot.yaml 대로 센서를 맞춤")
    a.add_argument("--yes", action="store_true", help=DANGEROUS["apply"])
    sub.add_parser("check", help="부착 방향을 가속도계와 대조")
    sub.add_parser("watch", help="들어오는 값을 계속 보여줌")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(name)s: %(message)s",
    )

    # 확인은 포트를 열기 전에 받음. 열고 나서 물으면 그 사이에 센서가 정지 상태로
    # 남을 수 있음.
    if args.command in DANGEROUS and not getattr(args, "yes", False):
        print(f"\n  [!] {DANGEROUS[args.command]}")
        print(f"      확인했으면 --yes 를 붙일 것\n")
        return 2

    path = Path(args.config) if args.config else _find_config()
    if path is None:
        print(f"[!] {CONFIG_NAME} 을 찾지 못함. --config 로 지정할 것")
        return 2

    try:
        robot = load_robot(path)
    except ConfigError as e:
        print(f"[!] 설정을 읽지 못함: {e}")
        return 2

    config = _pick_imu(robot, args.imu)
    _require_supported(config)
    return COMMANDS[args.command](args, config)


if __name__ == "__main__":
    raise SystemExit(main())
