#!/usr/bin/env python3
from __future__ import annotations

"""RobStride 영점 영속성 테스트 (leg_control 자립 버전).

MIT 모드의 set-zero(0xFE)만으로 영점이 전원 재투입 후에도 유지되는지 실측한다.
필요하면 RobStride private 프로토콜의 "저장(type 22)" 프레임을 함께 보내 그때는
유지되는지도 비교한다.

이 버전은 인버스펜듈럼 쪽 commission_motor.py에 의존하지 않고, 이 패키지의
utils/mit_codec.py(decode_state_frame)와 robot_constant.py(명령 바이트/모터 사양),
그리고 python-can만 사용한다.

진행 흐름 (각 단계는 Enter를 눌러야 다음으로 넘어감):
  0) 연결/응답 확인
  1) 영점 전 현재 각도 읽기
  2) set-zero 전송 (기본 MIT 0xFE, --set-zero-mode private면 type 6)
  3) 각도 재읽기 → 0 근처여야 함
  4) --save 를 주면 type 22 저장 프레임도 전송 (안 주면 스킵)
  5) "모터 전원을 껐다 켜라" 안내 → 사용자가 재투입 후 Enter
  6) 각도 재읽기
  7) 판정: |각도| <= --zero-threshold(기본 3도)면 '유지됨', 아니면 '휘발'

사용 예:
  python3 test_zero_persistence.py --motor-id 10 --channel can1
  python3 test_zero_persistence.py --motor-id 10 --channel can1 --save

주의: 모터를 움직이지 않는다(영점 설정 + 상태 읽기만). 영점은 "지금 자세"를 0으로
      만드므로 원하는 기준 자세에 두고 실행할 것. 5)에서는 모터 '전원'만 재투입한다.
"""

import argparse
import math
import sys
import time
from pathlib import Path as _Path

try:
    import can
except Exception as exc:  # pragma: no cover
    print(f"python-can이 필요합니다: pip install python-can ({exc})", file=sys.stderr)
    raise

# 이 패키지의 utils/ 를 import 경로에 추가
_PKG_DIR = _Path(__file__).resolve().parent
if str(_PKG_DIR) not in sys.path:
    sys.path.insert(0, str(_PKG_DIR))

from utils.mit_codec import decode_state_frame
from robot_constant import MOTORS, CAN_CMD_ZERO, CAN_CMD_CLEAR_FAULT


def _open_bus(interface: str, channel: str):
    return can.interface.Bus(interface=interface, channel=channel)


def _flush_bus(bus, max_msgs: int = 1000) -> None:
    for _ in range(max_msgs):
        if bus.recv(0.0005) is None:
            break


def make_ext_id(comm_type: int, host_id: int, target_id: int) -> int:
    return ((comm_type & 0x1F) << 24) | ((host_id & 0xFFFF) << 8) | (target_id & 0xFF)


def read_angle_deg(bus, motor_id, *, pmax, vmax, tmax, timeout_s: float = 0.2):
    """clear-fault(0xFB) 핑을 보내 상태 프레임을 받고 각도(deg)를 반환. 실패 시 None."""
    data = [0xFF] * 7 + [CAN_CMD_CLEAR_FAULT]
    bus.send(can.Message(arbitration_id=int(motor_id), data=data, is_extended_id=False))
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.02)
        if msg is None:
            continue
        raw = bytes(msg.data)
        if len(raw) < 8 or raw[0] != int(motor_id):
            continue
        _mid, st = decode_state_frame(raw, pmax=pmax, vmax=vmax, tmax=tmax)
        return float(st.position_deg)
    return None


def read_angle_avg_deg(bus, motor_id, *, pmax, vmax, tmax, n=5):
    vals = []
    for _ in range(n):
        a = read_angle_deg(bus, motor_id, pmax=pmax, vmax=vmax, tmax=tmax)
        if a is not None:
            vals.append(a)
        time.sleep(0.02)
    return sum(vals) / len(vals) if vals else None


def set_zero_mit(bus, motor_id) -> bool:
    """MIT set-zero: 표준 ID = motor_id, data = [0xFF]*7 + [0xFE]."""
    data = [0xFF] * 7 + [CAN_CMD_ZERO]
    bus.send(can.Message(arbitration_id=int(motor_id), data=data, is_extended_id=False))
    reply = bus.recv(timeout=0.2)
    return reply is not None


def set_zero_private(bus, motor_id, *, host_id: int = 0):
    """RobStride private type 6(set mechanical zero) 확장 ID 프레임."""
    arb = make_ext_id(0x06, int(host_id), int(motor_id))
    bus.send(can.Message(arbitration_id=arb, data=[0x01, 0, 0, 0, 0, 0, 0, 0], is_extended_id=True))


def save_to_flash(bus, motor_id, *, host_id: int = 0):
    """RobStride private type 22(0x16, motor data save) 저장 프레임."""
    arb = make_ext_id(0x16, int(host_id), int(motor_id))
    bus.send(can.Message(arbitration_id=arb, data=[1, 2, 3, 4, 5, 6, 7, 8], is_extended_id=True))


def _fmt(a):
    return "N/A" if a is None else f"{a:.2f} deg"


def _pause(msg: str) -> None:
    input(f"\n[Enter] {msg}")


def main() -> int:
    ap = argparse.ArgumentParser(description="RobStride 영점 영속성 테스트 (leg_control)")
    ap.add_argument("--motor-id", type=int, required=True)
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--channel", default="can1")
    ap.add_argument("--host-id", type=int, default=0)
    ap.add_argument("--pmax", type=float, default=None, help="MIT 위치 범위(rad). 미지정 시 MOTORS 사양")
    ap.add_argument("--vmax", type=float, default=None)
    ap.add_argument("--tmax", type=float, default=None)
    ap.add_argument("--set-zero-mode", choices=("mit", "private"), default="mit")
    ap.add_argument("--save", action="store_true", help="영점 후 type22 저장 프레임도 전송")
    ap.add_argument("--zero-threshold", type=float, default=3.0)
    args = ap.parse_args()

    mid = args.motor_id
    spec = MOTORS.get(mid)
    pmax = args.pmax if args.pmax is not None else (float(spec.pmax_rad) if spec else 12.57)
    vmax = args.vmax if args.vmax is not None else (float(spec.vmax_rad_s) if spec else 33.0)
    tmax = args.tmax if args.tmax is not None else (float(spec.tmax_nm) if spec else 17.0)

    bus = _open_bus(args.interface, args.channel)
    try:
        _flush_bus(bus)
        print("========================================")
        print(f"모터 {mid} @ {args.channel}  (pmax={pmax}, vmax={vmax}, tmax={tmax})")
        print("각 단계는 Enter를 눌러야 다음으로 넘어갑니다.")
        print("========================================")

        _pause("연결/응답을 확인합니다 →")
        probe = read_angle_avg_deg(bus, mid, pmax=pmax, vmax=vmax, tmax=tmax)
        if probe is None:
            print(f"[실패] 모터 {mid} 응답 없음. channel/motor-id/전원/MIT모드 확인.")
            return 1
        print(f"0) 연결 OK (현재 각도 {_fmt(probe)})")

        _pause("[1] 영점 전 현재 각도를 읽습니다 →")
        before = read_angle_avg_deg(bus, mid, pmax=pmax, vmax=vmax, tmax=tmax)
        print(f"1) 영점 전 현재 각도  : {_fmt(before)}")

        _pause("[2] set-zero를 전송합니다 (지금 자세가 0이 됨) →")
        if args.set_zero_mode == "mit":
            ok = set_zero_mit(bus, mid)
            print(f"2) set-zero(MIT 0xFE) 전송 → ack={ok}")
        else:
            set_zero_private(bus, mid, host_id=args.host_id)
            print("2) set-zero(private type6) 전송")
        time.sleep(0.2)

        _pause("[3] 영점 직후 각도를 읽습니다 (0 근처여야 정상) →")
        after_zero = read_angle_avg_deg(bus, mid, pmax=pmax, vmax=vmax, tmax=tmax)
        print(f"3) 영점 직후 각도     : {_fmt(after_zero)}  (0 근처여야 정상)")

        if args.save:
            _pause("[4] type22 저장 프레임을 전송합니다 →")
            save_to_flash(bus, mid, host_id=args.host_id)
            time.sleep(0.3)
            print("4) type22 저장 프레임 전송함 (플래시 저장 시도)")
        else:
            print("4) 저장 명령 없음 (MIT set-zero 단독 영속성 테스트)")

        print("----------------------------------------")
        print(">>> 지금 모터 전원을 껐다가 다시 켜세요 (CAN/USB는 그대로 둬도 됨).")
        input(">>> 재투입이 끝났으면 Enter를 누르세요...")
        _flush_bus(bus)
        time.sleep(0.3)

        _pause("[6] 전원 재투입 후 각도를 읽습니다 →")
        after_cycle = read_angle_avg_deg(bus, mid, pmax=pmax, vmax=vmax, tmax=tmax)
        print(f"6) 전원 재투입 후 각도: {_fmt(after_cycle)}")

        print("========================================")
        if after_cycle is None:
            print("[판정 불가] 재투입 후 응답 없음. 다시 시도하세요.")
            return 1
        if abs(after_cycle) <= args.zero_threshold:
            print(f"✅ 영점 유지됨(영속): 재투입 후에도 {after_cycle:.2f} deg (≈0)")
            print("   → MIT set-zero만으로 저장됨." if not args.save else "   → type22 저장으로 유지됨.")
        else:
            print(f"❌ 영점 사라짐(휘발): 재투입 후 {after_cycle:.2f} deg (0에서 벗어남)")
            if not args.save:
                print("   → MIT set-zero 단독으론 저장 안 됨. --save 로 type22 저장을 테스트하세요.")
            else:
                print("   → type22 저장도 소용 없음. private 모드 전환 후 저장이 필요할 수 있음.")
        return 0

    except KeyboardInterrupt:
        print("\n중단됨.")
        return 130
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
