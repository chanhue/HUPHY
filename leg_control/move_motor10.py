#!/usr/bin/env python3
from __future__ import annotations

"""10번 모터(오른쪽 다리 knee, can1)를 73도 위치로 이동시킨다.

commission_motor.py의 저수준 MIT 함수(enable_mit/disable_mit/mit_position_command)를
그대로 재사용한다. 현재 각도를 먼저 읽어 그 지점부터 목표각까지 선형으로 램프하면서
50Hz로 계속 재전송한다 (급격한 위치 점프 방지 + MIT 명령 타임아웃 방지).

실행 전 확인:
  - 10번 모터가 MIT 모드이고 CAN이 can1에 연결되어 있는지 (commission_motor.py scan으로 확인 가능)
  - robot_constant.py 기준 10번 모터의 raw 각도 리밋은 (-20.65, 74.79)도 -- 73도는 그
    상한(74.79)에 매우 가까우므로 실제 관절이 하드스톱 근처에 있지 않은지 먼저 눈으로 확인할 것.
"""

import sys
import time

from commission_motor import (
    _confirm,
    _open_bus,
    _shutdown_bus,
    disable_mit,
    enable_mit,
    mit_position_command,
)

MOTOR_ID = 10
CHANNEL = "can1"
INTERFACE = "socketcan"

TARGET_DEG = 73.0
KP = 8.0
KD = 0.3
RAMP_SPEED_DEG_S = 20.0  # 램프 속도
DT = 0.02  # 50Hz 재전송 주기
HOLD_S = 2.0  # 목표각 도달 후 유지 시간


def _shortest_raw_target(current_raw_deg: float, target_deg: float) -> float:
    """target_deg + k*360 중 current_raw_deg(모터가 보고하는 실제 절대 raw 각도)에서
    최단 거리인 표현을 고른다. 예: current=350, target=73 -> 73+360=433을 골라 -277(먼
    길, 반대 방향으로 거의 한 바퀴)이 아니라 +83(짧은 길)로 이동하게 한다.

    주의: current를 먼저 (-180,180] 같은 구간으로 wrap하면 안 된다. 모터에 보내는
    position_deg는 절대 인코더 값이라, wrap된 값을 기준으로 계산하면 실제로는 존재하지
    않는 360도짜리 이동 명령이 나가서 엉뚱한 방향/거리로 움직인다.
    """
    diff = ((target_deg - current_raw_deg + 180.0) % 360.0) - 180.0
    return current_raw_deg + diff


def ramp_and_hold(bus, motor_id: int, start_deg: float, target_deg: float) -> None:
    distance = abs(target_deg - start_deg)
    duration = max(distance / RAMP_SPEED_DEG_S, DT)
    n_steps = max(1, int(duration / DT))

    print(f"[모터 {motor_id}] {start_deg:.2f}도 -> {target_deg:.2f}도 램프 시작 ({n_steps} step)")
    for i in range(1, n_steps + 1):
        frac = i / n_steps
        pos = start_deg + (target_deg - start_deg) * frac
        mit_position_command(bus, motor_id, pos, kp=KP, kd=KD, timeout_s=0.03)
        time.sleep(DT)

    print(f"[모터 {motor_id}] 목표각 {target_deg:.2f}도 도달, {HOLD_S:.1f}초 유지")
    t0 = time.monotonic()
    while time.monotonic() - t0 < HOLD_S:
        mit_position_command(bus, motor_id, target_deg, kp=KP, kd=KD, timeout_s=0.03)
        time.sleep(DT)


def main() -> int:
    bus = _open_bus(INTERFACE, CHANNEL)
    try:
        print(f"[모터 {MOTOR_ID}] 활성화(Enable) 시도 중...")
        if not enable_mit(bus, MOTOR_ID):
            print(f"[실패] 모터 {MOTOR_ID} 활성화 응답 없음. channel/motor-id/전원을 확인하세요.")
            return 1

        # kp=kd=0 명령으로 토크 없이 현재 실측 각도만 읽는다.
        ok, fb = mit_position_command(
            bus, MOTOR_ID, 0.0, kp=0.0, kd=0.0, return_feedback=True
        )
        if not ok or fb is None:
            print(f"[경고] 모터 {MOTOR_ID} 현재 각도를 읽지 못했습니다. 0도에서 시작한다고 가정합니다.")
            start_deg = 0.0
        else:
            start_deg = fb["angle_deg"]
            print(f"[모터 {MOTOR_ID}] 현재 각도: {start_deg:.2f}도")

        target_deg = _shortest_raw_target(start_deg, TARGET_DEG)
        print(f"[모터 {MOTOR_ID}] 이동량: {target_deg - start_deg:+.2f}도 (최단 경로)")

        proceed = _confirm(
            f"{start_deg:.2f}도 -> {target_deg:.2f}도로 ({target_deg - start_deg:+.2f}도) 이동할까요?",
            default_yes=False,
        )
        if not proceed:
            print("취소되었습니다. 모터를 비활성화합니다.")
            return 0

        ramp_and_hold(bus, MOTOR_ID, start_deg, target_deg)
        return 0
    except KeyboardInterrupt:
        print("\n중단됨 -- 모터를 비활성화합니다.")
        return 130
    finally:
        disable_mit(bus, MOTOR_ID)
        _shutdown_bus(bus)


if __name__ == "__main__":
    raise SystemExit(main())
