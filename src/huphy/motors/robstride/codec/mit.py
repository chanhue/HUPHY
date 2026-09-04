"""MIT 표준 프레임(11-bit ID) 인코딩/디코딩.

로봇을 전혀 모름. 관절 이름도 모터 배치도 없고 숫자와 바이트만 다룸.
인코딩 범위를 인자로 받으므로 모델·프로토콜에 묶이지 않음.


## 명령 프레임 — Command 3 "MIT Dynamic Parameters" (매뉴얼 p.38)

11-bit 표준 ID = 대상 모터 CAN ID

    Byte0~1               목표각    16bit  <-> (-pmax ~ pmax) rad
    Byte2 + Byte3[7:4]    목표속도  12bit  <-> (-vmax ~ vmax) rad/s
    Byte3[3:0] + Byte4    Kp        12bit  <-> (0 ~ 500)
    Byte5 + Byte6[7:4]    Kd        12bit  <-> (0 ~ 5)
    Byte6[3:0] + Byte7    목표토크  12bit  <-> (-tmax ~ tmax) N.m


## 응답 프레임 — Response Command 1 "Data Feedback" (매뉴얼 p.37)

    Byte0                 모터 CAN ID
    Byte1~2               현재각    16bit
    Byte3 + Byte4[7:4]    현재속도  12bit
    Byte4[3:0] + Byte5    현재토크  12bit
    Byte6~7               권선 온도 (0.1도 단위)

**명령과 응답의 바이트 배치가 다름** — 응답은 앞에 모터 ID가 붙어 한 칸씩 밀림.


## 고장 프레임 — Command 5 응답 (매뉴얼 p.39)

    Byte0                 모터 CAN ID
    Byte1~4               고장값. 0이면 정상

일반 상태 프레임과 CAN ID가 같아 겉으로 구분되지 않음. 조회 명령을 보낸 직후의
첫 응답으로 간주해야 함.


## 단위

내부는 rad, 외부는 deg. 변환은 이 파일에서만 일어남 — 나머지 코드가 라디안을
신경 쓰지 않아도 되도록 경계에 가둠.
"""

from __future__ import annotations

import math
from typing import Tuple

from ..tables import EncodingRange

FRAME_LEN = 8


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """실수를 [x_min, x_max] 범위 안에서 bits 비트 정수로 양자화함.

    범위를 벗어나면 **클램프됨** (감싸지 않음). 따라서 전송 전에 값이 범위 안인지
    확인해야 함 — 넘으면 조용히 최대/최소값이 나감.

    NaN은 min/max 비교가 전부 False라 이 클램프를 통과함. 상위(safety.guards)에서
    걸러야 함.
    """
    x = max(x_min, min(x_max, float(x)))
    span = x_max - x_min
    norm = (x - x_min) / span if span > 0 else 0.0
    return int(norm * ((1 << bits) - 1))


def uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    """float_to_uint의 역변환."""
    span = x_max - x_min
    norm = float(x) / ((1 << bits) - 1)
    return norm * span + x_min


def pack_command(
    *,
    position_deg: float,
    velocity_deg_s: float,
    kp: float,
    kd: float,
    torque_nm: float,
    enc: EncodingRange,
) -> bytes:
    """MIT 동작 제어 명령 8바이트를 만듦.

    모터 펌웨어가 이 다섯 값으로 PD를 계산함:
        tau = kp*(목표각 - 현재각) + kd*(목표속도 - 현재속도) + 토크_FF
    """
    q = float_to_uint(math.radians(position_deg), -enc.pmax_rad, enc.pmax_rad, enc.pos_bits)
    dq = float_to_uint(
        math.radians(velocity_deg_s), -enc.vmax_rad_s, enc.vmax_rad_s, enc.vel_bits
    )
    kp_u = float_to_uint(kp, 0.0, enc.kp_max, enc.gain_bits)
    kd_u = float_to_uint(kd, 0.0, enc.kd_max, enc.gain_bits)
    tau = float_to_uint(torque_nm, -enc.tmax_nm, enc.tmax_nm, enc.tau_bits)

    return bytes(
        [
            (q >> 8) & 0xFF,                            # Byte0  목표각 상위
            q & 0xFF,                                   # Byte1  목표각 하위
            (dq >> 4) & 0xFF,                           # Byte2  목표속도 상위 8
            ((dq & 0x0F) << 4) | ((kp_u >> 8) & 0x0F),  # Byte3  속도 하위4 | Kp 상위4
            kp_u & 0xFF,                                # Byte4  Kp 하위 8
            (kd_u >> 4) & 0xFF,                         # Byte5  Kd 상위 8
            ((kd_u & 0x0F) << 4) | ((tau >> 8) & 0x0F), # Byte6  Kd 하위4 | 토크 상위4
            tau & 0xFF,                                 # Byte7  토크 하위 8
        ]
    )


def decode_state(data: bytes, *, enc: EncodingRange) -> Tuple[int, float, float, float, float]:
    """상태 프레임을 해석함.

    반환: (motor_id, position_deg, velocity_deg_s, torque_nm, temp_c)
    """
    if len(data) < FRAME_LEN:
        raise ValueError(f"상태 프레임은 {FRAME_LEN}바이트여야 함 (받은 길이 {len(data)})")

    motor_id = int(data[0])
    q_u = (data[1] << 8) | data[2]
    dq_u = (data[3] << 4) | (data[4] >> 4)
    tau_u = ((data[4] & 0x0F) << 8) | data[5]
    temp_u = (data[6] << 8) | data[7]

    pos_rad = uint_to_float(q_u, -enc.pmax_rad, enc.pmax_rad, enc.pos_bits)
    vel_rad = uint_to_float(dq_u, -enc.vmax_rad_s, enc.vmax_rad_s, enc.vel_bits)
    tau_nm = uint_to_float(tau_u, -enc.tmax_nm, enc.tmax_nm, enc.tau_bits)

    return (
        motor_id,
        math.degrees(pos_rad),
        math.degrees(vel_rad),
        tau_nm,
        float(temp_u) / 10.0,
    )


def decode_fault(data: bytes) -> Tuple[int, int]:
    """고장 응답 프레임을 해석함. 반환: (motor_id, fault_word). 0이면 정상.

    일반 상태 프레임과 CAN ID가 같아 겉으로 구분되지 않으므로, 조회 명령을 보낸
    직후의 첫 응답으로 간주해야 함.

    **고장값은 리틀 엔디안임 (2026-09-05 수정).** 상태 프레임의 위치·속도·토크·온도는
    빅 엔디안으로 채워지지만(`decode_state`), 고장/경고값은 그렇지 않음. 같은 프로토콜
    안에서 프레임마다 바이트 순서가 다름.

    근거 둘.

    1. 벤더 매뉴얼(`RS03/RS04 User Manual 251112`, 통신 타입 21 표): 데이터 필드는
       ``Byte0~3 고장값 / Byte4~7 경고값``. 파라미터 프레임 표에는 "the low byte is
       first and the high byte is second" 라고 적혀 있음.
    2. 벤더 공식 SDK(`ROBSTRIDE-DYNAMICS/Robstride-Dynamics-Python-SDK`,
       `robstride_dynamics/bus.py`)는 같은 프레임을 ``struct.unpack("<LL", data)`` 로
       읽음 — 리틀 엔디안 32비트 두 개.

    빅 엔디안으로 읽으면 **정의된 고장이 정의되지 않은 값으로 둔갑함.** 실측(벤치,
    2026-09-05): 두 모터가 동시에 정지했을 때 바이트가 ``08 00 00 00`` 이었는데,
    빅 엔디안으로 읽으면 ``0x08000000``(비트 27, 매뉴얼에 없는 값)이 되고 리틀
    엔디안으로 읽으면 ``0x00000008``(비트 3, 과전압)이 됨. 두 모터가 한 전원에서
    동시에 감속했으므로 과전압이 물리적으로도 맞음. 진단이 통째로 어긋났었음.

    MIT 프로토콜(11비트 표준 프레임)에서는 데이터 첫 바이트가 모터 id 라 매뉴얼의
    바이트 번호가 한 칸 밀림 — 매뉴얼/공식 SDK 는 29비트 확장 프레임 기준이고 거기서는
    모터 id 가 CAN ID 안에 들어감. 실측 응답: RS03 ``03 00 00 00 00``(5바이트),
    RS04 ``04 00 00 00 00 00 00 00``(8바이트).
    """
    if len(data) < 5:
        raise ValueError(f"고장 프레임은 최소 5바이트 필요 (받은 길이 {len(data)})")
    motor_id = int(data[0])
    word = int.from_bytes(data[1:5], "little")
    return motor_id, word
