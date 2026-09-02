"""RobStride 사양 테이블 — 데이터시트에서 오는 값.

실측이 아님. 모터를 바꾸지 않는 한 변하지 않음.
실측값(sign/offset/limits/gains)은 calibration/ 으로 감.

## 출처

제조사 GitHub 의 `260713` 판본임. 모델마다 파일이 따로 있고 절 번호가 같음.

    github.com/RobStride/Product_Information
      Product Literature/RS00/RS00User Manual260713.pdf
      Product Literature/RS02/RS02User Manual260713.pdf
      Product Literature/RS03/RS03User Manual260713.pdf
      Product Literature/RS04/RS04User Manual260713.pdf

    4.1.2  Communication Type 1 / 2         -> private
    6.5    Command 3 MIT Dynamic Parameters -> MIT. **여기 값을 씀**

**판본을 확인하고 볼 것.** 예전 판본(Seeed 배포 `251112`, Shopify CDN)은 MIT 표에
다른 숫자가 적혀 있음 -- RS02 속도가 33, RS04 각도가 15 로 되어 있어 이 표와
어긋남.


## 인코딩 범위는 "모델별"이 아니라 "프로토콜 x 모델"임

`260713` 판본에서는 **네 모델 다 두 프로토콜의 범위가 같음.** 축을 남겨 둔 것은
예전 판본이 다른 값을 적고 있었고, 펌웨어가 어느 쪽을 따르는지 실물로 확인한 적이
없기 때문임.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Dict


class Protocol(IntEnum):
    """모터의 통신 프로토콜.

    Command 8 (MIT) 또는 Type 25 (private)로 전환하며, **전원 재투입 후 적용됨.**
    현재 설정은 파라미터 0x201F protocol_1 로 확인.
    """

    PRIVATE = 0   # 공장 기본값. 29-bit 확장 프레임
    CANOPEN = 1   # CiA 402
    MIT = 2       # 11-bit 표준 프레임 <- 본 프로젝트가 사용


class ControlMode(IntEnum):
    """MIT 프로토콜 내부의 제어 모드. Command 6으로 전환."""

    MIT = 0       # 전원 투입 기본값. 위치/속도/kp/kd/토크 5개 파라미터
    POSITION = 1  # 목표 위치 + 속도 제한. 모터가 궤적을 생성
    VELOCITY = 2  # 목표 속도 + 전류 제한


class Model(str, Enum):
    RS00 = "RS00"
    RS02 = "RS02"
    RS03 = "RS03"
    RS04 = "RS04"


# ---------------------------------------------------------------------------
# 인코딩 범위
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class EncodingRange:
    """MIT류 프레임의 고정점 양자화 범위.

    CAN 2.0 프레임은 데이터가 8바이트뿐이라 부동소수를 쓸 수 없음. 각 값을
    [-max, max] 범위 안에서 정수로 양자화함.

        uint = (값 + max) / (2*max) * (2**bits - 1)

    해상도 = 2*max / (2**bits - 1). 범위를 넓게 잡을수록 해상도가 나빠짐.
    위치에 16비트를 몰아주고 나머지를 12비트로 깎아 8바이트에 맞춤
    (16+12+12+12+12 = 64bit).
    """

    pmax_rad: float
    vmax_rad_s: float
    tmax_nm: float
    pos_bits: int = 16
    vel_bits: int = 12
    tau_bits: int = 12
    gain_bits: int = 12
    kp_max: float = 500.0
    kd_max: float = 5.0


# MIT 프로토콜 (11-bit 표준 프레임) — 본 프로젝트가 사용
#
#   매뉴얼 6.5 "Command 3: MIT Dynamic Parameters":
#     Byte0~1               목표각    [0~65535] <-> (-12.57 rad ~ 12.57 rad)
#     Byte2 + Byte3[7:4]    목표속도  [0~4096]  <-> 모델마다 다름
#     Byte3[3:0] + Byte4    Kp        [0~4096]  <-> 모델마다 다름
#     Byte5 + Byte6[7:4]    Kd        [0~4096]  <-> 모델마다 다름
#     Byte6[3:0] + Byte7    목표토크  [0~4096]  <-> 모델마다 다름
MIT_ENCODING: Dict[Model, EncodingRange] = {
    #        각도     속도    Kp      Kd     토크
    #  RS00  ±12.57   ±33     0~500   0~5    ±14 N.m
    #  RS02  ±12.57   ±44     0~500   0~5    ±17 N.m
    #  RS03  ±12.57   ±20     0~5000  0~100  ±60 N.m
    #  RS04  ±12.57   ±15     0~5000  0~100  ±120 N.m
    #
    # 각도만 네 모델이 같음. 큰 모터일수록 속도 범위가 좁고 게인 범위가 넓음.
    Model.RS00: EncodingRange(pmax_rad=12.57, vmax_rad_s=33.0, tmax_nm=14.0),
    Model.RS02: EncodingRange(pmax_rad=12.57, vmax_rad_s=44.0, tmax_nm=17.0),
    Model.RS03: EncodingRange(
        pmax_rad=12.57, vmax_rad_s=20.0, tmax_nm=60.0, kp_max=5000.0, kd_max=100.0
    ),
    Model.RS04: EncodingRange(
        pmax_rad=12.57, vmax_rad_s=15.0, tmax_nm=120.0, kp_max=5000.0, kd_max=100.0
    ),
}
"""**모델마다 다름.** 같은 정수를 서로 다른 값으로 되돌리므로, 표가 틀리면 그
비율만큼 어긋남 -- 프레임에는 Nm 이나 rad 가 아니라 범위 안의 눈금만 실림.

한 다리 안에서도 갈림.

    0.5   발목이 RS00, 나머지가 RS02
    1.0   hip_yaw 와 발목이 RS03, 나머지가 RS04

**게인 범위가 제일 위험함.** RS03·RS04 가 `0~5000` / `0~100` 이고 나머지가
`0~500` / `0~5` 임. 표를 `500` 으로 둔 채 RS04 에 `kp=20` 을 보내면 모터가 `199` 로
받음 -- 10배임. `kd` 는 20배가 됨.

첫 통전에서 다리가 예상보다 세게 튀면 이 표부터 볼 것. `--gain-scale` 로 낮춰도
비율은 그대로임.
"""

# private 프로토콜 (29-bit 확장 프레임) — 매뉴얼 4.1.2 Communication Type 1
#
# **범위는 MIT 와 같음.** 다른 것은 비트 폭뿐임 -- private 은 토크가 중재 id 안에
# 들어가고 데이터에 16bit 씩 네 개가 들어가는데, MIT 은 8바이트에 다섯 값을 넣느라
# 16+12+12+12+12 로 쪼개 씀.
#
# 현재 코드는 MIT 만 사용함. 이 표는 손으로 확장 프레임을 만들 때(docs/motor_setup.md)
# 눈금이 같은지 확인하는 용도로 둠.
PRIVATE_ENCODING: Dict[Model, EncodingRange] = {
    Model.RS00: EncodingRange(
        pmax_rad=12.57, vmax_rad_s=33.0, tmax_nm=14.0, vel_bits=16, tau_bits=16
    ),
    Model.RS02: EncodingRange(
        pmax_rad=12.57, vmax_rad_s=44.0, tmax_nm=17.0, vel_bits=16, tau_bits=16
    ),
    Model.RS03: EncodingRange(
        pmax_rad=12.57, vmax_rad_s=20.0, tmax_nm=60.0,
        kp_max=5000.0, kd_max=100.0, vel_bits=16, tau_bits=16,
    ),
    Model.RS04: EncodingRange(
        pmax_rad=12.57, vmax_rad_s=15.0, tmax_nm=120.0,
        kp_max=5000.0, kd_max=100.0, vel_bits=16, tau_bits=16,
    ),
}

ENCODING: Dict[Protocol, Dict[Model, EncodingRange]] = {
    Protocol.MIT: MIT_ENCODING,
    Protocol.PRIVATE: PRIVATE_ENCODING,
}


def encoding_for(model: Model, protocol: Protocol = Protocol.MIT) -> EncodingRange:
    """모델·프로토콜 조합의 인코딩 범위. 없으면 KeyError.

    조용히 기본값으로 때우지 않음 — 범위가 틀리면 명령한 값과 실제가 배율만큼
    어긋나는데, 그건 실물에서 찾기 매우 어려움.
    """
    try:
        return ENCODING[protocol][model]
    except KeyError as exc:
        raise KeyError(
            f"{model.value} / {protocol.name} 조합의 인코딩 범위가 tables.py에 없음. "
            f"매뉴얼에서 확인해 추가할 것."
        ) from exc


# ---------------------------------------------------------------------------
# MIT 표준 프레임 명령 (매뉴얼 p.38~39)
#
#   data[0:6] = 0xFF,  data[6] = F_CMD,  data[7] = 명령 코드
#
#   F_CMD가 0xFF면 기본 동작, 다른 값이면 변형 동작임.
#     0xFC + F_CMD=0xFF -> Enable        / 0xFC + F_CMD=mode  -> 제어모드 설정
#     0xFD + F_CMD=0xFF -> Stop          / 0xFD + F_CMD=proto -> 프로토콜 전환
#     0xFB + F_CMD=0xFF -> 고장 클리어    / 0xFB + F_CMD=그외  -> 고장값 조회
# ---------------------------------------------------------------------------
CMD_ENABLE = 0xFC        # Command 1. F_CMD=0xFF
CMD_STOP = 0xFD          # Command 2. F_CMD=0xFF
CMD_SET_ZERO = 0xFE      # Command 4. 비위치 모드에서만 동작
CMD_FAULT = 0xFB         # Command 5. F_CMD로 클리어/조회 구분
CMD_SET_MODE = 0xFC      # Command 6. F_CMD = ControlMode
CMD_SET_CAN_ID = 0xFA    # Command 7. F_CMD = 새 CAN ID
CMD_SET_PROTOCOL = 0xFD  # Command 8. F_CMD = Protocol. 전원 재투입 후 적용

F_CMD_DEFAULT = 0xFF     # 기본 동작
F_CMD_FAULT_QUERY = 0x00 # 고장값 조회 (0xFF가 아니면 무엇이든 조회)


# ---------------------------------------------------------------------------
# 고장 비트 (Command 5 응답 Byte1~4)
# ---------------------------------------------------------------------------
FAULT_BITS: Dict[str, int] = {
    "stall_overload": 14,       # 스톨 / I^2t 과부하
    "encoder_uncalibrated": 7,
    "overvoltage": 3,
    "undervoltage": 2,
    "driver_ic": 1,
    "overtemperature": 0,       # 기본 임계 130도 (매뉴얼 p.39)
}


# ---------------------------------------------------------------------------
# 파라미터 인덱스
#
# 본 프로젝트의 하드웨어 전제를 확인하는 데 쓰는 것만 둠. 나머지는 필요해질 때
# 매뉴얼에서 추가할 것.
# ---------------------------------------------------------------------------
PARAM_PROTOCOL_FLAG = 0x201F   # 현재 통신 프로토콜
PARAM_ZERO_STA = 0x7029        # 0 = 위치 보고 [0,360), 1 = [-180,180)
