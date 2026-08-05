"""RobStride 사양 테이블 — 데이터시트에서 오는 값.

실측이 아님. 모터를 바꾸지 않는 한 변하지 않음.
실측값(sign/offset/limits/gains)은 calibration/ 으로 감.

출처: RS02 User Manual (Seeed Studio 배포판, 251112)
  p.20~21  Communication Type 1 / 2         -> private 프로토콜
  p.27     Motor Function Description       -> zero_sta, 프로토콜 전환
  p.37~40  MIT Communication Protocol       -> MIT 프로토콜, 명령 목록


## 인코딩 범위는 "모델별"이 아니라 "프로토콜 x 모델"임

같은 RS02라도 프로토콜에 따라 속도 범위가 다름. 이 축을 없애면 다른 프로토콜의
값을 가져다 쓰는 실수가 남.
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
#   매뉴얼 p.38 Command 3 "MIT Dynamic Parameters":
#     Byte0~1               목표각    [0~65535] <-> (-12.57 rad ~ 12.57 rad)
#     Byte2 + Byte3[7:4]    목표속도  [0~4096]  <-> (-33 rad/s ~ 33 rad/s)
#     Byte3[3:0] + Byte4    Kp        [0~4096]  <-> (0 ~ 500)
#     Byte5 + Byte6[7:4]    Kd        [0~4096]  <-> (0 ~ 5)
#     Byte6[3:0] + Byte7    목표토크  [0~4096]  <-> (-17 N.m ~ 17 N.m)
MIT_ENCODING: Dict[Model, EncodingRange] = {
    # TODO: verify — RS00 MIT 표는 직접 확인하지 못했음. RS02 매뉴얼 p.26의
    # 파라미터 목록(RS00에서 복사된 것으로 보임)을 근거로 (33, 14)로 둠.
    # RS00 매뉴얼로 교차 확인할 것.
    Model.RS00: EncodingRange(pmax_rad=12.57, vmax_rad_s=33.0, tmax_nm=14.0),
    Model.RS02: EncodingRange(pmax_rad=12.57, vmax_rad_s=33.0, tmax_nm=17.0),
}

# private 프로토콜 (29-bit 확장 프레임)
#
#   매뉴얼 p.20 Communication Type 1: 속도가 +-44 rad/s (16bit)
#   무부하 410rpm ~= 43 rad/s 이므로 private은 전 범위를, MIT은 12bit라 +-33으로
#   잘라 쓰는 것으로 보임.
#
# 현재 코드는 MIT만 사용하지만, **두 범위가 다르다는 사실 자체가 중요하므로**
# 표에 남김. 이 축이 없으면 private 값을 MIT에 가져다 쓰는 실수가 남.
PRIVATE_ENCODING: Dict[Model, EncodingRange] = {
    Model.RS02: EncodingRange(
        pmax_rad=12.57, vmax_rad_s=44.0, tmax_nm=17.0, vel_bits=16, tau_bits=16
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
# 파라미터 인덱스 (private type 17/18로 접근)
#
# 본 프로젝트의 하드웨어 전제를 확인하는 데 쓰는 것만 둠. 나머지는 필요해질 때
# 매뉴얼에서 추가할 것.
# ---------------------------------------------------------------------------
PARAM_PROTOCOL_FLAG = 0x201F   # 현재 통신 프로토콜. 전제 3 확인
PARAM_ZERO_STA = 0x7029        # 0 = 위치 보고 [0,360), 1 = [-180,180). 전제 1 확인
                               # 매뉴얼 p.27. Type 22로 저장해야 유지됨
