"""센서 — 로봇이 자기 상태를 재는 것 중 모터가 아닌 것.

    base.py       ImuState, Imu. **벤더를 모르는 자료형과 계약**
    group.py      붙어 있는 IMU 묶음. 여는 것과 읽는 것
    registry.py   설정의 model 문자열 -> 구현체
    xsens/        Xsens MTi

`motors/` 와 같은 나눔임. 중립 계층이 자료형과 계약을 정하고, 벤더 폴더가 그것을
구현함. 센서를 갈아 끼워도 위쪽 코드는 바뀌지 않음.

IMU 는 **로봇 아래에 있고 어디에 붙었는지는 설정이 말함** (`ImuConfig.mount`).
다리에 붙였다가 몸통으로 옮기면 설정 한 줄만 바뀜.

실행 객체에서도 같음 -- `ImuGroup` 이 다리와 나란히 놓임. 다리의 부속이 아님.
"""

from .base import Imu, ImuState
from .group import ImuGroup
from .registry import MODELS, make_imu, make_imus

__all__ = ["Imu", "ImuState", "ImuGroup", "make_imu", "make_imus", "MODELS"]
