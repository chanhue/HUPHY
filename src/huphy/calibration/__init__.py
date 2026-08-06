"""캘리브레이션 계층.

    store.py   config/calibration/*.json 읽기·쓰기

조립을 재서 얻는 값만 다룸 -- sign, offset_deg, zero_reference. 한계와 게인은 적는
값이라 robot.yaml 에 있음 (이슈 #2).
"""

from . import store
from .store import CalibrationError, attach, identity, load, save, unmeasured

__all__ = [
    "store",
    "load",
    "save",
    "attach",
    "identity",
    "unmeasured",
    "CalibrationError",
]
