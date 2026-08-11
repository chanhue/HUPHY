"""Xsens MTi 시리즈 IMU.

    imu.py    `Imu` 프로토콜 구현. 위쪽이 쓰는 것
    xbus/     받아 온 시리얼 읽기 코드. 그대로 둔 자리
"""

from .imu import DEFAULT_BAUDRATE, XsensImu

__all__ = ["XsensImu", "DEFAULT_BAUDRATE"]
