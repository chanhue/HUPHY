"""설정 계층.

    schema.py   설정 자료형. 사람이 적는 것의 구조
    loader.py   robot.yaml -> RobotConfig

`schema` 는 순수 자료형이라 의존이 없음. `loader` 는 PyYAML 을 함수 안에서 import함.
"""

from . import schema
from .loader import ConfigError, load_robot
from .schema import ImuConfig, LimbConfig, RobotConfig, SafetyConfig, TelemetryConfig

__all__ = [
    "schema",
    "load_robot",
    "ConfigError",
    "RobotConfig",
    "LimbConfig",
    "ImuConfig",
    "SafetyConfig",
    "TelemetryConfig",
]
