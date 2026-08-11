"""설정의 `model` 문자열을 구현체로 바꿈.

`Motor.model` 이 `"RS02"` 라는 문자열이고 그것을 벤더 모듈이 푸는 것과 같은 자리임.
센서를 갈아 끼우면 **표에 한 줄 늘고 `sensors/<벤더>/` 가 하나 생김.** 위쪽 코드는
그대로임.

벤더 모듈을 여기서 import 하지 않음 -- 만들 때 그때 부름. 안 쓰는 센서의 의존성
(`pyserial` 등)까지 깔려 있어야 설정을 읽을 수 있으면 곤란함.
"""

from __future__ import annotations

from typing import Any, Callable, Dict

from .base import Imu


def _xsens(config: Any) -> Imu:
    from .xsens import XsensImu  # noqa: PLC0415

    return XsensImu(config.name, config.port, baudrate=config.baudrate)


MODELS: Dict[str, Callable[[Any], Imu]] = {
    "xsens_mti": _xsens,
}
"""`model` -> 만드는 함수. 키가 곧 `robot.yaml` 에 적는 이름임."""


def make_imu(config: Any) -> Imu:
    """`ImuConfig` 로 IMU 하나를 만듦. 아직 연결하지 않음.

    만드는 것과 여는 것을 나눔 -- 설정이 맞는지는 포트를 열기 전에 알 수 있어야 함.
    """
    factory = MODELS.get(config.model)
    if factory is None:
        raise ValueError(
            f"{config.name}: 모르는 IMU model {config.model!r} "
            f"(가용: {sorted(MODELS)})"
        )
    return factory(config)
