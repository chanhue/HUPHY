"""설정의 `model` 문자열을 구현체로 바꿈.

`Motor.model` 이 `"RS02"` 라는 문자열이고 그것을 벤더 모듈이 푸는 것과 같은 자리임.
센서를 갈아 끼우면 **표에 한 줄 늘고 `sensors/<벤더>/` 가 하나 생김.** 위쪽 코드는
그대로임.

벤더 모듈을 여기서 import 하지 않음 -- 만들 때 그때 부름. 안 쓰는 센서의 의존성
(`pyserial` 등)까지 깔려 있어야 설정을 읽을 수 있으면 곤란함.
"""

from __future__ import annotations

from typing import Any, Callable, Dict, Iterable, List

import logging

from .base import Imu


def _baudrate(config: Any, fallback: int) -> int:
    """설정에 적혀 있으면 그것, 없으면 벤더 출하 기본값.

    출하 기본값이 센서마다 달라 `ImuConfig` 에 숫자를 박아 둘 수 없음 -- 한쪽 값을
    기본값으로 두면 다른 쪽 설정에서 이 줄을 생략했을 때 조용히 안 붙음.
    """
    value = getattr(config, "baudrate", None)
    return fallback if value is None else int(value)


def _xsens(config: Any) -> Imu:
    from .xsens import DEFAULT_BAUDRATE, XsensImu  # noqa: PLC0415

    # `output` 계열은 안 씀 -- Xsens 는 패킷에 어떤 항목인지가 들어 있어 설정으로
    # 알려줄 필요가 없음. EBIMU 전용 키가 적혀 있어도 그냥 무시됨.
    return XsensImu(
        config.name, config.port, baudrate=_baudrate(config, DEFAULT_BAUDRATE)
    )


def _ebimu(config: Any) -> Imu:
    from .ebimu import DEFAULT_BAUDRATE, EbimuImu  # noqa: PLC0415

    return EbimuImu(
        config.name,
        config.port,
        baudrate=_baudrate(config, DEFAULT_BAUDRATE),
        output=config.output,
    )


logger = logging.getLogger(__name__)

MODELS: Dict[str, Callable[[Any], Imu]] = {
    "xsens_mti": _xsens,
    "ebimu": _ebimu,
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


def make_imus(configs: Iterable[Any]) -> List[Imu]:
    """여러 IMU 를 만듦. **만들지 못한 것은 빼고 진행함.**

    모르는 `model` 이나 빠진 의존성 때문에 로봇 전체를 못 쓰게 되면, 센서가 고장
    났을 때 안전한 자세로 되돌리는 것조차 막힘. 빠진 것은 경고로 남김.

    아직 열지 않음 — 여는 것은 `ImuGroup.connect()` 가 함.
    """
    out: List[Imu] = []
    for config in configs:
        try:
            out.append(make_imu(config))
        except Exception as e:  # noqa: BLE001 - 센서 하나로 로봇을 막지 않음
            logger.warning("IMU %s 를 만들지 못함 (없이 진행함): %s", config.name, e)
    return out
