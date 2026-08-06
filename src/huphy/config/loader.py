"""`robot.yaml` 을 읽어 `RobotConfig` 로 만듦.

읽기만 함. 이 파일은 모터도 CAN 도 모름.


## 모르는 키를 에러로 냄

YAML 은 오타를 조용히 삼킴. `contorl_hz: 200` 이라고 쓰면 그 줄이 무시되고 기본값
100Hz 로 돕니다 — 설정을 고쳤는데 아무것도 안 바뀌는 상황이 됨.

찾기 어려운 종류의 문제라 **모르는 키가 있으면 멈춤.** 오타는 대부분 여기서 걸림.


## 기본값을 여기 두지 않음

기본값은 전부 `schema.py` 의 dataclass 에 있음. 두 군데 있으면 어느 쪽이 실제로
쓰이는지 알 수 없음.

이 파일이 하는 것은 **YAML 에 있는 키만 골라 넘기는 것**임. 없는 키는 넘기지 않고,
그러면 dataclass 의 기본값이 쓰임.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional

from ..motors.base import Gains, Motor
from .schema import LimbConfig, RobotConfig, SafetyConfig, TelemetryConfig

ROBOT_KEYS = {"name", "limbs", "safety", "telemetry"}
LIMB_KEYS = {
    "kind", "side", "channel", "interface", "control_hz", "calibration", "motors",
}
MOTOR_KEYS = {"id", "model", "limits_deg", "kp", "kd"}
SAFETY_KEYS = {"command_margin_deg", "max_delta_deg", "enforce_limits"}
TELEMETRY_KEYS = {"host", "port", "csv_path", "csv_flush_every"}


class ConfigError(ValueError):
    """설정 파일이 읽히지 않거나 앞뒤가 안 맞음."""


def _check_keys(where: str, data: Mapping[str, Any], allowed: Iterable[str]) -> None:
    unknown = sorted(set(data) - set(allowed))
    if unknown:
        raise ConfigError(
            f"{where}: 모르는 키 {unknown} (가용: {sorted(allowed)}). "
            f"오타이거나 지원하지 않는 설정임"
        )


def _pick(data: Mapping[str, Any], keys: Iterable[str]) -> Dict[str, Any]:
    """있는 키만 골라냄. 없는 것은 넘기지 않아 dataclass 기본값이 쓰이게 함."""
    return {k: data[k] for k in keys if k in data}


def _limits(where: str, raw: Any) -> Optional[tuple]:
    if raw is None:
        return None
    if not isinstance(raw, (list, tuple)) or len(raw) != 2:
        raise ConfigError(
            f"{where}: limits_deg 는 [최소, 최대] 두 개여야 함 (받은 값 {raw!r})"
        )
    return (float(raw[0]), float(raw[1]))


def _motor(where: str, joint: str, data: Mapping[str, Any]) -> Motor:
    if not isinstance(data, Mapping):
        raise ConfigError(f"{where}.{joint}: 항목이 사전이어야 함 (받은 값 {data!r})")
    _check_keys(f"{where}.{joint}", data, MOTOR_KEYS)

    for required in ("id", "model"):
        if required not in data:
            raise ConfigError(f"{where}.{joint}: {required} 항목이 없음")

    # _limits 는 자기 위치를 이미 붙이므로 try 밖에서 부름. 안에서 부르면 아래
    # 핸들러가 같은 위치를 한 번 더 붙임.
    limits = _limits(f"{where}.{joint}", data.get("limits_deg"))
    try:
        return Motor(
            id=int(data["id"]),
            model=str(data["model"]),
            limits_deg=limits,
            gains=Gains(kp=float(data.get("kp", 0.0)), kd=float(data.get("kd", 0.0))),
        )
    except (TypeError, ValueError) as e:
        raise ConfigError(f"{where}.{joint}: {e}") from e


def _limb(name: str, data: Mapping[str, Any], *, base_dir: Path) -> LimbConfig:
    where = f"limbs.{name}"
    if not isinstance(data, Mapping):
        raise ConfigError(f"{where}: 항목이 사전이어야 함")
    _check_keys(where, data, LIMB_KEYS)

    if "kind" not in data:
        raise ConfigError(f"{where}: kind 가 없음 (leg, arm 등)")

    motors_raw = data.get("motors") or {}
    if isinstance(motors_raw, list):
        raise ConfigError(
            f"{where}.motors: 목록이 아니라 사전이어야 함. "
            f"관절 이름을 키로 씀 -- knee: {{id: 10, model: RS02}}"
        )
    if not isinstance(motors_raw, Mapping):
        raise ConfigError(f"{where}.motors: 사전이어야 함 (받은 값 {type(motors_raw).__name__})")
    if not motors_raw:
        raise ConfigError(f"{where}: 모터가 하나도 없음")

    calibration = data.get("calibration")
    fields = _pick(data, ("kind", "side", "channel", "interface", "control_hz"))

    motors = {j: _motor(f"{where}.motors", j, m) for j, m in motors_raw.items()}
    try:
        return LimbConfig(
            name=name,
            motors=motors,
            calibration_path=(base_dir / calibration) if calibration else None,
            **fields,
        )
    except ConfigError:
        raise
    except (TypeError, ValueError) as e:
        # LimbConfig 의 메시지는 이미 팔다리 이름으로 시작하므로 그대로 씀.
        raise ConfigError(str(e)) from e


def load_robot(path: "str | Path") -> RobotConfig:
    """`robot.yaml` 을 읽음.

    상대 경로(`calibration:`)는 **이 파일이 있는 폴더 기준**으로 풀림. 실행 위치가
    달라져도 같은 파일을 가리키게 하려는 것임.
    """
    try:
        import yaml  # noqa: PLC0415
    except ImportError as e:
        raise ImportError("설정을 읽으려면 PyYAML 이 필요함. `pip install PyYAML`") from e

    p = Path(path)
    if not p.is_file():
        raise ConfigError(f"설정 파일이 없음: {p}")

    try:
        data = yaml.safe_load(p.read_text(encoding="utf-8"))
    except yaml.YAMLError as e:
        raise ConfigError(f"{p}: YAML 을 읽을 수 없음\n{e}") from e

    if not isinstance(data, Mapping):
        raise ConfigError(f"{p}: 최상위가 사전이어야 함 (받은 값 {type(data).__name__})")
    _check_keys(str(p), data, ROBOT_KEYS)

    limbs_raw = data.get("limbs") or {}
    if not isinstance(limbs_raw, Mapping):
        raise ConfigError(f"{p}: limbs 는 사전이어야 함")
    if not limbs_raw:
        raise ConfigError(f"{p}: limbs 가 비어 있음")

    safety_raw = data.get("safety") or {}
    telemetry_raw = data.get("telemetry") or {}
    _check_keys(f"{p}: safety", safety_raw, SAFETY_KEYS)
    _check_keys(f"{p}: telemetry", telemetry_raw, TELEMETRY_KEYS)

    try:
        return RobotConfig(
            name=str(data.get("name") or p.stem),
            limbs={n: _limb(n, d, base_dir=p.parent) for n, d in limbs_raw.items()},
            safety=SafetyConfig(**_pick(safety_raw, SAFETY_KEYS)),
            telemetry=TelemetryConfig(**_pick(telemetry_raw, TELEMETRY_KEYS)),
            source_path=p,
        )
    except ConfigError:
        raise
    except (TypeError, ValueError) as e:
        raise ConfigError(f"{p}: {e}") from e
