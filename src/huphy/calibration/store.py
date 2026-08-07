"""캘리브레이션 파일 읽기·쓰기.

`config/calibration/*.json` <-> `dict[관절이름, MotorCalibration]`

여기 담기는 것은 **로봇을 만져서 알아내는 값**뿐임 — `sign`, `offset_deg`,
`zero_reference`, `limits_deg`. 게인은 같은 모델로 갈면 그대로 쓰는 값이라
`robot.yaml` 에 있음 (이슈 #2).

`limits_deg` 와 `offset_deg` 는 `commission sweep` 이, `zero_reference` 는
`commission zero` 가 적음. `sign` 은 설계가 정한 값이라 쓰는 코드가 없음.


## 관절 이름으로 키를 맞춤

CAN id 는 바뀔 수 있음 (`commissioning.set_can_id`). 관절 자리는 안 바뀜.

`robot.yaml` 도 관절 이름을 키로 쓰므로 두 파일을 나란히 놓고 대조할 수 있음.


## 여기만 씀

제어 경로는 읽기만 함. 쓰기는 캘리브레이션 절차에서만 일어남 — 제어 중에 실측값이
바뀌면 좌표계가 도중에 옮겨감.

저장은 **임시 파일에 쓰고 바꿔치기**함. 도중에 죽으면 원본이 그대로 남음. 실측값을
잃으면 다시 재는 수밖에 없는데, 그건 로봇을 분해해야 하는 작업일 수 있음.
"""

from __future__ import annotations

import json
import logging
import os
import tempfile
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional

from ..motors.base import MotorCalibration

logger = logging.getLogger(__name__)

SCHEMA_VERSION = 2
"""파일 형식 번호.

형식이 바뀌면 올림. 읽을 때 대조해서, 코드가 기대하는 것과 다른 파일을 조용히
읽어 들이지 않게 함 -- 항목 하나가 무시되면 그 관절만 항등변환으로 돎.
"""

ENTRY_KEYS = {"sign", "offset_deg", "zero_reference", "limits_deg"}
FILE_KEYS = {"schema_version", "limb", "note", "motors"}


class CalibrationError(ValueError):
    """캘리브레이션 파일이 읽히지 않거나 앞뒤가 안 맞음."""


def _limits(where: str, raw) -> "tuple | None":
    """`[최소, 최대]` 를 읽음. 없으면 `None` -- **아직 안 잼** 이라는 뜻임.

    "제한 없음" 과 구분해야 함. 값이 없으면 `Motor.is_configured` 가 `False` 라
    제어 진입이 막힘.
    """
    if raw is None:
        return None
    if not isinstance(raw, (list, tuple)) or len(raw) != 2:
        raise CalibrationError(
            f"{where}: limits_deg 는 [최소, 최대] 두 개여야 함 (받은 값 {raw!r})"
        )
    try:
        return (float(raw[0]), float(raw[1]))
    except (TypeError, ValueError) as e:
        raise CalibrationError(f"{where}: limits_deg 가 숫자가 아님 -- {e}") from e


def _entry(where: str, joint: str, data: Mapping) -> MotorCalibration:
    if not isinstance(data, Mapping):
        raise CalibrationError(f"{where}.{joint}: 항목이 사전이어야 함")

    unknown = sorted(set(data) - ENTRY_KEYS)
    if unknown:
        raise CalibrationError(
            f"{where}.{joint}: 모르는 키 {unknown} (가용: {sorted(ENTRY_KEYS)}). "
            f"게인은 robot.yaml 에 있음"
        )

    try:
        sign = float(data.get("sign", 1.0))
        offset = float(data.get("offset_deg", 0.0))
    except (TypeError, ValueError) as e:
        raise CalibrationError(f"{where}.{joint}: 숫자가 아닌 값 -- {e}") from e

    limits = _limits(f"{where}.{joint}", data.get("limits_deg"))

    if sign == 0.0:
        raise CalibrationError(
            f"{where}.{joint}: sign 이 0임. 모든 raw 가 같은 cal 로 뭉개져 "
            f"역변환이 불가능함. +1 또는 -1 이어야 함"
        )

    # motor_id 는 파일에 없음. robot.yaml 과 합칠 때 채워짐 (`attach`).
    try:
        return MotorCalibration(
            motor_id=-1,
            sign=sign,
            offset_deg=offset,
            zero_reference=str(data.get("zero_reference", "")),
            limits_deg=limits,
        )
    except ValueError as e:
        raise CalibrationError(f"{where}.{joint}: {e}") from None


def load(path: "str | Path") -> Dict[str, MotorCalibration]:
    """캘리브레이션 파일을 읽음. 관절 이름 -> `MotorCalibration`.

    `motor_id` 는 `-1` 로 남음. 파일에 없는 값이고, `attach()` 가 `robot.yaml` 의
    모터 목록과 맞춰 채움.
    """
    p = Path(path)
    if not p.is_file():
        raise CalibrationError(f"캘리브레이션 파일이 없음: {p}")

    try:
        data = json.loads(p.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        raise CalibrationError(f"{p}: JSON 을 읽을 수 없음\n{e}") from e

    if not isinstance(data, Mapping):
        raise CalibrationError(f"{p}: 최상위가 사전이어야 함")

    unknown = sorted(set(data) - FILE_KEYS)
    if unknown:
        raise CalibrationError(f"{p}: 모르는 키 {unknown} (가용: {sorted(FILE_KEYS)})")

    version = data.get("schema_version")
    if version != SCHEMA_VERSION:
        raise CalibrationError(
            f"{p}: schema_version 이 {version!r} 임. 이 코드는 {SCHEMA_VERSION} 을 읽음. "
            f"형식이 맞지 않는 파일을 읽으면 항목이 조용히 무시되어 그 관절만 "
            f"항등변환으로 돎"
        )

    motors = data.get("motors")
    if not isinstance(motors, Mapping):
        raise CalibrationError(f"{p}: motors 는 사전이어야 함")

    return {joint: _entry(str(p), joint, e) for joint, e in motors.items()}


def save(
    path: "str | Path",
    calibrations: Mapping[str, MotorCalibration],
    *,
    limb: Optional[str] = None,
    note: str = "",
) -> None:
    """캘리브레이션을 파일로 씀. **덮어씀.**

    임시 파일에 쓰고 바꿔치기하므로, 도중에 죽어도 원본이 남음.

    `motor_id` 는 저장하지 않음. `robot.yaml` 이 가진 값이고, 두 군데 있으면
    어긋날 수 있음.
    """
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)

    payload = {
        "schema_version": SCHEMA_VERSION,
        "limb": limb or p.stem,
        "note": note,
        "motors": {
            joint: {
                "sign": float(c.sign),
                "offset_deg": float(c.offset_deg),
                "zero_reference": c.zero_reference,
                "limits_deg": (
                    None if c.limits_deg is None else [float(v) for v in c.limits_deg]
                ),
            }
            for joint, c in calibrations.items()
        },
    }
    text = json.dumps(payload, ensure_ascii=False, indent=2) + "\n"

    fd, tmp = tempfile.mkstemp(dir=str(p.parent), suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, p)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise

    logger.info("캘리브레이션 저장: %s (%d개 관절)", p, len(calibrations))


def identity(joints: Iterable[str]) -> Dict[str, MotorCalibration]:
    """전부 항등변환인 캘리브레이션. 실측 전 상태임.

    `sign=1, offset=0` 이면 `cal == raw` 라 두 공간이 같은 숫자가 됨. 지금 이 상태라
    두 공간을 섞어 써도 드러나지 않음 (이슈 #2).
    """
    return {j: MotorCalibration(motor_id=-1) for j in joints}


def attach(
    calibrations: Mapping[str, MotorCalibration],
    motors: Mapping[str, "object"],
) -> Dict[int, MotorCalibration]:
    """`robot.yaml` 의 모터 목록과 맞춰 모터 id 로 다시 키를 잡음.

    `motors` 는 `LimbConfig.motors` — 관절 이름 -> `Motor`.

    양쪽 관절 이름이 정확히 같아야 함. 한쪽에만 있으면 에러임 — 관절 하나가 조용히
    항등변환으로 도는 것이 가장 나쁨. sign 이 반대인 관절이 그렇게 되면 목표에서
    **멀어지는 방향**으로 토크가 걸림.
    """
    missing = sorted(set(motors) - set(calibrations))
    extra = sorted(set(calibrations) - set(motors))
    if missing or extra:
        parts = []
        if missing:
            parts.append(f"캘리브레이션에 없는 관절 {missing}")
        if extra:
            parts.append(f"설정에 없는 관절 {extra}")
        raise CalibrationError(
            f"{', '.join(parts)}. robot.yaml 과 캘리브레이션 파일의 관절 이름이 "
            f"같아야 함"
        )

    out: Dict[int, MotorCalibration] = {}
    for joint, motor in motors.items():
        c = calibrations[joint]
        out[motor.id] = MotorCalibration(
            motor_id=motor.id,
            sign=c.sign,
            offset_deg=c.offset_deg,
            zero_reference=c.zero_reference,
            limits_deg=c.limits_deg,
        )
    return out


def unmeasured(calibrations: Mapping[str, MotorCalibration]) -> tuple:
    """아직 실측되지 않은 것으로 보이는 관절 이름들.

    `zero_reference` 가 비어 있으면 영점을 어느 자세에서 잡았는지 모르는 것이고,
    그러면 `offset` 값이 항등이든 아니든 신뢰할 수 없음.

    판정 근거가 `sign`/`offset` 이 아닌 이유: 실측 결과가 우연히 `1.0`/`0.0` 일 수
    있음. 메모는 사람이 적는 것이라 우연히 채워지지 않음.
    """
    return tuple(j for j, c in calibrations.items() if not c.zero_reference.strip())
