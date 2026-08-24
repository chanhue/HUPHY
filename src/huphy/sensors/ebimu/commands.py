"""EBIMU 사양 테이블 — 매뉴얼에서 오는 값.

실측이 아님. 센서를 바꾸지 않는 한 변하지 않음. `robstride/tables.py` 와 같은
자리임.

출처: E2BOX EBIMU-9DOFV5 매뉴얼 rev3


## 명령 형식

    <명령값>        예: <sof2>  <sog1>  <sor10>

전부 ASCII 이고 센서가 `<ok>` 로 답함. **보낸 설정은 센서 비휘발성 메모리에 자동
저장됨** -- 전원을 껐다 켜도 남고, 되돌리려면 반대 명령을 보내야 함.


## 출력 순서는 명령 순서로 고정임

    *<sof><sog><soa><som><sod><sot><sots>(CR)(LF)

켜 놓은 항목만 이 순서대로 이어 붙음. **패킷에는 무엇이 켜져 있는지가 안 적혀
있음** -- 숫자만 옴. 그래서 설정을 `robot.yaml` 에 적어 두고 그것이 기준이 됨
(`ImuConfig.output`).

필드 수가 같은 조합이 여럿이라 개수로 추정할 수 없음. 예를 들어 10개는
`quat+gyro+accel` 일 수도 `euler+gyro+accel+temp` 일 수도 있음.


## 값이 뜻을 바꾸는 명령이 둘 있음

    <soa1|2|3>   중력 포함 / 제거 Local / 제거 Global 가속도
    <sod1|2>     Local / Global 거리

필드 수가 같아서 패킷으로는 구분이 안 됨. 그래서 `accel_mode`, `dist_mode` 를 따로
적음.
"""

from __future__ import annotations

import re
from typing import Dict, Iterable, List, Sequence, Tuple

# ---------------------------------------------------------------------------
# 출력 블록
# ---------------------------------------------------------------------------
BLOCK_SIZE: Dict[str, int] = {
    "euler": 3,
    "quat": 4,
    "gyro": 3,
    "accel": 3,
    "vel": 3,
    "mag": 3,
    "dist": 3,
    "temp": 1,
    "time": 1,
}
"""블록별 필드 수. `output` 합계가 곧 한 줄의 숫자 개수임."""

BLOCK_ORDER: Tuple[str, ...] = (
    "euler", "quat", "gyro", "accel", "vel", "mag", "dist", "temp", "time",
)
"""패킷에 나오는 순서. `output` 은 이 순서를 지켜야 함.

`euler`/`quat` 은 자세 자리 하나를 나눠 쓰고, `accel`/`vel` 은 `soa` 자리를 나눠
씀. 그래서 각 쌍은 동시에 나올 수 없음.
"""

EXCLUSIVE: Tuple[Tuple[str, str], ...] = (("euler", "quat"), ("accel", "vel"))
"""같은 명령을 나눠 써서 동시에 켤 수 없는 쌍."""

ATTITUDE = ("euler", "quat")
"""자세 블록. **끌 수 없음** -- 둘 중 하나가 반드시 나옴."""


def field_count(output: Sequence[str]) -> int:
    """이 구성에서 한 줄에 오는 숫자 개수."""
    return sum(BLOCK_SIZE[b] for b in output)


def validate(output: Sequence[str]) -> None:
    """`output` 이 센서가 낼 수 있는 구성인지. 아니면 `ValueError`.

    실행 전에 걸러야 함 -- 필드 수만 맞고 순서가 틀린 구성은 값이 엉뚱한 자리로
    들어가는데 숫자가 그럴듯해서 실물에서 안 잡힘.
    """
    unknown = [b for b in output if b not in BLOCK_SIZE]
    if unknown:
        raise ValueError(
            f"모르는 출력 항목 {unknown} (가용: {list(BLOCK_ORDER)})"
        )

    duplicated = sorted({b for b in output if list(output).count(b) > 1})
    if duplicated:
        raise ValueError(f"출력 항목이 중복됨 {duplicated}")

    for a, b in EXCLUSIVE:
        if a in output and b in output:
            raise ValueError(
                f"{a!r} 과 {b!r} 은 같은 명령을 나눠 써서 동시에 낼 수 없음"
            )

    if not any(b in output for b in ATTITUDE):
        raise ValueError(
            f"자세는 끌 수 없음. {ATTITUDE[0]!r} 또는 {ATTITUDE[1]!r} 이 있어야 함"
        )

    rank = {b: i for i, b in enumerate(BLOCK_ORDER)}
    if list(output) != sorted(output, key=lambda b: rank[b]):
        raise ValueError(
            f"출력 항목은 패킷 순서대로 적어야 함. "
            f"받은 것 {list(output)}, 이 순서로 {sorted(output, key=lambda b: rank[b])}"
        )


# ---------------------------------------------------------------------------
# 명령
# ---------------------------------------------------------------------------
ATTITUDE_VALUE: Dict[str, str] = {"euler": "1", "quat": "2"}
"""`<sof_>` 값. 자세는 끌 수 없어 0이 없음."""

ACCEL_VALUE: Dict[str, str] = {"gravity": "1", "local": "2", "global": "3"}
"""`<soa_>` 값 중 가속도 쪽.

    gravity   중력성분 포함. 정지 상태에서 중력방향이 그대로 읽힘
    local     중력 제거, 센서 축 기준
    global    중력 제거, 동서남북 기준

**`gravity` 가 기본임.** 부착 방향 검사가 중력을 재는 것에 기대고 있음.
"""

VEL_VALUE: Dict[str, str] = {"local": "4", "global": "5"}
"""`<soa_>` 값 중 속도 쪽. 가속도와 같은 자리라 하나만 고름."""

DIST_VALUE: Dict[str, str] = {"local": "1", "global": "2"}
"""`<sod_>` 값. 다리에 붙은 센서면 `local` 임."""

BAUD_DEFAULT = 115200
"""공장 출하 보레이트. 설정에 `baudrate` 가 없을 때 씀."""

BAUD_VALUE: Dict[int, str] = {
    9600: "1", 19200: "2", 38400: "3", 57600: "4",
    115200: "5", 230400: "6", 460800: "7", 921600: "8",
}
"""`<sb_>` 값. **바꾸는 순간 통신이 끊김** -- 보낸 뒤 새 보레이트로 다시 열어야 함."""

RATE_MIN_MS = 1
RATE_MAX_MS = 1000
"""`<sor_>` 는 1~1000. 출력 주기 = 1ms x 값. `10` 이면 100Hz."""

QUERY_CONFIG = "<cfg>"
"""현재 설정을 명령어별로 전부 출력함.

**응답 뒤 센서가 정지 상태로 머묾.** `>` 를 보내야 출력이 다시 시작됨. 읽기만 하는
명령이라 설정은 안 바뀜.
"""

CONFIG_RESUME = ">"
"""`<cfg>` 뒤에 반드시 보내야 하는 것. 안 보내면 센서가 계속 멈춰 있음."""

DANGEROUS: Tuple[str, ...] = ("<lf", "<sb", "<reset", "<stop", "<pons")
"""확인 없이 보내면 안 되는 것.

    <lf>     공장초기화. 캘리브레이션 결과까지 사라짐
    <sb_>    보레이트 변경. 보낸 순간 통신이 끊김
    <reset>  센서 리셋
    <stop>   출력 중지
    <pons0>  전원 인가시 작동 안 함
"""

CALIBRATION: Tuple[str, ...] = ("<cg", "<ca", "<cm", "<cn", "<+cn")
"""센서를 물리적으로 움직여야 하는 캘리브레이션. 사람이 옆에 있어야 함."""


def rate_command(rate_hz: float) -> str:
    """`<sor_>` 를 만듦. 출력 주기 = 1ms x 값.

    100Hz 가 정확히 10ms 로 떨어지는 것처럼 나누어떨어져야 함 -- 안 떨어지면
    실제 주기가 요청과 달라지는데 조용히 그렇게 됨.
    """
    if rate_hz <= 0:
        raise ValueError(f"rate_hz 는 0보다 커야 함 (받은 값 {rate_hz})")
    period_ms = 1000.0 / float(rate_hz)
    steps = round(period_ms)
    if abs(period_ms - steps) > 1e-6:
        raise ValueError(
            f"rate_hz {rate_hz} 는 1ms 단위로 안 떨어짐 (주기 {period_ms:.3f}ms). "
            f"센서는 1ms 배수만 됨 -- 100, 200, 500 같은 값을 쓸 것"
        )
    if not RATE_MIN_MS <= steps <= RATE_MAX_MS:
        raise ValueError(
            f"출력 주기 {steps}ms 가 범위를 벗어남 ({RATE_MIN_MS}~{RATE_MAX_MS})"
        )
    return f"<sor{steps}>"


def output_commands(
    output: Sequence[str],
    *,
    accel_mode: str = "gravity",
    dist_mode: str = "local",
    rate_hz: float = 100.0,
) -> List[str]:
    """이 구성을 만드는 명령들. **켜는 것과 끄는 것을 다 냄.**

    안 쓰는 항목을 명시적으로 끄는 이유: 센서에 예전 설정이 남아 있으면 필드가 더
    붙어 나와 파싱이 통째로 밀림. "적은 것만 나오게" 하려면 나머지를 꺼야 함.
    """
    validate(output)

    if accel_mode not in ACCEL_VALUE:
        raise ValueError(
            f"모르는 accel_mode {accel_mode!r} (가용: {sorted(ACCEL_VALUE)})"
        )
    if dist_mode not in DIST_VALUE:
        raise ValueError(
            f"모르는 dist_mode {dist_mode!r} (가용: {sorted(DIST_VALUE)})"
        )

    attitude = "quat" if "quat" in output else "euler"
    out = [f"<sof{ATTITUDE_VALUE[attitude]}>"]

    out.append(f"<sog{1 if 'gyro' in output else 0}>")

    if "accel" in output:
        out.append(f"<soa{ACCEL_VALUE[accel_mode]}>")
    elif "vel" in output:
        out.append(f"<soa{VEL_VALUE[dist_mode]}>")
    else:
        out.append("<soa0>")

    out.append(f"<som{1 if 'mag' in output else 0}>")
    out.append(f"<sod{DIST_VALUE[dist_mode] if 'dist' in output else 0}>")
    out.append(f"<sot{1 if 'temp' in output else 0}>")
    out.append(f"<sots{1 if 'time' in output else 0}>")
    out.append(rate_command(rate_hz))
    return out


# ---------------------------------------------------------------------------
# <cfg> 응답 읽기
# ---------------------------------------------------------------------------
CONFIG_TOKEN = re.compile(r"<([a-z_+]+?)([-0-9.]*)>")
"""`<sog1>` `<sor10>` 같은 토큰을 뽑음."""

CONFIG_BARE = re.compile(r"\b(sof|sog|soa|som|sod|sots|sot)\s*[:=]?\s*([0-9])\b")
"""괄호 없이 답하는 펌웨어를 위한 대비책. 토큰이 하나도 안 잡혔을 때만 씀."""

SETTING_TO_BLOCK: Tuple[Tuple[str, Dict[str, str]], ...] = (
    ("sof", {"1": "euler", "2": "quat"}),
    ("sog", {"1": "gyro"}),
    ("soa", {"1": "accel", "2": "accel", "3": "accel", "4": "vel", "5": "vel"}),
    ("som", {"1": "mag"}),
    ("sod", {"1": "dist", "2": "dist"}),
    ("sot", {"1": "temp"}),
    ("sots", {"1": "time"}),
)
"""설정값 -> 어떤 블록이 켜지는지. 순서가 곧 패킷 순서임."""

SETTING_DETAIL: Dict[Tuple[str, str], str] = {
    ("sof", "1"): "Euler Angles",
    ("sof", "2"): "Quaternion",
    ("soa", "1"): "중력성분 포함 가속도",
    ("soa", "2"): "중력성분 제거 Local 가속도",
    ("soa", "3"): "중력성분 제거 Global 가속도",
    ("soa", "4"): "Local 속도",
    ("soa", "5"): "Global 속도",
    ("sod", "1"): "Local 거리",
    ("sod", "2"): "Global 거리",
    ("sog", "1"): "각속도",
    ("som", "1"): "지자기",
    ("sot", "1"): "온도",
    ("sots", "1"): "타임스탬프",
}
"""사람에게 보여줄 설명. 값에 따라 뜻이 달라지는 것이 있어 쌍으로 키를 잡음."""


def parse_config(text: str) -> Dict[str, str]:
    """`<cfg>` 응답에서 `{명령이름: 값}` 을 뽑음."""
    found = dict(CONFIG_TOKEN.findall(text))
    if not any(key in found for key, _ in SETTING_TO_BLOCK):
        found.update(dict(CONFIG_BARE.findall(text)))
    return found


def output_from_config(settings: Dict[str, str]) -> List[str]:
    """설정에서 지금 켜져 있는 출력 항목을 만듦. 패킷 순서로 나옴.

    자세가 안 보이면 `euler` 로 봄 -- 끌 수 없는 항목이고 공장 기본값이 그것임.
    """
    out: List[str] = []
    for key, mapping in SETTING_TO_BLOCK:
        value = settings.get(key)
        if value is None:
            if key == "sof":
                out.append("euler")
            continue
        block = mapping.get(value)
        if block:
            out.append(block)
    return out


def describe(settings: Dict[str, str]) -> List[Tuple[str, str]]:
    """`(명령, 설명)` 목록. 사람에게 보여주는 용도임."""
    rows: List[Tuple[str, str]] = []
    for key, _ in SETTING_TO_BLOCK:
        value = settings.get(key)
        if value is None:
            continue
        detail = SETTING_DETAIL.get((key, value), "")
        if not detail:
            continue
        rows.append((f"<{key}{value}>", detail))
    if "sor" in settings:
        period = settings["sor"]
        try:
            hz = 1000.0 / float(period)
        except (ValueError, ZeroDivisionError):
            rows.append((f"<sor{period}>", "출력 주기"))
        else:
            rows.append((f"<sor{period}>", f"{period}ms ({hz:.0f}Hz)"))
    return rows


def is_dangerous(command: str) -> bool:
    """확인을 받아야 하는 명령인지."""
    text = command.strip().lower()
    return text.startswith(DANGEROUS) or text.startswith(CALIBRATION)


def joined(commands: Iterable[str]) -> str:
    return " ".join(commands)
