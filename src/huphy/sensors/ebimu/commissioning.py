"""EBIMU 커미셔닝 — 조립할 때 한 번 하는 조작.

`robstride/commissioning.py` 와 같은 자리임. **되돌리기 어려운 것을 제어 루프에서
부를 수 있는 자리에 두지 않으려는 것.**

    imu.py            런타임. 읽기만 함
    commissioning.py  센서 설정을 바꿈. 여기 있음

`huphy-imu` 가 이 함수들을 부름.


## 설정은 센서에 저장됨

보낸 명령이 비휘발성 메모리에 자동 저장됨. **전원을 껐다 켜도 남고, 되돌리려면
반대 명령을 보내야 함.** 그래서 `apply` 가 `--yes` 를 요구함.

읽기(`<cfg>`)는 안전함. 바꾸기 전에 이걸로 지금 설정을 적어 둘 것.


## `<cfg>` 뒤에는 반드시 `>` 를 보내야 함

매뉴얼 6-4-1: `<cfg>` 는 `>` 를 받을 때까지 센서를 정지 상태로 둠. 안 보내면 출력이
멈춘 채로 남아 다음 실행에서 "패킷이 안 옴" 으로 보임. 예외가 나도 보내도록
`finally` 에 둠.


## 부착 방향 검사

정지 상태의 가속도계는 **중력방향을 직접 잼.** 자세에서 계산한 중력방향과 같아야
하고, 다르면 센서가 예상과 다른 방향으로 붙어 있는 것임.

한 축만 기울이면 못 잡는 어긋남이 있으므로 **두 축을 동시에** 기울여 놓고 잴 것.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

from ..base import ImuState
from . import commands

logger = logging.getLogger(__name__)

DRAIN_S = 0.4
"""명령 하나를 보내고 응답을 모으는 시간."""

CONFIG_DRAIN_S = 2.0
"""`<cfg>` 는 항목이 많아 더 걸림."""

LEVEL_TOLERANCE = 0.05
"""부착 검사 통과 기준. 중력 단위벡터 성분 차이.

0.05 는 약 3도임. 손으로 놓은 자세의 미세한 움직임과 센서 노이즈를 흡수하되,
축이 뒤바뀐 경우(차이 1.0 이상)나 부호가 반대인 경우는 확실히 걸러냄.
"""

MIN_TILT = 0.15
"""검사에 필요한 최소 기울기. 중력의 수평 성분 크기.

수평으로 놓고 재면 두 축이 다 0이라 **부착이 틀려도 통과함.** 약 9도 이상
기울어져 있어야 의미가 있고, 두 축 다 기울여야 함.
"""


def drain(port: Any, seconds: float = DRAIN_S) -> str:
    """그동안 들어온 것을 전부 문자열로 모음."""
    deadline = time.monotonic() + seconds
    chunks: List[str] = []
    while time.monotonic() < deadline:
        try:
            raw = port.readline()
        except Exception as e:
            logger.debug("읽기 실패 (무시함): %s", e)
            break
        if raw:
            chunks.append(raw.decode("utf-8", errors="ignore"))
    return "".join(chunks)


def strip_packets(text: str) -> str:
    """응답에서 데이터 줄을 걸러냄.

    센서는 설정 명령을 받아도 **출력을 멈추지 않음.** 그래서 응답을 모으는 동안
    측정 패킷이 같이 딸려 옴 -- 100Hz 면 0.4초에 40줄임. 그대로 보여주면 정작
    `<ok>` 가 그 사이에 묻힘.
    """
    lines = [
        line.strip()
        for line in text.replace("\r", "\n").split("\n")
        if line.strip() and not line.strip().startswith("*")
    ]
    return " ".join(lines)


def send(port: Any, command: str, *, wait_s: float = DRAIN_S) -> str:
    """명령 하나를 보내고 응답을 돌려줌. 데이터 줄은 걸러냄.

    **위험 명령인지 여기서 안 봄.** 그 판단은 호출부가 함 -- 이 함수는 `apply` 와
    대화형 입력 양쪽에서 쓰이고, 확인을 받는 방식이 서로 다름.
    """
    port.write(command.encode("ascii"))
    port.flush()
    return strip_packets(drain(port, wait_s))


def read_settings(port: Any) -> Dict[str, str]:
    """센서에 저장된 설정을 읽음. `{명령이름: 값}`.

    **설정을 바꾸지 않음.** `>` 를 보내 출력을 되살리는 것까지 여기서 함.
    """
    try:
        port.reset_input_buffer()
    except Exception as e:
        logger.debug("입력 버퍼 비우기 실패 (무시함): %s", e)

    port.write(commands.QUERY_CONFIG.encode("ascii"))
    port.flush()
    try:
        text = strip_packets(drain(port, CONFIG_DRAIN_S))
    finally:
        # 안 보내면 센서가 멈춘 채로 남음. 예외가 나도 반드시 보냄.
        port.write(commands.CONFIG_RESUME.encode("ascii"))
        port.flush()
        drain(port, 0.5)
    return commands.parse_config(text)


# ---------------------------------------------------------------------------
# 설정과 대조
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Mismatch:
    """센서와 설정 파일이 다른 항목 하나."""

    command: str
    """센서를 맞추려면 보낼 명령."""

    now: str
    """센서가 지금 그렇게 되어 있는 상태. 사람이 읽는 문장."""

    wanted: str
    """설정 파일이 요구하는 상태."""


def _setting_of(command: str) -> Tuple[str, str]:
    """`<sog1>` -> `("sog", "1")`."""
    body = command.strip().lstrip("<").rstrip(">")
    index = len(body)
    while index > 0 and (body[index - 1].isdigit() or body[index - 1] in "-."):
        index -= 1
    return body[:index], body[index:]


def _describe(key: str, value: str) -> str:
    detail = commands.SETTING_DETAIL.get((key, value))
    if detail:
        return detail
    if key == "sor":
        try:
            return f"{value}ms ({1000.0 / float(value):.0f}Hz)"
        except (ValueError, ZeroDivisionError):
            return f"주기 {value}"
    return "끔" if value == "0" else f"{key}={value}"


def compare(settings: Dict[str, str], wanted: Sequence[str]) -> List[Mismatch]:
    """센서 설정과 보내려는 명령을 대조함. 다른 것만 돌려줌.

    빈 목록이면 센서가 이미 설정 파일대로 되어 있는 것임. 그때 `apply` 는 아무것도
    보내지 않음 -- 같은 값을 다시 쓰는 것도 비휘발성 메모리에 쓰는 일임.
    """
    out: List[Mismatch] = []
    for command in wanted:
        key, value = _setting_of(command)
        current = settings.get(key)
        if current == value:
            continue
        out.append(
            Mismatch(
                command=command,
                now=_describe(key, current) if current is not None else "모름",
                wanted=_describe(key, value),
            )
        )
    return out


def apply(port: Any, mismatches: Sequence[Mismatch]) -> List[Tuple[str, str]]:
    """다른 항목만 센서에 보냄. `(명령, 응답)` 목록을 돌려줌.

    **호출 전에 사람 확인을 받을 것.** 여기서는 안 물어봄 -- 확인을 받는 방식이
    터미널마다 다르고, 이 함수는 그것을 모르는 편이 나음.
    """
    out: List[Tuple[str, str]] = []
    for item in mismatches:
        response = send(port, item.command)
        out.append((item.command, response.strip()))
        logger.info("%s -> %s", item.command, response.strip() or "(응답 없음)")
    return out


# ---------------------------------------------------------------------------
# 부착 방향
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class MountCheck:
    """부착 방향 검사 결과."""

    from_attitude: Tuple[float, float, float]
    """자세에서 계산한 중력방향. `ImuState.gravity` 그대로임.

    정책이 쓰는 것과 **같은 값**임 -- 여기서 따로 계산하면 검사는 통과하는데 로봇은
    틀리는 경우가 생김.
    """

    from_accel: Tuple[float, float, float]
    """가속도계가 직접 잰 중력방향. 정규화하고 부호를 맞춘 값임."""

    error: float
    """두 벡터의 최대 성분 차이."""

    tilt: float
    """기울기. 중력의 수평 성분 크기."""

    accel_sign: int
    """가속도계 부호 규약. `+1` 이면 중력과 같은 방향, `-1` 이면 반대.

    벤더마다 다름 -- 정지 상태에서 "센서에 작용하는 힘" 을 재느냐 "센서가 받는
    반작용" 을 재느냐임. 어느 쪽이든 자세와의 대조에는 영향 없어서, 맞는 쪽으로
    맞춰 놓고 어느 쪽이었는지만 알려줌.
    """

    @property
    def tilted_enough(self) -> bool:
        """검사가 의미 있을 만큼 기울어져 있는지."""
        return self.tilt >= MIN_TILT

    @property
    def ok(self) -> bool:
        return self.tilted_enough and self.error <= LEVEL_TOLERANCE


def check_mount(state: ImuState) -> MountCheck:
    """자세와 가속도계를 대조함. **정지 상태에서 부를 것.**

    움직이는 중이면 가속도계가 중력 말고 다른 것도 재므로 결과가 의미 없음.

    가속도계 부호 규약을 모르므로 두 부호를 다 계산해 **가까운 쪽을 고름.** 어느
    쪽이 맞았는지는 결과에 담아 사람이 볼 수 있게 함.

    센서가 자세를 어떤 형식으로 주는지는 안 봄 -- 벤더 모듈이 만들어 둔 `gravity`
    를 그대로 씀. 그래서 이 검사는 EBIMU 든 Xsens 든 그대로 돎.
    """
    expected = state.gravity

    norm = sum(v * v for v in state.accel_mps2) ** 0.5
    if norm < 1e-6:
        measured = (0.0, 0.0, 0.0)
        sign = 1
    else:
        unit = tuple(v / norm for v in state.accel_mps2)
        positive = max(abs(a - b) for a, b in zip(expected, unit))
        negative = max(abs(a + b) for a, b in zip(expected, unit))
        sign = 1 if positive <= negative else -1
        measured = tuple(sign * v for v in unit)

    return MountCheck(
        from_attitude=tuple(round(v, 4) for v in expected),
        from_accel=tuple(round(v, 4) for v in measured),
        error=max(abs(a - b) for a, b in zip(expected, measured)),
        tilt=(expected[0] ** 2 + expected[1] ** 2) ** 0.5,
        accel_sign=sign,
    )


def sample(imu: Any, *, seconds: float = 1.0) -> Optional[ImuState]:
    """값이 들어올 때까지 기다렸다 최신 상태 하나를 냄. 없으면 `None`."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        state = imu.read()
        if state.is_valid:
            return state
        time.sleep(0.02)
    return None
