"""로봇 계약 — 관절 이름으로 말하는 계층.

여기부터 위는 **관절**을 앎. 아래는 **모터 id 와 raw 각도**만 앎.

    control/    "무릎 30도"          관절 이름, cal 공간
    robots/     <-- 경계 -->
    motors/     "m10 에 62.79도"     모터 id, raw 공간


## 이 경계에서 일어나는 일

    관절 이름 -> 모터 id       robot.yaml 의 매핑
    cal -> raw                캘리브레이션의 sign/offset
    발목 pitch/roll -> a1/a2   기구학
    한계·점프 검사             safety.guards (cal 공간)

**넷 다 여기서만 일어남.** 위 계층은 관절 각도만 다루고, 아래 계층은 바이트만 다룸.


## 계약이 요구하는 것

    joint_names            이 로봇이 아는 관절
    observation_features   무엇을 관찰할 수 있는지
    action_features        무엇을 명령할 수 있는지

이름이 아니라 **구조**를 내놓게 한 이유: 텔레메트리와 기록이 이 목록으로 열을
만듦. 실행해 보고 나서야 필드를 아는 구조면 로그 형식이 매 실행마다 달라짐.


## 계산·전송·수거를 나눔

    build_commands(action)   계산만. CAN 을 쓰지 않음
    send(commands)           전송만
    collect()                수거만
    send_action(action)      셋을 합친 것

버스가 둘일 때 "왼다리 계산 -> 오른다리 계산 -> 왼다리 전송 -> 오른다리 전송 ->
수거" 순서를 짜려면 나뉘어 있어야 함 (이슈 #10). 다리 하나뿐이면 `send_action`
하나로 충분함.
"""

from __future__ import annotations

import abc
from typing import Any, Dict, Mapping, Tuple

Action = Mapping[str, float]
"""관절 이름 -> 목표 각도 (cal 공간, 도)."""

Observation = Dict[str, Any]
"""필드 이름 -> 값. 구조는 `observation_features` 와 같음."""


class Robot(abc.ABC):
    """관절 이름으로 말하는 로봇 하나.

    다리, 팔, 나중에는 양다리를 묶은 것도 이 계약을 채움.
    """

    name: str = "robot"
    """종류 이름. 개체 이름은 `id` 임."""

    id: str = ""
    """개체 이름. 같은 종류가 둘 이상일 때 구분함 (`right_leg`, `left_leg`)."""

    def __str__(self) -> str:
        return f"{self.id or self.name} ({type(self).__name__})"

    # ---- 구조 -------------------------------------------------------------
    @property
    @abc.abstractmethod
    def joint_names(self) -> Tuple[str, ...]:
        """이 로봇이 아는 관절 이름들. 순서가 의미를 가짐."""

    @property
    @abc.abstractmethod
    def observation_features(self) -> Dict[str, type]:
        """관찰 필드 이름 -> 타입.

        텔레메트리와 기록이 이 목록으로 열을 만듦. **실행 전에 알 수 있어야 함.**
        """

    @property
    @abc.abstractmethod
    def action_features(self) -> Dict[str, type]:
        """명령 필드 이름 -> 타입."""

    # ---- 수명 -------------------------------------------------------------
    @property
    @abc.abstractmethod
    def is_connected(self) -> bool:
        ...

    @abc.abstractmethod
    def connect(self) -> None:
        ...

    @abc.abstractmethod
    def disconnect(self) -> None:
        """토크를 끊고 정리함. **여러 번 불려도 안전해야 함.**"""

    def __enter__(self) -> "Robot":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """예외로 빠져나가도 토크가 끊기도록 보장함.

        이게 없으면 제어 중 예외가 났을 때 모터가 마지막 명령을 계속 유지함 —
        사람이 전원을 뽑을 때까지 다리가 힘을 주고 있음.
        """
        self.disconnect()

    # ---- 캘리브레이션 -----------------------------------------------------
    @property
    @abc.abstractmethod
    def is_calibrated(self) -> bool:
        """제어에 쓸 만큼 실측값이 채워졌는지.

        계약에 둔 이유: 이게 없으면 각 로봇이 제멋대로 판정하게 되고, 호출부가
        "이 로봇은 준비됐나" 를 물어볼 공통 방법이 없어짐. 미실측 상태로 토크를
        넣는 것이 가장 위험함.
        """

    @abc.abstractmethod
    def calibrate(self) -> None:
        """실측값을 다시 읽어 들임.

        재는 절차 자체는 여기 없음 — 그건 사람이 하는 일이고
        `motors/robstride/commissioning.py` 와 `scripts/` 가 도움.
        """

    # ---- 관찰 -------------------------------------------------------------
    @abc.abstractmethod
    def get_observation(self) -> Observation:
        """지금 상태. 구조는 `observation_features` 와 같음.

        각도는 **cal 공간**임.
        """

    # ---- 명령 -------------------------------------------------------------
    @abc.abstractmethod
    def build_commands(self, action: Action) -> Dict[int, Any]:
        """명령을 계산함. **CAN 을 쓰지 않음.**

        반환: 모터 id -> 벤더 명령. 무엇이 잘렸는지는 로봇의 카운터에 쌓임.
        """

    @abc.abstractmethod
    def send(self, commands: Mapping[int, Any]) -> int:
        """계산된 명령을 보냄. **수거하지 않음.** 보낸 개수를 반환함."""

    @abc.abstractmethod
    def collect(self) -> Tuple[int, ...]:
        """응답을 수거해 상태를 갱신함. **응답이 없었던 모터 id** 를 반환함."""

    @abc.abstractmethod
    def refresh(self) -> Tuple[int, ...]:
        """명령하지 않고 상태만 읽음. 응답이 없었던 모터 id 를 반환함.

        **읽기 전용 통신이 아닐 수 있음.** MIT 프로토콜에는 상태 읽기 명령이 따로
        없어서, 힘이 나가지 않는 명령을 보내고 그 응답을 받는 방식임.

        관찰 모드가 이것을 씀. 아무것도 보내지 않으면 아무것도 오지 않음.
        """

    def send_action(self, action: Action) -> Dict[str, float]:
        """계산·전송·수거를 한 번에. **실제로 나간 명령**을 관절 이름으로 돌려줌.

        명령한 것과 다를 수 있음 — 한계나 점프에 걸리면 잘림. 무엇을 보냈는지가
        아니라 **무엇이 실행됐는지**를 기록해야 나중에 로그를 믿을 수 있음.

        버스가 하나면 이걸 쓰면 됨. 둘 이상이면 `build_commands` / `send` /
        `collect` 를 직접 엮어 전송을 먼저 몰아야 함 (이슈 #10).
        """
        commands = self.build_commands(action)
        self.send(commands)
        self.collect()
        return self.last_sent

    @property
    @abc.abstractmethod
    def last_sent(self) -> Dict[str, float]:
        """마지막으로 나간 명령. 관절 이름 -> cal 각도."""
