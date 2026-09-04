"""통신이 끊긴 것을 판정함. **순수 함수, 하드웨어 무관.**

`guards` 가 "이 명령을 내보내도 되나" 를 보는 곳이면, 여기는 "명령이 도착하고
있나" 를 보는 곳임. 둘 다 명령이 나가기 전에 걸리지만 근거가 다름 — 가드는 값을
보고, 이쪽은 응답을 봄.


## 왜 필요한가

MIT 모드는 명령을 받으면 **반드시** 상태 프레임으로 답함. 안 오면 그 모터가
명령을 처리하지 않은 것임.

한 주기 빠지는 것은 흔함 — 버스가 붐비거나 수거가 늦으면 다음 주기에 옴. 문제는
**이어질 때**임. 그때 그 모터는:

    현재 위치를 모름          가드가 명령을 거부함 (reject_nostate)
    마지막 명령을 유지함       힘은 계속 나가고 있음

다리 하나면 그 관절만 굳음. **양다리면 로봇이 넘어짐** — 한쪽은 명령을 따라
움직이고 다른 쪽은 마지막 자세로 버팀.


## 판정은 주기 수로 함

시간이 아니라 연속 무응답 주기 수로 셈. 루프가 밀리면 시간 기준은 같은 상황에서
다르게 판정하는데, 주기 수는 "몇 번 명령했는데 몇 번 답이 없었나" 라서 루프
속도와 무관하게 뜻이 같음.

100Hz 에서 5주기면 50ms 임.


## 여기서 멈추지 않음

판정만 함. 실제로 세우는 것은 제어 루프임 (`control/loop.py`) — 정지 절차는
자세를 붙잡고 토크를 끊는 순서가 있어서, 그 순서를 아는 곳에서 해야 함.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Mapping, Optional, Tuple

DEFAULT_MAX_MISS_CYCLES = 5
"""이 주기 수만큼 연속으로 응답이 없으면 끊긴 것으로 봄. 100Hz 에서 50ms.

한 주기 빠지는 것은 흔해서 1 로 두면 멀쩡한 로봇이 계속 멈춤. 반대로 너무 크면
끊긴 채로 명령이 계속 나감.
"""


@dataclass(frozen=True)
class LinkLoss:
    """끊긴 것으로 판정된 상태. 무엇이 몇 주기째인지 담음."""

    motors: Tuple[str, ...]
    """끊긴 모터 이름. 합성 로봇이면 `팔다리/모터` 임."""

    cycles: int
    """가장 오래 끊긴 것의 연속 무응답 주기 수."""

    def __str__(self) -> str:
        return f"{list(self.motors)} 가 {self.cycles}주기째 응답 없음"


class LinkWatch:
    """응답 상태를 받아 끊겼는지 판정함. **상태를 들고 있지 않음.**

    연속 무응답 주기 수는 로봇이 이미 세고 있음 (`link_status` 의 `miss`). 여기서
    또 세면 같은 것을 두 군데서 세게 되고, 한쪽만 초기화되는 일이 생김.
    """

    def __init__(self, max_miss_cycles: int = DEFAULT_MAX_MISS_CYCLES) -> None:
        self.max_miss_cycles = int(max_miss_cycles)
        """0 이하면 판정하지 않음. 커미셔닝처럼 응답이 원래 드문 상황용임."""

    def __repr__(self) -> str:
        if not self.enabled:
            return "LinkWatch(꺼짐)"
        return f"LinkWatch({self.max_miss_cycles}주기)"

    @property
    def enabled(self) -> bool:
        return self.max_miss_cycles > 0

    def check(
        self, link_status: Mapping[str, Mapping[str, float]]
    ) -> Optional[LinkLoss]:
        """끊겼으면 `LinkLoss`, 아니면 `None`.

        `link_status` 는 `Leg.link_status()` / `Biped.link_status()` 가 내는 것임.

        **한 모터라도 걸리면 끊긴 것으로 봄.** 다리 하나가 통째로 죽는 것만
        위험한 게 아님 — 무릎 하나가 마지막 명령을 유지한 채 굳어도 그 다리는
        의도한 자세를 못 만듦.
        """
        if not self.enabled:
            return None

        stuck: Dict[str, int] = {
            name: int(value.get("miss", 0))
            for name, value in link_status.items()
            if int(value.get("miss", 0)) >= self.max_miss_cycles
        }
        if not stuck:
            return None
        return LinkLoss(motors=tuple(sorted(stuck)), cycles=max(stuck.values()))
