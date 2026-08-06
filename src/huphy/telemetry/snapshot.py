"""한 시점의 스냅샷 — **필드 이름을 정하는 유일한 곳.**

UDP 와 CSV 가 둘 다 여기서 나온 사전을 소비함. 두 군데에서 필드를 만들면 반드시
어긋남 — CSV 헤더에는 있는데 UDP 에는 없는 값이 생기고, 어느 쪽이 맞는지 알 수
없어짐.


## 숫자만 담음

PlotJuggler 는 숫자만 그림. 불리언은 `0`/`1` 정수로, 문자열은 정수 코드로 바꿔
보냄. 원문 문자열이 필요하면 CSV 쪽에 따로 남길 것.


## 이름 규약

    t                          시작부터 흐른 초
    right_leg/knee/pos         실측 위치 (cal 공간)
    right_leg/knee/tgt         목표 위치
    right_leg/knee/err         오차 = tgt - pos
    right_leg/guard/clip_limit 누적 카운터

`/` 로 나눔. PlotJuggler 가 트리로 묶어 보여줌 — 모터가 20개를 넘어가면 평평한
목록에서는 찾을 수 없음.

**팔다리 이름이 앞에 붙음.** 양다리를 같이 기록할 때 `knee` 가 둘이 되기 때문임.


## 필드 목록을 미리 알 수 있어야 함

`field_names()` 가 로봇만 보고 목록을 냄. 실행 전에 알아야 CSV 헤더를 쓸 수 있고,
PlotJuggler 레이아웃도 미리 만들어 둘 수 있음.

**필드가 나타났다 사라지면 안 됨.** 값이 없어도 키는 내보내고 `0` 을 채움 — 중간에
사라지면 그래프가 끊기고 CSV 열이 밀림.
"""

from __future__ import annotations

from typing import Any, Dict, Mapping, Optional, Tuple

MOTOR_FIELDS = ("pos", "tgt", "err", "vel", "tau", "temp")
"""모터마다 나가는 값.

    pos    실측 위치 (cal 공간)
    tgt    목표 위치. 실제로 나간 것이지 명령한 것이 아님
    err    tgt - pos. 게인 튜닝에서 제일 먼저 보는 값
    vel    실측 속도
    tau    실측 토크
    temp   권선 온도
"""

GUARD_FIELDS = ("clip_limit", "clip_jump", "reject_nan", "reject_nostate")
CAN_FIELDS = ("tx_errors", "rx_errors", "drain_timeouts")
LOOP_FIELDS = ("t", "loop_dt", "missing")


def _motor_key(limb: str, motor: str, field: str) -> str:
    return f"{limb}/{motor}/{field}"


def field_names(robot: Any) -> Tuple[str, ...]:
    """이 로봇이 내보낼 필드 이름들. **실행 전에 알 수 있음.**

    CSV 헤더와 PlotJuggler 레이아웃이 이 목록을 씀. 순서가 CSV 열 순서임.
    """
    names = ["t", "loop_dt", "missing"]
    limb = robot.id or robot.name
    for motor in robot.motor_names:
        names.extend(_motor_key(limb, motor, f) for f in MOTOR_FIELDS)
    names.extend(f"{limb}/guard/{f}" for f in GUARD_FIELDS)
    names.extend(f"{limb}/can/{f}" for f in CAN_FIELDS)
    return tuple(names)


def build(
    robot: Any,
    *,
    t: float,
    loop_dt_ms: float = 0.0,
    missing: int = 0,
) -> Dict[str, float]:
    """지금 상태를 한 사전으로 만듦.

    `robot` 에서 읽기만 함 — 새로 통신하지 않음. 제어 루프가 이미 수거해 둔 값을
    씀. 여기서 CAN 을 건드리면 기록이 주기를 흔들게 됨.

    `tgt` 는 **실제로 나간 명령**임 (`robot.last_sent`). 명령한 값이 아니라 잘리고
    남은 값이라, 오차를 보면 모터가 왜 그렇게 움직였는지가 설명됨.
    """
    limb = robot.id or robot.name
    observation = robot.get_observation()
    sent = robot.last_sent

    out: Dict[str, float] = {
        "t": float(t),
        "loop_dt": float(loop_dt_ms),
        "missing": int(missing),
    }

    for motor in robot.motor_names:
        pos = float(observation.get(f"{motor}.pos", 0.0))
        # 발목은 명령이 관절(pitch/roll)로 오므로 모터별 목표가 last_sent 에 없음.
        # 그때는 실측을 목표로 둬서 오차가 0이 되게 함 -- 0이 아닌 가짜 오차가
        # 그래프에 남는 것보다 나음.
        tgt = float(sent.get(motor, pos))
        out[_motor_key(limb, motor, "pos")] = pos
        out[_motor_key(limb, motor, "tgt")] = tgt
        out[_motor_key(limb, motor, "err")] = tgt - pos
        out[_motor_key(limb, motor, "vel")] = float(observation.get(f"{motor}.vel", 0.0))
        out[_motor_key(limb, motor, "tau")] = float(observation.get(f"{motor}.torque", 0.0))
        out[_motor_key(limb, motor, "temp")] = float(observation.get(f"{motor}.temp", 0.0))

    guard = getattr(robot, "counters", None)
    out[f"{limb}/guard/clip_limit"] = _counter(guard, "clips", "limit")
    out[f"{limb}/guard/clip_jump"] = _counter(guard, "clips", "jump")
    out[f"{limb}/guard/reject_nan"] = _counter(guard, "rejects", "nan")
    out[f"{limb}/guard/reject_nostate"] = _counter(guard, "rejects", "nostate")

    can = getattr(getattr(robot, "bus", None), "bus", None)
    can_counters = getattr(can, "counters", None)
    for field in CAN_FIELDS:
        out[f"{limb}/can/{field}"] = float(getattr(can_counters, field, 0))

    return out


def _counter(counters: Any, group: str, key: str) -> float:
    """카운터가 없어도 0을 냄. **키가 사라지면 그래프가 끊김.**"""
    if counters is None:
        return 0.0
    return float(getattr(counters, group, {}).get(key, 0))


def merge(*snapshots: Mapping[str, float]) -> Dict[str, float]:
    """여러 팔다리의 스냅샷을 하나로 합침. **CSV 전용임.**

    팔다리 이름이 앞에 붙어 있어 키가 겹치지 않음. `t` 와 `loop_dt` 는 같은 주기의
    값이라 뒤엣것이 앞엣것을 덮어써도 같음.

    **UDP 로는 합쳐 보내지 말 것.** 다리 하나가 이미 1.3 KB 가까이 되어, 둘을 합치면
    이더넷 MTU(1500)를 넘어 조각남. 조각 하나만 잃어도 패킷 전체가 버려짐.

    UDP 는 팔다리마다 한 패킷씩 보냄 — PlotJuggler 는 여러 출처를 같은 타임라인에
    올림. CSV 는 크기 제약이 없으므로 한 파일에 모든 열을 두는 편이 다시 볼 때 편함.
    """
    out: Dict[str, float] = {}
    for snapshot in snapshots:
        out.update(snapshot)
    return out
