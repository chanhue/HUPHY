"""한 시점의 스냅샷 — **필드 이름을 정하는 유일한 곳.**

UDP 와 CSV 가 둘 다 여기서 나온 사전을 소비함. 두 군데에서 필드를 만들면 반드시
어긋남 — CSV 헤더에는 있는데 UDP 에는 없는 값이 생기고, 어느 쪽이 맞는지 알 수
없어짐.


## 두 갈래로 나눔

    빠른 것   매 주기      pos tgt err vel tau
    진단      N주기마다    temp age ack miss, 카운터, 마지막 사건 이후 경과

**나누는 이유는 패킷 크기임.** 다리 하나가 필드 66개면 약 1.8 KB 로 이더넷
MTU(1500)를 넘어 조각남. 조각 하나만 잃어도 패킷 전체가 버려짐.

나누는 것이 자연스럽기도 함 — `temp` 는 초 단위로 변하고 카운터는 사건이 있을 때만
변함. 100Hz 로 보낼 이유가 없음.

**CSV 는 나누지 않음.** 크기 제약이 없고, 한 줄에 다 있어야 나중에 대조하기 쉬움.


## 숫자만 담음

PlotJuggler 는 숫자만 그림. 불리언은 `0`/`1` 로, "없음" 은 `-1` 로 보냄 — 무한대는
JSON 으로 못 보내고 CSV 에서도 읽기 어려움.


## 이름 규약

    t                           시작부터 흐른 초
    right_leg/knee/pos          실측 위치 (cal 공간)
    right_leg/knee/ack          직전 주기에 응답했나
    right_leg/guard/clip_limit  누적 카운터

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

import time
from typing import Any, Dict, Mapping, Tuple

FAST_MOTOR_FIELDS = ("pos", "tgt", "err", "vel", "tau")
"""매 주기 나가는 값. 게인 튜닝에서 보는 것들임.

    pos    실측 위치 (cal 공간)
    tgt    목표 위치. 실제로 나간 것이지 명령한 것이 아님
    err    tgt - pos. 제일 먼저 보는 값
    vel    실측 속도
    tau    실측 토크
"""

DIAG_MOTOR_FIELDS = ("temp", "age", "ack", "miss")
"""느리게 변하거나 사건이 있을 때만 변하는 값.

    temp   권선 온도. 초 단위로 변함
    age    마지막 응답 이후 경과 (ms). 한 번도 없었으면 -1
    ack    1 응답함 / 0 씹힘 / -1 명령하지 않음
    miss   연속 무응답 주기 수

`age` 가 가장 믿을 만함. 응답이 한 번도 없던 모터는 현재 위치를 몰라 가드가 명령을
거부하므로, 명령도 안 나가고 응답 대상에서도 빠져 `ack` 가 -1 로 가려짐. `age` 는
명령 여부와 무관하게 참임.
"""

TORQUE_COMMAND_FIELD = "tau_cmd"
"""`tau_ff` 로 실어 보낸 토크. 모터의 `tau` 와 짝임.

    tau       모터가 냈다고 보고한 값
    tau_cmd   우리가 시킨 값

둘을 같이 봐야 원인이 좁혀짐 -- `tau` 가 낮을 때 적게 시킨 것인지, 시킨 만큼 못
낸 것인지(전류 한계·마찰·걸림) 구분됨.

위치 모드에서는 늘 0임. **모드에 따라 열이 나타났다 사라지지 않게** 그때도 냄.
"""

ANKLE_JOINT_FIELDS = ("pos", "tgt", "err", "vel")
"""발목 **관절** 값. 모터 값과 별개로 냄.

    pos    실측 각도. 모터각을 FK 로 푼 것
    tgt    목표 각도. 실제로 나간 것
    err    tgt - pos
    vel    실측 각속도 (도/초). 모터 속도를 야코비안으로 푼 것

모터 값(`ankle_a/pos` 등)만 보면 발이 실제로 어떤 자세인지 안 보임 -- 두 모터가
로드로 물려 있어 각도 하나가 자세 하나에 대응하지 않음. 사람도 모델도 관절로
생각하므로 그 축을 같이 냄.
"""

IMU_FIELDS = ("gx", "gy", "gz", "ax", "ay", "az", "grav_x", "grav_y", "grav_z", "age")
"""**모든 IMU 가 내는 값.** 센서를 바꿔도 이 열은 그대로임.

    gx gy gz            각속도 (도/초)
    ax ay az            가속도 (m/s^2). 중력 포함
    grav_x/y/z          중력방향 단위벡터. 정책에 그대로 들어가는 값
    age                 파싱한 지 얼마나 됐나 (ms). 한 번도 못 받았으면 -1

`grav_*` 를 내는 이유: **정책이 실제로 본 값**임. 자세가 이상해 보일 때 센서 원본이
이상한 것인지 중력방향 계산이 이상한 것인지 여기서 갈림.

센서 고유 값(오일러, 쿼터니언, 거리, 온도 등)은 `Imu.extra_fields` 로 따로 붙음 --
아래 `_imu_extra` 참조.
"""

IMU_MISSING_IS_MINUS_ONE = ("age", "sensor_dt", "sensor_ms")
"""값이 없을 때 0 이 아니라 -1 로 내보내는 필드.

나머지는 0 이 "아직 모름" 으로 읽혀도 무해하지만, 이 셋은 0 이 **"방금 받았다"** 는
뜻이라 정반대가 됨.
"""

SENSOR_DT_FIELD = "sensor_dt"
"""직전 패킷과의 센서 시각 차 (ms). 센서가 `sensor_ms` 를 낼 때만 붙음.

**패킷 손실이 여기서 드러남.** 100Hz 라면 10 이 정상이고 20 이면 한 개가 빠진 것임.
`age` 는 이걸 못 잡음 -- 빠진 패킷은 흔적 없이 사라지고, 다음 패킷이 제때 오면
`age` 는 정상으로 보임.

`age` 와 같이 봐야 원인이 좁혀짐.

    sensor_dt 10, age 10       정상
    sensor_dt 10, age 가 튐    센서는 규칙적인데 우리 스레드가 밀림
    sensor_dt 20, 30           패킷이 빠지는 중. 선이나 보레이트
    age 만 자람                센서가 멈춤
"""

FAST_GLOBAL = ("t", "loop_dt")
DIAG_GLOBAL = ("t", "missing", "since_clip", "since_reject")

GUARD_FIELDS = ("clip_limit", "clip_jump", "reject_nan", "reject_nostate")
CAN_FIELDS = ("tx_errors", "rx_errors", "drain_timeouts")


def _key(limb: str, group: str, field: str) -> str:
    return f"{limb}/{group}/{field}"


def _limb_of(robot: Any) -> str:
    return robot.id or robot.name


def parts(robot: Any) -> Tuple[Any, ...]:
    """기록 단위. 합성 로봇이면 팔다리들, 아니면 자기 자신 하나.

    **팔다리마다 따로 찍음.** 합성 로봇을 통째로 한 단위로 보면 필드 이름이
    `huphy/right_leg/knee/pos` 처럼 한 겹 깊어져, 다리 하나로 돌리던 때의 로그·그래프
    레이아웃과 안 맞음. 팔다리별로 찍으면 `right_leg/knee/pos` 그대로임.

    UDP 를 팔다리마다 한 패킷씩 보내는 근거이기도 함 -- 합치면 MTU 를 넘음
    (`merge` 참조).
    """
    return tuple(getattr(robot, "parts", ())) or (robot,)


def imus_of(robot: Any) -> Tuple[Any, ...]:
    """이 로봇에서 값을 낼 수 있는 IMU 전부.

    합성 로봇은 자기가 든 것과 팔다리가 든 것을 합쳐 `all_imus` 로 냄 -- 몸통
    센서와 다리 센서가 한 줄에 같이 있어야 대조가 됨.
    """
    return tuple(getattr(robot, "all_imus", None) or getattr(robot, "imus", ()))


# ===========================================================================
# 필드 목록
# ===========================================================================
def fast_field_names(robot: Any) -> Tuple[str, ...]:
    """매 주기 나가는 필드."""
    limb = _limb_of(robot)
    names = list(FAST_GLOBAL)
    for motor in robot.motor_names:
        names.extend(_key(limb, motor, f) for f in FAST_MOTOR_FIELDS)
    for joint in _ankle_joints(robot):
        names.extend(_key(limb, joint, f) for f in ANKLE_JOINT_FIELDS)
    for motor in _torque_motors(robot):
        names.append(_key(limb, motor, TORQUE_COMMAND_FIELD))
    return tuple(names)


def diag_field_names(robot: Any) -> Tuple[str, ...]:
    """진단 필드. 느린 주기로 나감."""
    limb = _limb_of(robot)
    names = list(DIAG_GLOBAL)
    for motor in robot.motor_names:
        names.extend(_key(limb, motor, f) for f in DIAG_MOTOR_FIELDS)
    names.extend(f"{limb}/guard/{f}" for f in GUARD_FIELDS)
    names.extend(f"{limb}/can/{f}" for f in CAN_FIELDS)
    return tuple(names)


def imu_field_names(robot: Any) -> Tuple[str, ...]:
    """IMU 필드. 붙은 IMU 가 없으면 `t` 만.

    **팔다리 이름이 앞에 붙지 않음.** IMU 개체 이름을 씀 -- 같은 센서가 다리에
    붙었다가 몸통으로 옮겨감. 팔다리 이름을 쓰면 옮기는 순간 필드 이름이 바뀌어
    예전 로그·그래프 레이아웃과 안 맞음.
    """
    names = ["t"]
    for imu in imus_of(robot):
        head = f"imu/{imu.name}"
        names.extend(f"{head}/{f}" for f in IMU_FIELDS)
        names.extend(f"{head}/{f}" for f in _extra_fields(imu))
        if "sensor_ms" in _extra_fields(imu):
            names.append(f"{head}/{SENSOR_DT_FIELD}")
    return tuple(names)


def field_names(robot: Any) -> Tuple[str, ...]:
    """CSV 열. 빠른 것·진단·IMU 를 합친 것임. 순서가 열 순서임.

    **팔다리가 여럿이면 전부 덮음.** 한 줄에 양다리가 같이 들어가야 같은 시각의
    두 다리를 나란히 볼 수 있음.

    `t` 는 셋 다에 있으므로 한 번만 넣음. IMU 는 로봇 전체 것이라 팔다리마다
    반복하지 않음.
    """
    out: list = []
    for part in parts(robot):
        for name in fast_field_names(part) + diag_field_names(part):
            if name not in out:
                out.append(name)
    for name in imu_field_names(robot):
        if name not in out:
            out.append(name)
    return tuple(out)


# ===========================================================================
# 스냅샷
# ===========================================================================
def build_fast(robot: Any, *, t: float, loop_dt_ms: float = 0.0) -> Dict[str, float]:
    """매 주기 나갈 값.

    `robot` 에서 읽기만 함 — 새로 통신하지 않음. 제어 루프가 이미 수거해 둔 값을
    씀. 여기서 CAN 을 건드리면 기록이 주기를 흔들게 됨.

    `tgt` 는 **실제로 나간 명령**임 (`robot.last_sent`). 명령한 값을 기록하면
    한계에 걸린 것과 게인이 낮은 것이 그래프에서 구분되지 않음.
    """
    limb = _limb_of(robot)
    observation = robot.get_observation()
    sent = robot.last_sent

    out: Dict[str, float] = {"t": float(t), "loop_dt": float(loop_dt_ms)}

    for motor in robot.motor_names:
        pos = float(observation.get(f"{motor}.pos", 0.0))
        # 발목은 명령이 관절(pitch/roll)로 오므로 모터별 목표가 last_sent 에 없음.
        # 그때는 실측을 목표로 둬서 오차가 0이 되게 함 -- 0이 아닌 가짜 오차가
        # 그래프에 남는 것보다 나음.
        tgt = float(sent.get(motor, pos))
        out[_key(limb, motor, "pos")] = pos
        out[_key(limb, motor, "tgt")] = tgt
        out[_key(limb, motor, "err")] = tgt - pos
        out[_key(limb, motor, "vel")] = float(observation.get(f"{motor}.vel", 0.0))
        out[_key(limb, motor, "tau")] = float(observation.get(f"{motor}.torque", 0.0))

    # 발목 관절. 모터 값과 축이 달라 따로 냄.
    velocity = _ankle_velocity(robot)
    for index, joint in enumerate(_ankle_joints(robot)):
        pos = float(observation.get(f"{joint}.pos", 0.0))
        tgt = float(sent.get(joint, pos))
        out[_key(limb, joint, "pos")] = pos
        out[_key(limb, joint, "tgt")] = tgt
        out[_key(limb, joint, "err")] = tgt - pos
        out[_key(limb, joint, "vel")] = velocity[index]

    # 시킨 토크. 모터가 보고한 tau 와 짝임.
    commanded = _torque_command(robot)
    for motor in _torque_motors(robot):
        out[_key(limb, motor, TORQUE_COMMAND_FIELD)] = float(commanded.get(motor, 0.0))
    return out


def build_diag(robot: Any, *, t: float) -> Dict[str, float]:
    """진단 값.

    `ack` 가 핵심임 — MIT 모드는 명령을 받으면 반드시 상태 프레임으로 답하므로,
    **안 오면 그 모터가 명령을 처리하지 않은 것임.**

    `can/tx_errors` 와 같이 봐야 원인이 좁혀짐.

        tx_errors = 0 인데 ack = 0   ->  모터가 명령을 무시함 (프로토콜·모드 불일치)
        tx_errors > 0                ->  버스에 아무도 없음 (배선·전원)

    CAN 하드웨어 ACK 는 버스에 붙은 아무 노드나 찍어주므로 "누군가 들었다" 일 뿐임.
    모터별로 처리됐는지는 응답 프레임으로만 알 수 있음 (이슈 #11).
    """
    limb = _limb_of(robot)
    observation = robot.get_observation()
    link = _link_status(robot)

    out: Dict[str, float] = {
        "t": float(t),
        # 명령했는데 응답이 없는 모터 수. 명령하지 않은 것(-1)은 세지 않음.
        "missing": float(sum(1 for s in link.values() if s.get("ack", -1.0) == 0.0)),
        "since_clip": _call(robot, "since_clip"),
        "since_reject": _call(robot, "since_reject"),
    }

    for motor in robot.motor_names:
        status = link.get(motor, {})
        out[_key(limb, motor, "temp")] = float(observation.get(f"{motor}.temp", 0.0))
        out[_key(limb, motor, "age")] = float(status.get("age", -1.0))
        out[_key(limb, motor, "ack")] = float(status.get("ack", -1.0))
        out[_key(limb, motor, "miss")] = float(status.get("miss", 0.0))

    guard = getattr(robot, "counters", None)
    out[f"{limb}/guard/clip_limit"] = _counter(guard, "clips", "limit")
    out[f"{limb}/guard/clip_jump"] = _counter(guard, "clips", "jump")
    out[f"{limb}/guard/reject_nan"] = _counter(guard, "rejects", "nan")
    out[f"{limb}/guard/reject_nostate"] = _counter(guard, "rejects", "nostate")

    can_counters = getattr(getattr(getattr(robot, "bus", None), "bus", None), "counters", None)
    for field in CAN_FIELDS:
        out[f"{limb}/can/{field}"] = float(getattr(can_counters, field, 0))

    return out


def build_imu(robot: Any, *, t: float) -> Dict[str, float]:
    """IMU 값. 붙은 것이 없으면 `t` 만 담김.

    **값이 없어도 키는 냄.** 센서가 아직 패킷을 못 받았으면 전부 0 이고 `age` 가
    -1 임 -- 0도라는 뜻이 아니라 모른다는 뜻이고, `age` 로 구분함.
    """
    out: Dict[str, float] = {"t": float(t)}
    states = _imu_states(robot)
    now = time.monotonic()

    for imu in imus_of(robot):
        head = f"imu/{imu.name}"
        extra_names = _extra_fields(imu)
        state = states.get(imu.name)

        if state is None:
            for field in IMU_FIELDS + extra_names:
                out[f"{head}/{field}"] = _blank(field)
            if "sensor_ms" in extra_names:
                out[f"{head}/{SENSOR_DT_FIELD}"] = -1.0
            continue

        out[f"{head}/gx"], out[f"{head}/gy"], out[f"{head}/gz"] = (
            float(v) for v in state.gyro_dps
        )
        out[f"{head}/ax"], out[f"{head}/ay"], out[f"{head}/az"] = (
            float(v) for v in state.accel_mps2
        )
        out[f"{head}/grav_x"], out[f"{head}/grav_y"], out[f"{head}/grav_z"] = (
            float(v) for v in state.gravity
        )
        out[f"{head}/age"] = float(state.age_ms(now))

        # 센서 고유 값. 이번 주기에 없어도 키는 냄 -- 열이 사라지면 CSV 가 밀림.
        for field in extra_names:
            out[f"{head}/{field}"] = float(state.extra.get(field, _blank(field)))
        if "sensor_ms" in extra_names:
            out[f"{head}/{SENSOR_DT_FIELD}"] = _sensor_dt(
                imu.name, out[f"{head}/sensor_ms"]
            )
    return out


def _blank(field: str) -> float:
    """값이 없을 때 낼 숫자."""
    return -1.0 if field in IMU_MISSING_IS_MINUS_ONE else 0.0


def _extra_fields(imu: Any) -> Tuple[str, ...]:
    """이 센서가 내는 고유 값의 키 목록. 안 내면 빈 것.

    `Imu` 계약의 일부지만 없어도 동작하게 둠 -- 테스트의 가짜 IMU 나 아주 단순한
    구현이 이것까지 갖추도록 강요할 이유가 없음.
    """
    return tuple(getattr(imu, "extra_fields", ()))


_LAST_SENSOR_MS: Dict[str, float] = {}
"""IMU 이름 -> 직전에 본 `sensor_ms`. `sensor_dt` 를 내려고 들고 있음."""


def _sensor_dt(imu_name: str, sensor_ms: float) -> float:
    """직전 패킷과의 센서 시각 차 (ms). 센서가 시각을 안 주면 -1."""
    if sensor_ms < 0:
        return -1.0
    previous = _LAST_SENSOR_MS.get(imu_name)
    _LAST_SENSOR_MS[imu_name] = float(sensor_ms)
    if previous is None:
        return -1.0
    return float(sensor_ms) - previous


def build(robot: Any, *, t: float, loop_dt_ms: float = 0.0) -> Dict[str, float]:
    """CSV 한 줄. 빠른 것·진단·IMU 를 합친 것임."""
    out = build_fast(robot, t=t, loop_dt_ms=loop_dt_ms)
    out.update(build_diag(robot, t=t))
    out.update(build_imu(robot, t=t))
    return out


# ===========================================================================
# 없어도 0을 냄
# ===========================================================================
def _ankle_joints(robot: Any) -> Tuple[str, ...]:
    """이 로봇에 발목 관절이 있으면 그 이름들. 없으면 빈 것.

    `Robot` 계약에 없는 선택 기능임 -- 팔에는 발목이 없음.
    """
    joints = getattr(robot, "joint_names", ())
    return tuple(j for j in joints if j in ("ankle_pitch", "ankle_roll"))


def _torque_motors(robot: Any) -> Tuple[str, ...]:
    """토크를 실어 보낼 수 있는 모터. 없으면 빈 것.

    로봇이 고정으로 내는 목록임 -- 모드에 따라 바뀌면 CSV 열이 밀림.
    """
    return tuple(getattr(robot, "torque_motors", ()))


def _torque_command(robot: Any) -> Dict[str, float]:
    """직전 주기에 시킨 토크. 없으면 빈 사전."""
    value = getattr(robot, "last_torque", None)
    return dict(value) if value else {}


def _ankle_velocity(robot: Any) -> Tuple[float, float]:
    """발목 관절 각속도. 없거나 실패하면 0.

    **예외를 삼킴.** 기록이 제어를 멈추면 안 됨.
    """
    getter = getattr(robot, "ankle_velocity", None)
    if not callable(getter):
        return (0.0, 0.0)
    try:
        return getter()
    except Exception:
        return (0.0, 0.0)


def _imu_names(robot: Any) -> Tuple[str, ...]:
    """붙은 IMU 이름들. 없으면 빈 것.

    `Robot` 계약에 없는 선택 기능임 -- IMU 가 없는 로봇도 그대로 돎.
    """
    return tuple(imu.name for imu in imus_of(robot))


def _imu_states(robot: Any) -> Dict[str, Any]:
    """IMU 값을 물어봄. 없거나 실패하면 빈 사전.

    **예외를 삼킴.** 센서 하나 때문에 기록이 멈추면 안 됨.
    """
    getter = getattr(robot, "imu_states", None)
    if not callable(getter):
        return {}
    try:
        return getter()
    except Exception:
        return {}


def _link_status(robot: Any) -> Dict[str, Dict[str, float]]:
    """링크 상태를 물어봄. 없으면 빈 사전.

    `Robot` 계약에 없는 선택 기능임 — 로봇마다 통신 방식이 달라 응답 개념이 없을
    수도 있음. 없으면 `ack=0`, `age=-1` 로 나감.
    """
    getter = getattr(robot, "link_status", None)
    return getter() if callable(getter) else {}


def _call(robot: Any, name: str) -> float:
    getter = getattr(robot, name, None)
    return float(getter()) if callable(getter) else -1.0


def _counter(counters: Any, group: str, key: str) -> float:
    """카운터가 없어도 0을 냄. **키가 사라지면 그래프가 끊김.**"""
    if counters is None:
        return 0.0
    return float(getattr(counters, group, {}).get(key, 0))


def merge(*snapshots: Mapping[str, float]) -> Dict[str, float]:
    """여러 팔다리의 스냅샷을 하나로 합침. **CSV 전용임.**

    팔다리 이름이 앞에 붙어 있어 키가 겹치지 않음. `t` 와 `loop_dt` 는 같은 주기의
    값이라 뒤엣것이 앞엣것을 덮어써도 같음.

    **UDP 로는 합쳐 보내지 말 것.** 다리 하나의 빠른 패킷이 이미 1 KB 가까이 되어,
    둘을 합치면 이더넷 MTU(1500)를 넘어 조각남. 조각 하나만 잃어도 패킷 전체가
    버려짐.

    UDP 는 팔다리마다 한 패킷씩 보냄 — PlotJuggler 는 여러 출처를 같은 타임라인에
    올림.
    """
    out: Dict[str, float] = {}
    for snapshot in snapshots:
        out.update(snapshot)
    return out
