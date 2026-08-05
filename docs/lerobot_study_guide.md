# LeRobot 코드 읽기 가이드 — 로봇 제어 입문

로봇 제어가 처음인 사람이 [LeRobot](https://github.com/huggingface/lerobot) 저장소를
**직접 열어보며** 구조를 익히기 위한 안내서.

저장소는 이미 클론되어 있다: `<repo>/lerobot/` (git 추적 제외)
경로는 전부 `lerobot/src/lerobot/` 기준이며, 문서에서는 `src/lerobot/`로 줄여 쓴다.

---

## 0. 이 가이드를 쓰는 법

**순서대로 읽으면서 실제 파일을 같이 열 것.** 각 단계마다

```
📂 열 파일:   경로:줄번호
🎯 목표:      이 단계에서 이해할 것
❓ 확인 질문:  스스로 답해보기
```

가 붙어 있다. 코드를 안 열고 이 문서만 읽으면 남는 게 없다.

### 규모 감각

```
src/lerobot/  파이썬 파일 490개

policies/     52,201줄   ← 신경망 정책. 지금은 건너뛴다
datasets/     12,202줄   ← 데이터셋. 4단계에서
robots/        7,984줄   ← ★ 여기서 시작
scripts/       7,603줄   ← ★ 진입점
processor/     6,992줄
teleoperators/ 6,920줄   ← ★
motors/        5,341줄   ← ★
cameras/       3,008줄
configs/       2,126줄
```

**★ 표시된 4개만 읽어도 "로봇을 어떻게 움직이는가"는 전부 이해된다.**
`policies/`와 `datasets/`는 "움직임을 학습시키는" 영역이라 나중이다.

---

## 1. 큰 그림 — LeRobot은 무엇을 하나

한 문장: **사람이 로봇 팔을 움직여 시범을 보이고 → 그걸 데이터로 기록하고 →
신경망에 학습시켜 → 로봇이 스스로 하게 만드는** 파이프라인.

```
   [사람이 조종]              [기록]                [학습]              [자율 실행]
  lerobot-teleoperate  →  lerobot-record  →  lerobot-train  →  lerobot-eval
       ↓                       ↓                  ↓                  ↓
  Teleoperator            LeRobotDataset      PreTrainedPolicy    Policy → Robot
       ↓
     Robot
```

우리(HUPHY)에게 지금 필요한 건 **맨 왼쪽 두 칸**이다. 다리 제어는 아직 "움직이게 하는"
단계이고 "학습" 단계가 아니다.

### 4개의 핵심 추상

| 추상 | 파일 | 한 줄 |
|---|---|---|
| `Robot` | `robots/robot.py` | 관측을 주고 행동을 받는 하드웨어 |
| `Teleoperator` | `teleoperators/teleoperator.py` | 행동을 만들어내는 입력 장치 |
| `MotorsBus` | `motors/motors_bus.py` | 모터들과 실제로 통신하는 계층 |
| `LeRobotDataset` | `datasets/lerobot_dataset.py` | 기록된 시연 데이터 |

`Robot`과 `Teleoperator`가 **거울상**이라는 게 핵심이다:

```
Teleoperator.get_action()  →  {"shoulder_pan.pos": 12.3, ...}  →  Robot.send_action()
```

리더 팔(사람이 잡는 것)과 팔로워 팔(움직이는 것)이 **같은 모터·같은 버스**를 쓴다.
리더는 읽기만 하고 팔로워는 쓰기만 할 뿐이다. 이게 SO-100/101 시스템의 전부다.

---

## 2. 읽기 순서 로드맵

```
1단계  teleoperate 스크립트     ← 가장 작은 완결 루프. 여기서 시작
2단계  Robot 추상 (ABC)         ← 인터페이스가 무엇을 요구하나
3단계  SOFollower 구체 구현     ← 실제 로봇 한 대
4단계  MotorsBus                ← 모터와의 실제 통신
5단계  설정과 임포트 체계        ← draccus, 팩토리, __init__
6단계  Teleoperator             ← 거울상 확인
7단계  record → dataset         ← 데이터로 넘어가기
8단계  policy → train → eval    ← 학습 (선택)
```

각 단계는 **1~2시간**을 잡으면 된다. 1~4단계가 핵심이고, 여기까지가 우리 프로젝트와
직결된다.

---

## 3. [1단계] 가장 작은 완결 루프

```
📂 src/lerobot/scripts/lerobot_teleoperate.py  (289줄)
🎯 로봇을 움직이는 최소한의 코드가 무엇인지
```

### 3.1 먼저 `teleop_loop`만 본다 (155~237줄)

문서 문자열과 import를 다 건너뛰고 **188줄의 `while True`부터** 읽는다.

```python
while True:
    loop_start = time.perf_counter()

    obs = robot.get_observation()                            # ① 로봇 상태 읽기
    raw_action = teleop.get_action()                         # ② 사람 입력 읽기
    teleop_action = teleop_action_processor((raw_action, obs))    # ③ 가공
    robot_action_to_send = robot_action_processor((teleop_action, obs))
    _ = robot.send_action(robot_action_to_send)              # ④ 로봇에 전송

    dt_s = time.perf_counter() - loop_start
    precise_sleep(max(1 / fps - dt_s, 0.0))                  # ⑤ 주기 맞추기
```

**이게 로봇 제어의 전부다.** 읽고 → 계산하고 → 쓰고 → 잔다. 나머지 48만 줄은 이
네 줄을 안전하고 유연하게 만들기 위한 것이다.

### 3.2 주목할 것

**`send_action`이 반환값을 갖는다** (210줄)
```python
_ = robot.send_action(robot_action_to_send)
```
안전 클램프 때문에 **실제로 보낸 값이 요청과 다를 수 있다.** 무엇이 나갔는지
호출부가 알아야 한다. (우리 `SingleLeg.send_action()`도 같은 이유로 반환값이 있다)

**주기 맞추기가 `sleep(1/fps)`가 아니다** (231줄)
```python
dt_s = time.perf_counter() - loop_start
precise_sleep(max(1 / fps - dt_s, 0.0))
```
루프가 이미 쓴 시간을 **빼고** 잔다. 그냥 `sleep(1/fps)`면 실제 주기가
`작업시간 + 1/fps`가 되어 목표보다 항상 느려진다.

> 🔍 `src/lerobot/utils/robot_utils.py`의 `precise_sleep`을 열어보라. 왜 그냥
> `time.sleep`을 안 쓰는지 알 수 있다.

**루프 시간을 매 사이클 찍는다** (233줄)
```python
print(f"Teleop loop time: {loop_s * 1e3:.2f}ms ({1 / loop_s:.0f} Hz)")
```
실제 주기를 보여주는 것이 기본이다. (우리가 `loop_dt`를 텔레메트리에 넣은 것과 같은 동기)

### 3.3 `teleoperate()` 함수 (240~280줄)

```python
teleop = make_teleoperator_from_config(cfg.teleop)   # 팩토리
robot = make_robot_from_config(cfg.robot)            # 팩토리

teleop.connect()
robot.connect()
try:
    teleop_loop(...)
except KeyboardInterrupt:
    pass
finally:
    teleop.disconnect()      # ← 반드시 finally에서
    robot.disconnect()
```

**`finally`에서 끊는 것이 안전 규약이다.** Ctrl+C를 눌러도 모터가 토크를 문 채로
남으면 안 된다. 우리 `bringup.py`도 같은 구조를 쓴다.

### ❓ 확인 질문
1. `fps=60`이면 한 사이클에 몇 ms가 주어지나? 그 안에 `get_observation`,
   `get_action`, `send_action`이 다 끝나야 하나?
2. `robot.get_observation()`이 20ms 걸리면 실제 주기는 어떻게 되나?
3. `except KeyboardInterrupt: pass`가 `finally`보다 앞에 있는 이유는?

---

## 4. [2단계] `Robot` 추상 — 인터페이스가 요구하는 것

```
📂 src/lerobot/robots/robot.py  (211줄)
🎯 "로봇이란 무엇인가"를 코드로 정의하면 어떻게 되나
```

### 4.1 클래스 변수 (42~44줄)

```python
class Robot(abc.ABC):
    config_class: builtins.type[RobotConfig]
    name: str
```

모든 구체 로봇은 이 둘을 정의해야 한다. `name`은 `"so_follower"` 같은 식별자,
`config_class`는 이 로봇이 받는 설정 dataclass다.

### 4.2 `__init__`이 하는 일 (46~56줄)

```python
def __init__(self, config: RobotConfig):
    self.robot_type = self.name
    self.id = config.id
    self.calibration_dir = (
        config.calibration_dir if config.calibration_dir
        else HF_LEROBOT_CALIBRATION / ROBOTS / self.name
    )
    self.calibration_dir.mkdir(parents=True, exist_ok=True)
    self.calibration_fpath = self.calibration_dir / f"{self.id}.json"
    self.calibration: dict[str, MotorCalibration] = {}
    if self.calibration_fpath.is_file():
        self._load_calibration()
```

**핵심**: 캘리브레이션이 **파일에서 로드된다.** 코드에 상수로 박혀 있지 않다.

- `id`는 같은 종류 로봇을 구분하는 이름 (`"black"`, `"blue"`)
- 캘리브레이션 파일은 `~/.cache/huggingface/lerobot/calibration/robots/so_follower/black.json`
- 파일이 없으면 빈 dict로 시작하고, `calibrate()`가 만들어 저장한다

> 💡 **우리 프로젝트와 대조**: HUPHY도 `config/calibration/right_leg.json`으로 같은
> 구조를 쓴다. 원본 `robot_constant.py`가 값을 코드에 박아둬서 `CALIBRATED=False`가
> 풀리지 않았던 것과 대비된다.

### 4.3 컨텍스트 매니저와 소멸자 (61~85줄)

```python
def __enter__(self):
    self.connect()
    return self

def __exit__(self, exc_type, exc_value, traceback):
    self.disconnect()

def __del__(self):
    try:
        if self.is_connected:
            self.disconnect()
    except Exception:
        pass
```

```python
with SOFollower(config) as robot:
    robot.send_action(...)
# 여기서 자동으로 disconnect
```

`__del__`은 **마지막 안전망**이다. 예외로 죽어서 `finally`도 못 탔을 때 가비지
컬렉션 시점에라도 끊으려는 것. 예외를 삼키는 이유는 소멸자에서 예외가 나면 더
난감해지기 때문이다.

### 4.4 추상 속성 4개

| 속성 | 의미 |
|---|---|
| `observation_features` | `get_observation()`이 돌려줄 dict의 **구조** |
| `action_features` | `send_action()`이 받는 dict의 구조 |
| `is_connected` | |
| `is_calibrated` | 해당 없으면 항상 `True` |

**`observation_features`의 값은 타입 또는 shape이다:**
```python
{
    "shoulder_pan.pos": float,          # 단일 값 → 타입
    "front": (480, 640, 3),             # 배열 → shape (h, w, c)
}
```

> **왜 이런 게 필요한가**: 데이터셋을 만들 때 "이 로봇은 어떤 필드를 갖는가"를 미리
> 알아야 한다. 또 정책의 입출력 차원을 정할 때도 쓰인다. 하드웨어를 연결하지 않고도
> 호출 가능해야 하는 이유가 이것이다 (docstring에 명시되어 있다).

### 4.5 추상 메서드 6개

| 메서드 | 언제 |
|---|---|
| `connect(calibrate=True)` | 시작 시 1회 |
| `calibrate()` | 캘리브레이션이 없거나 어긋날 때 |
| `configure()` | 연결 후 매번 (모드·게인 설정) |
| `get_observation()` | 매 사이클 |
| `send_action(action)` | 매 사이클 |
| `disconnect()` | 종료 시 |

**`calibrate()`와 `configure()`의 차이**가 중요하다:
- `calibrate` — **모터에 영구 저장**되는 값 (영점, 가동범위). 드물게
- `configure` — 매 연결마다 다시 쓰는 런타임 설정 (제어 모드, PID 게인)

### ❓ 확인 질문
1. `observation_features`가 "연결 여부와 무관하게 호출 가능"해야 하는 이유는?
2. `calibrate()`와 `configure()` 중 어느 것이 전원을 껐다 켜도 유지되나?
3. `__del__`에서 예외를 삼키는 이유는?

---

## 5. [3단계] 구체 로봇 — `SOFollower`

```
📂 src/lerobot/robots/so_follower/so_follower.py  (242줄)
🎯 추상 인터페이스를 실제 하드웨어로 채우면 어떻게 되나
```

**이 파일이 이 가이드에서 가장 중요하다.** 242줄로 로봇 한 대가 완성된다.

### 5.1 `__init__` — 모터 선언 (46~63줄)

```python
def __init__(self, config: SOFollowerRobotConfig):
    super().__init__(config)
    self.config = config
    norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
    self.bus = FeetechMotorsBus(
        port=self.config.port,
        motors={
            "shoulder_pan":  Motor(1, "sts3215", norm_mode_body),
            "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
            "elbow_flex":    Motor(3, "sts3215", norm_mode_body),
            "wrist_flex":    Motor(4, "sts3215", norm_mode_body),
            "wrist_roll":    Motor(5, "sts3215", norm_mode_body),
            "gripper":       Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        },
        calibration=self.calibration,
    )
    self.cameras = make_cameras_from_configs(config.cameras)
```

**여기서 배울 것 4가지**

1. **`{관절이름: Motor(id, model, norm_mode)}`** — 이 dict 하나가 로봇의 토폴로지다.
   우리 `config/robot.yaml`의 `motors:` 목록과 같은 역할인데, LeRobot은 코드에 두고
   우리는 yaml로 뺐다.

2. **`gripper`만 정규화 모드가 다르다.** 그리퍼는 "열림 0% ~ 닫힘 100%"가 자연스럽고,
   관절은 각도나 −100~100이 자연스럽다. **같은 로봇 안에서도 관절마다 단위가 다를 수 있다.**

3. **`calibration=self.calibration`** — 부모 `__init__`이 파일에서 읽어둔 것을 버스에
   넘긴다. 버스가 정규화할 때 이 값을 쓴다.

4. **카메라도 설정에서 만들어진다.** 로봇 = 모터 + 카메라.

### 5.2 features (65~85줄)

```python
@property
def _motors_ft(self) -> dict[str, type]:
    return {f"{motor}.pos": float for motor in self.bus.motors}

@cached_property
def observation_features(self) -> dict[str, type | tuple]:
    return {**self._motors_ft, **self._cameras_ft}

@cached_property
def action_features(self) -> dict[str, type]:
    return self._motors_ft
```

**`.pos` 접미사에 주목.** 나중에 `.vel`, `.effort` 같은 걸 추가할 수 있게 열어둔
명명 규약이다. 관측은 `모터 + 카메라`, 행동은 `모터만`이다.

`@cached_property`인 이유: 매 사이클 호출되므로 dict를 매번 만들면 낭비다.

### 5.3 `connect` (91~109줄)

```python
@check_if_already_connected
def connect(self, calibrate: bool = True) -> None:
    self.bus.connect()
    if not self.is_calibrated and calibrate:
        self.calibrate()
    for cam in self.cameras.values():
        cam.connect()
    self.configure()
```

**`@check_if_already_connected` 데코레이터**를 열어보라
(`src/lerobot/utils/decorators.py`). 중복 연결을 막는 가드를 데코레이터로 빼서
모든 로봇이 재사용한다.

순서가 중요하다: **버스 연결 → 캘리브레이션 → 카메라 → 설정.** 캘리브레이션은 카메라와
무관하고, `configure`는 캘리브레이션 후에 와야 한다.

### 5.4 `calibrate` — 이 파일의 백미 (115~157줄)

```python
def calibrate(self) -> None:
    if self.calibration:
        user_input = input("Press ENTER to use provided calibration file ..., or type 'c' to run calibration: ")
        if user_input.strip().lower() != "c":
            self.bus.write_calibration(self.calibration)
            return

    self.bus.disable_torque()                                    # ① 힘을 뺀다
    for motor in self.bus.motors:
        self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    input("Move to the middle of its range of motion and press ENTER....")
    homing_offsets = self.bus.set_half_turn_homings()            # ② 중앙 = 영점

    full_turn_motor = "wrist_roll"
    unknown_range_motors = [m for m in self.bus.motors if m != full_turn_motor]
    print("Move all joints ... through their entire ranges of motion.")
    range_mins, range_maxes = self.bus.record_ranges_of_motion(unknown_range_motors)   # ③ 범위 기록
    range_mins[full_turn_motor] = 0
    range_maxes[full_turn_motor] = 4095                          # ④ 무한회전 관절은 전 범위

    self.calibration = {}
    for motor, m in self.bus.motors.items():
        self.calibration[motor] = MotorCalibration(
            id=m.id, drive_mode=0,
            homing_offset=homing_offsets[motor],
            range_min=range_mins[motor], range_max=range_maxes[motor],
        )

    self.bus.write_calibration(self.calibration)                 # ⑤ 모터에 쓰고
    self._save_calibration()                                     # ⑥ 파일로도 저장
```

**캘리브레이션 절차의 표준형이다.**

| 단계 | 왜 |
|---|---|
| ① 토크 차단 | 사람이 손으로 움직일 수 있어야 한다 |
| ② 중앙에서 영점 | 가동범위 중앙을 0으로 잡으면 ±가 대칭이 된다 |
| ③ 전 범위 스윕 | 사람이 끝에서 끝까지 움직이는 동안 최대·최소를 기록 |
| ④ 무한회전 예외 | `wrist_roll`은 끝이 없으므로 엔코더 전 범위(0~4095) |
| ⑤ 모터에 쓰기 | 모터가 자기 영점을 갖게 |
| ⑥ 파일에 쓰기 | 다음 실행에서 재사용 |

> 💡 **우리 프로젝트와 대조**: HUPHY의 `calibration/store.py`가 ⑥을 담당하고,
> `commissioning.py`가 ①②⑤에 해당한다. ③(`record_ranges_of_motion`)에 해당하는
> 것이 아직 없다 — 만들 가치가 큰 기능이다.

### 5.5 `configure` (159~171줄)

```python
def configure(self) -> None:
    with self.bus.torque_disabled():                # ← 컨텍스트 매니저
        self.bus.configure_motors()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            self.bus.write("P_Coefficient", motor, self.config.position_p_coefficient)
            self.bus.write("I_Coefficient", motor, self.config.position_i_coefficient)
            self.bus.write("D_Coefficient", motor, self.config.position_d_coefficient)

            if motor == "gripper":
                self.bus.write("Max_Torque_Limit", motor, 500)     # 50% — 소손 방지
                self.bus.write("Protection_Current", motor, 250)
                self.bus.write("Overload_Torque", motor, 25)
```

**`with self.bus.torque_disabled():`** — 설정을 바꾸는 동안 토크를 끄고, **블록을
나갈 때 반드시 되돌린다.** 예외가 나도 복구된다. 이런 패턴을 익혀두면 좋다.

**그리퍼만 토크를 절반으로 제한한다.** 물체를 쥔 채 계속 힘을 주면 모터가 탄다.
주석에 "avoid burnout"이라고 적혀 있다 — 실제로 태워본 사람이 쓴 코드다.

### 5.6 `get_observation` (179~202줄)

```python
@check_if_not_connected
def get_observation(self) -> RobotObservation:
    start = time.perf_counter()
    obs_dict = self.bus.sync_read("Present_Position", num_retry=self.config.num_read_retries)
    obs_dict = {f"{motor}.pos": val for motor, val in obs_dict.items()}
    dt_ms = (time.perf_counter() - start) * 1e3
    logger.debug(f"{self} read state: {dt_ms:.1f}ms")

    for cam_key, cam in self.cameras.items():
        start = time.perf_counter()
        obs_dict[cam_key] = cam.read_latest()
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

    return obs_dict
```

**각 단계마다 소요 시간을 잰다.** 제어 루프에서 무엇이 느린지 알아야 하기 때문이다.
카메라 읽기가 대개 가장 느리다.

`read_latest()`는 "최신 프레임"이지 "지금 찍기"가 아니다. 카메라는 별도 스레드에서
계속 돌고, 제어 루프는 가장 최근 것을 가져간다. **블로킹하지 않기 위한 설계다.**

### 5.7 `send_action` — 안전 클램프 (204~230줄)

```python
@check_if_not_connected
def send_action(self, action: RobotAction) -> RobotAction:
    goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

    if self.config.max_relative_target is not None:
        present_pos = self.bus.sync_read("Present_Position", num_retry=...)
        goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
        goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

    self.bus.sync_write("Goal_Position", goal_pos)
    return {f"{motor}.pos": val for motor, val in goal_pos.items()}
```

**`max_relative_target`이 안전장치다.** 현재 위치에서 한 번에 얼마나 멀리 갈 수 있는지
제한한다. 우리 `max_cmd_delta_deg`(점프 가드)와 같은 개념이다.

주석에 이렇게 적혀 있다:
```python
# /!\ Slower fps expected due to reading from the follower.
```
**안전을 켜면 느려진다** — 현재 위치를 읽어야 하므로 통신이 한 번 더 든다. 안전과
속도의 트레이드오프가 코드에 그대로 드러난다.

```
📂 src/lerobot/robots/utils.py 의 ensure_safe_goal_position
```
```python
safe_diff = clamp(goal - present, [-max_diff, max_diff])
```
**로봇 클래스 바깥의 순수 함수**다. 그래서 모든 로봇이 재사용하고 테스트하기도 쉽다.
(우리가 `safety/`를 순수 함수로 뺀 것과 같은 이유)

### 5.8 파일 맨 끝 (241~242줄)

```python
SO100Follower = SOFollower
SO101Follower = SOFollower
```

SO-100과 SO-101은 **코드가 같다.** 이름만 다른 별칭이다. 하드웨어 차이가 설정으로
흡수된다는 뜻이다.

### ❓ 확인 질문
1. `gripper`만 `RANGE_0_100`인 이유는?
2. `configure()`가 `with torque_disabled()` 안에서 도는 이유는?
3. `max_relative_target`을 켜면 왜 fps가 떨어지나?
4. `send_action`이 요청과 다른 값을 반환할 수 있는 경우는?

---

## 6. [4단계] `MotorsBus` — 모터와의 실제 통신

```
📂 src/lerobot/motors/motors_bus.py  (1,296줄)
🎯 "관절 각도"가 어떻게 "바이트"가 되는가
```

1,296줄이지만 **전부 읽을 필요 없다.** 아래 순서로 발췌해서 본다.

### 6.1 자료형 3개 (169~191줄) — 여기부터

```python
class MotorNormMode(str, Enum):
    RANGE_0_100 = "range_0_100"
    RANGE_M100_100 = "range_m100_100"
    DEGREES = "degrees"

@dataclass
class MotorCalibration:
    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int

@dataclass
class Motor:
    id: int
    model: str
    norm_mode: MotorNormMode
    ...
```

**`MotorCalibration`의 5개 필드가 캘리브레이션의 전부다.**

| 필드 | 의미 |
|---|---|
| `id` | CAN/시리얼 상의 모터 번호 |
| `drive_mode` | 방향 뒤집기 (0/1). 우리 `sign`에 해당 |
| `homing_offset` | 영점 오프셋. 우리 `offset_deg`에 해당 |
| `range_min` / `range_max` | 가동 범위 (엔코더 카운트). 우리 `limit_lo/hi_deg`에 해당 |

**단위가 엔코더 카운트(정수)다.** 우리는 degree(실수)를 쓰는데, LeRobot은 모터가
보고하는 원시 정수를 그대로 저장한다. 어느 쪽이든 되지만 일관성이 중요하다.

### 6.2 정규화 — 가장 중요한 개념 (`_normalize`)

```python
def _normalize(self, ids_values: dict[int, int]) -> dict[int, float]:
    if not self.calibration:
        raise RuntimeError(f"{self} has no calibration registered.")

    for id_, val in ids_values.items():
        motor = self._id_to_name(id_)
        min_ = self.calibration[motor].range_min
        max_ = self.calibration[motor].range_max
        drive_mode = self.apply_drive_mode and self.calibration[motor].drive_mode

        bounded_val = min(max_, max(min_, val))          # ① 범위로 자르고

        if norm_mode is RANGE_M100_100:                  # ② 모드별 변환
            norm = (((bounded_val - min_) / (max_ - min_)) * 200) - 100
            normalized_values[id_] = -norm if drive_mode else norm
        elif norm_mode is RANGE_0_100:
            norm = ((bounded_val - min_) / (max_ - min_)) * 100
            normalized_values[id_] = 100 - norm if drive_mode else norm
        elif norm_mode is DEGREES:
            mid = (min_ + max_) / 2
            max_res = self.model_resolution_table[...] - 1
            normalized_values[id_] = (val - mid) * 360 / max_res
```

**왜 정규화하는가**

로봇마다 엔코더 해상도가 다르고 가동 범위가 다르다. 그대로 쓰면

- 정책(신경망)이 로봇마다 다시 학습돼야 하고
- 사람이 "지금 30이 무슨 뜻이지?"를 매번 물어야 한다

정규화하면 **"−100 = 한쪽 끝, +100 = 반대쪽 끝"**으로 통일된다. 모터가 바뀌어도
정책은 그대로다.

**세 모드의 차이**

| 모드 | 범위 | 언제 |
|---|---|---|
| `RANGE_M100_100` | −100 ~ +100 | 양방향 관절 (어깨, 팔꿈치) |
| `RANGE_0_100` | 0 ~ 100 | 단방향 (그리퍼 열림/닫힘) |
| `DEGREES` | 실제 각도 | 사람이 읽어야 할 때, 기구학 계산 |

**`DEGREES`만 `range_min/max`를 안 쓴다.** 중앙(`mid`)에서의 편차를 엔코더 해상도로
나눠 실제 각도를 만든다. 가동범위와 무관한 절대 각도다.

> 💡 **우리 프로젝트와 대조**: HUPHY는 정규화를 하지 않는다. `sign * raw + offset`으로
> degree를 만들 뿐이다. 이족 보행은 기구학 계산이 필요해서 실제 각도가 있어야 하기
> 때문인데, 나중에 정책 학습으로 가면 정규화 계층이 필요해질 수 있다.

### 6.3 읽기/쓰기 4종 (58~131줄, `MotorsBusBase`)

```python
def read(self, data_name: str, motor: str) -> Value
def write(self, data_name: str, motor: str, value: Value) -> None
def sync_read(self, data_name: str, motors=None) -> dict[str, Value]
def sync_write(self, data_name: str, values: dict[str, Value]) -> None
```

**`read`/`write`와 `sync_*`의 차이가 핵심이다.**

| | 대상 | 통신 횟수 | 언제 |
|---|---|---|---|
| `read`/`write` | 모터 1개 | 1회씩 | 설정 변경 (`configure`) |
| `sync_read`/`sync_write` | 여러 모터 | **1회** | 제어 루프 |

`sync_*`는 프로토콜 레벨에서 **한 번의 패킷으로 여러 모터를 처리**한다. 6개 모터를
따로 읽으면 6번 왕복하고, `sync_read`면 1번이다. 60Hz 루프에서 이 차이가 결정적이다.

`data_name`이 문자열인 것에 주목: `"Present_Position"`, `"Goal_Position"`,
`"P_Coefficient"`... 이 이름들은 **컨트롤 테이블**에 정의되어 있다.

```
📂 src/lerobot/motors/feetech/tables.py
```
를 열어보면 `{이름: (주소, 바이트수)}` 매핑이 있다. 모터 제조사가 정한 레지스터 맵이다.

### 6.4 `torque_disabled` 컨텍스트 매니저

```python
@contextmanager
def torque_disabled(self, motors=None):
    self.disable_torque(motors)
    try:
        yield
    finally:
        self.enable_torque(motors)
```

**예외가 나도 토크가 반드시 복구된다.** 설정 변경처럼 "잠깐만 힘을 빼야 하는" 작업에
쓴다.

### 6.5 벤더별 드라이버

```
motors/
├── motors_bus.py        공통 추상 + 정규화
├── feetech/             SO-100/101이 쓰는 것 (시리얼)
├── dynamixel/           로보티즈 (시리얼)
├── damiao/              CAN
└── robstride/           ★ 우리가 쓰는 것 (CAN)
```

```
📂 src/lerobot/motors/robstride/robstride.py  (1,086줄)
📂 src/lerobot/motors/robstride/tables.py     (121줄)
```

**`robstride/tables.py`를 반드시 열어보라.** 우리 프로젝트의
`src/huphy/motors/robstride/tables.py`와 같은 역할이다. 비교해보면

- LeRobot: `MOTOR_LIMIT_PARAMS[MotorType]` — 모델별 하나
- 우리: `ENCODING[Protocol][Model]` — **프로토콜 × 모델**

우리가 축을 하나 더 둔 이유는 [architecture.md](architecture.md) §2.1에 있다.
(그리고 LeRobot의 `O2` 항목은 RS02 매뉴얼과 일치하지 않는다)

### ❓ 확인 질문
1. `sync_read`와 `read`를 6번 부르는 것의 차이는? 60Hz에서 왜 중요한가?
2. `RANGE_M100_100`으로 정규화된 값 `0`은 무엇을 뜻하나?
3. `DEGREES` 모드가 `range_min/max`를 쓰지 않는 이유는?
4. `drive_mode`는 우리 코드의 무엇에 해당하나?

---

## 7. [5단계] 설정과 임포트 체계

사용자가 특히 궁금해한 부분. **CLI 한 줄이 어떻게 객체가 되는가.**

### 7.1 전체 흐름

```
$ lerobot-teleoperate --robot.type=so101_follower --robot.port=/dev/tty... --robot.id=black
                              │
                              ▼
  @parser.wrap()  (draccus)      ← CLI 문자열 → dataclass
                              │
                              ▼
  TeleoperateConfig(robot=SOFollowerRobotConfig(port=..., id="black"), ...)
                              │
                              ▼
  make_robot_from_config(cfg.robot)   ← 팩토리: 설정 타입 → 클래스
                              │
                              ▼
  SOFollower(config)
```

### 7.2 `draccus.ChoiceRegistry` — 다형성의 열쇠

```
📂 src/lerobot/robots/config.py
```
```python
@dataclass(kw_only=True)
class RobotConfig(draccus.ChoiceRegistry, abc.ABC):
    id: str | None = None
    calibration_dir: Path | None = None

    @property
    def type(self) -> str:
        return self.get_choice_name(self.__class__)
```

```
📂 src/lerobot/robots/so_follower/config_so_follower.py:56
```
```python
@RobotConfig.register_subclass("so101_follower")
@RobotConfig.register_subclass("so100_follower")
@dataclass
class SOFollowerRobotConfig(RobotConfig, SOFollowerConfig):
    pass
```

**이 데코레이터가 `--robot.type=so101_follower`를 가능하게 한다.**

- `register_subclass("so101_follower")`가 이름 ↔ 클래스를 레지스트리에 등록
- draccus가 `--robot.type` 값을 보고 해당 dataclass를 선택
- 나머지 `--robot.xxx` 인자를 그 dataclass의 필드에 채운다

**즉 CLI 인자가 dataclass 필드에서 자동으로 나온다.** 필드를 추가하면 CLI 옵션이
공짜로 생긴다.

```python
@dataclass
class SOFollowerConfig:
    port: str                                  # --robot.port
    disable_torque_on_disconnect: bool = True  # --robot.disable_torque_on_disconnect
    max_relative_target: float | dict | None = None
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    use_degrees: bool = True
    position_p_coefficient: int = 16            # --robot.position_p_coefficient
    ...
```

> 💡 **우리 프로젝트와 대조**: HUPHY는 draccus 대신 YAML을 쓴다
> ([architecture.md](architecture.md) §6-b가 이 선택을 미결정으로 남겨뒀다).
> YAML은 의존성이 없고 파일로 버전 관리하기 좋다. draccus는 CLI가 공짜로 생긴다.

### 7.3 팩토리

```
📂 src/lerobot/robots/utils.py
```
```python
def make_robot_from_config(config: RobotConfig) -> Robot:
    if isinstance(config, KochFollowerConfig):
        from .koch_follower import KochFollower
        return KochFollower(config)
    elif isinstance(config, SOFollowerRobotConfig):
        from .so_follower import SOFollower
        return SOFollower(config)
    ...
```

**import가 함수 안에 있는 것에 주목.** 이것이 **지연 임포트(lazy import)**다.

**왜?** 로봇마다 필요한 라이브러리가 다르다. Reachy2는 `reachy2-sdk`, 유니트리는
`unitree_sdk`가 필요하다. 최상단에서 전부 import하면 **하나만 안 깔려 있어도 전체가
죽는다.** 함수 안에 두면 그 로봇을 실제로 쓸 때만 로드된다.

> 💡 **우리도 같은 문제를 만났다.** 처음에 `__init__.py`가 하위 모듈을 전부 즉시
> import해서, 순수 계층 테스트가 `python-can`을 요구했다. PEP 562 `__getattr__`로
> 지연 로딩을 넣어 해결했다. `src/huphy/motors/__init__.py`를 열어보라.

### 7.4 `__init__.py`는 얇다

```
📂 src/lerobot/robots/__init__.py  (전부)
```
```python
from .config import RobotConfig
from .robot import Robot
from .utils import make_robot_from_config

__all__ = ["Robot", "RobotConfig", "make_robot_from_config"]
```

**구체 로봇 클래스가 하나도 없다.** 추상과 팩토리만 노출한다. 구체 로봇은
`from lerobot.robots.so_follower import SOFollower`로 직접 가져가거나 팩토리를 쓴다.

```
📂 src/lerobot/motors/__init__.py
```
```python
from .motors_bus import Motor, MotorCalibration, MotorNormMode
__all__ = ["Motor", "MotorCalibration", "MotorNormMode"]
```

역시 **자료형만.** `FeetechMotorsBus`는 `from lerobot.motors.feetech import ...`로
따로 가져간다.

**규칙 (AGENTS.md에 명시되어 있다)**
```
- 같은 모듈 안 형제 파일끼리   → 상대 import  (from .sibling import X)
- 다른 모듈 간                 → 절대 import  (from lerobot.module import X)
```

`so_follower.py` 상단을 다시 보면:
```python
from lerobot.motors import Motor, MotorCalibration, MotorNormMode   # 다른 모듈 → 절대
from ..robot import Robot                                           # 형제 → 상대
from .config_so_follower import SOFollowerRobotConfig               # 같은 폴더 → 상대
```

### 7.5 스크립트 상단의 이상한 import

```python
from lerobot.robots import (  # noqa: F401
    Robot, RobotConfig, bi_openarm_follower, ..., so_follower, ...
)
```

**`# noqa: F401`**은 "안 쓰는 import지만 린터 경고를 끄라"는 뜻이다.

**왜 안 쓰는 걸 import하나?** 모듈을 import해야 그 안의
`@RobotConfig.register_subclass(...)` 데코레이터가 **실행되어** 레지스트리에 등록된다.
등록이 안 되면 `--robot.type=so101_follower`를 인식하지 못한다.

**부작용을 노린 import**다. 처음 보면 반드시 헷갈리는 패턴이니 기억해둘 것.

### ❓ 확인 질문
1. `--robot.type=so101_follower`가 어떻게 클래스로 바뀌나? 세 단계로 설명해보라.
2. `make_robot_from_config` 안에 import가 있는 이유는?
3. `# noqa: F401`이 붙은 import를 지우면 무슨 일이 일어나나?

---

## 8. [6단계] `Teleoperator` — 거울상 확인

```
📂 src/lerobot/teleoperators/teleoperator.py   (208줄)
📂 src/lerobot/teleoperators/so_leader/so_leader.py  (167줄)
```

`so_leader.py`를 `so_follower.py`와 **나란히 놓고 비교**하면 5분이면 끝난다.

### 8.1 같은 것

- `__init__`의 모터 dict가 **완전히 동일** (같은 하드웨어)
- `connect`, `calibrate`, `configure`가 거의 같음
- 둘 다 `FeetechMotorsBus`를 쓴다

### 8.2 다른 것

| | Follower (로봇) | Leader (조종기) |
|---|---|---|
| 핵심 메서드 | `get_observation()` + `send_action()` | `get_action()` |
| 버스 사용 | `sync_read` **와** `sync_write` | `sync_read`만 |
| 토크 | 켠다 (움직여야 하므로) | **끈다** (사람이 손으로 움직이므로) |
| 카메라 | 있음 | 없음 |

```python
# so_leader.py:146
def get_action(self) -> dict[str, float]:
    action = self.bus.sync_read("Present_Position", num_retry=...)
    return {f"{motor}.pos": val for motor, val in action.items()}
```

**리더는 자기 현재 위치를 읽어서 그대로 행동으로 내놓는다.** 사람이 리더 팔을 30도
굽히면 → 리더가 30을 읽고 → 팔로워가 30으로 간다. 이게 텔레오퍼레이션의 전부다.

### 8.3 `send_feedback` (155줄)

```python
def send_feedback(self, feedback: dict[str, float]) -> None:
    goals = {k.removesuffix(".pos"): v for k, v in feedback.items() if k.endswith(".pos")}
    if goals:
        self.bus.sync_write("Goal_Position", goals)
```

리더에게 **역으로 위치를 쓸 수도 있다.** 팔로워가 벽에 부딪혔을 때 리더도 안 움직이게
하는 **햅틱 피드백**이 가능해진다.

`teleoperate.py:197`에서 유니트리 G1만 이걸 쓰는 것을 볼 수 있다.

### ❓ 확인 질문
1. 리더의 토크를 켜면 어떻게 되나?
2. 리더와 팔로워의 캘리브레이션이 다르면 무슨 일이 생기나?

---

## 9. [7단계] 기록과 데이터셋

```
📂 src/lerobot/scripts/lerobot_record.py  (552줄) — record_loop 부분만
```

`teleop_loop`과 비교하면 **데이터셋 관련 코드만 추가**된 것을 알 수 있다.

```python
obs = robot.get_observation()
obs_processed = robot_observation_processor(obs)

if dataset is not None:
    observation_frame = build_dataset_frame(dataset.features, obs_processed, prefix=OBS_STR)

act = teleop.get_action()
act_processed_teleop = teleop_action_processor((act, obs))
robot_action_to_send = robot_action_processor((act_processed_teleop, obs))
sent_action = robot.send_action(robot_action_to_send)

# ... dataset.add_frame(observation_frame | action_frame)
```

**`observation_features`가 여기서 쓰인다.** `build_dataset_frame`이 로봇의 스키마
선언을 보고 데이터셋 행을 만든다. 4.4절에서 "왜 필요한가"라고 했던 것의 답이다.

### 개념

| 용어 | 의미 |
|---|---|
| **frame** | 한 시점의 `{관측 + 행동}` |
| **episode** | 한 번의 시연 (수백 frame) |
| **dataset** | 여러 episode |
| **task** | 자연어 지시 (`"pick up the red cube"`) |

```
📂 src/lerobot/datasets/lerobot_dataset.py
```
는 12,000줄짜리 영역이라 지금은 개념만 알고 넘어가도 된다. 다만
**비디오로 저장한다**는 점은 알아둘 만하다 — 이미지를 프레임마다 PNG로 두면 용량이
폭발하므로 mp4로 인코딩하고 읽을 때 디코딩한다.

---

## 10. [8단계] 정책과 학습 (선택)

지금 단계에서는 **개념만** 알면 된다.

```
policies/
├── act/          Action Chunking Transformer — 가장 많이 쓰임
├── diffusion/    Diffusion Policy
├── smolvla/      Vision-Language-Action
└── pi0/ ...
```

모든 정책은 `PreTrainedPolicy`를 상속하고 `nn.Module` + `HubMixin`이다.
Hugging Face Hub에 올리고 받을 수 있다는 뜻.

```
📂 lerobot/AGENT_GUIDE.md
```
에 정책 선택 기준, 학습 시간, 데이터 수집 요령이 사용자 관점에서 잘 정리되어 있다.
**한 번 읽어볼 가치가 있다.** 특히:

> "Good data beats clever models." — 좋은 데이터가 영리한 모델을 이긴다
> "Start small, then extend" — 50개 에피소드로 시작해 ACT를 학습시키고, 실패를
> 분석한 뒤 한 축씩 다양성을 늘려라

---

## 11. 핵심 개념 정리

읽으면서 반복적으로 나오는 개념들.

### 11.1 관측(observation) vs 행동(action)

```
observation = 로봇이 알려주는 것  (현재 각도, 카메라 이미지)
action      = 로봇에게 시키는 것  (목표 각도)
```

**같은 "각도"라도 방향이 반대다.** `shoulder_pan.pos`가 관측에 있으면 "지금 여기",
행동에 있으면 "여기로 가라"는 뜻이다.

### 11.2 정규화(normalization)

원시 엔코더 값 → 통일된 범위. 로봇이 달라도 정책이 같게 만드는 장치.
**캘리브레이션이 있어야만 가능하다** (`range_min/max`가 필요하므로).

### 11.3 캘리브레이션 vs 설정(configure)

| | 캘리브레이션 | 설정 |
|---|---|---|
| 저장 위치 | 모터 플래시 + 파일 | 휘발 (매번 다시) |
| 빈도 | 조립 시 1회 | 연결할 때마다 |
| 예 | 영점, 가동범위, 방향 | 제어 모드, PID 게인 |

우리 프로젝트는 이걸 `commissioning.py`(영구)와 `configure()`(런타임)로 파일까지
나눴다.

### 11.4 제어 주기(control loop)

```
목표 fps → 주기 = 1/fps → 그 안에 읽기+계산+쓰기를 끝내야 함
```

넘치면 실제 주기가 늘어난다. 그래서 **모든 단계의 소요 시간을 잰다.**

### 11.5 안전장치의 종류

| 종류 | 예 | 위치 |
|---|---|---|
| 상대 이동 제한 | `max_relative_target` | `send_action` |
| 토크 제한 | `Max_Torque_Limit` | `configure` |
| 연결 상태 가드 | `@check_if_not_connected` | 데코레이터 |
| 종료 시 토크 차단 | `disable_torque_on_disconnect` | `disconnect` |
| 컨텍스트 복구 | `torque_disabled()` | 컨텍스트 매니저 |

**우리 프로젝트는 여기에 더해** 한계 클램프, E-STOP, 근접 감쇠, 360도 wrap 해소가
있다. 이족 보행이 팔보다 위험하기 때문이다 (넘어지면 로봇 전체가 손상된다).

---

## 12. HUPHY와의 대조표

우리 코드를 읽을 때 참고.

| 개념 | LeRobot | HUPHY |
|---|---|---|
| 로봇 추상 | `robots/robot.py` | `src/huphy/robots/base.py` |
| 구체 로봇 | `robots/so_follower/so_follower.py` | `src/huphy/robots/leg.py` |
| 설정 스키마 | `robots/config.py` (draccus) | `src/huphy/config/robot.py` (dataclass) |
| 설정 값 | CLI 인자 | `config/robot.yaml` |
| 캘리브레이션 자료형 | `MotorCalibration` (엔코더 카운트) | `MotorCalibration` (degree) |
| 캘리브레이션 저장 | `~/.cache/.../{id}.json` | `config/calibration/*.json` |
| 모터 버스 | `motors/motors_bus.py` | `src/huphy/motors/base.py` + `canbus.py` |
| 벤더 드라이버 | `motors/robstride/robstride.py` (1,086줄 단일) | `motors/robstride/` (5파일 분리) |
| 사양 테이블 | `[모델]` | **`[프로토콜][모델]`** |
| 안전 함수 | `robots/utils.py` 함수 1개 | `safety/` 패키지 (wrap/limits/guards) |
| 제어 루프 | 스크립트 안 `while` | `control/loop.py` |
| 궤적 생성 | 없음 (정책이 매 프레임 목표를 줌) | `control/trajectory.py` |
| 텔레메트리 | Rerun / Foxglove | `telemetry/` (UDP → PlotJuggler) |
| 팩토리 | `make_robot_from_config` | `robots/factory.py` |
| 지연 임포트 | 팩토리 함수 안 | `__init__.py`의 `__getattr__` |

### 왜 다른가 — 3가지

**1. 팔 vs 다리**
LeRobot의 팔은 정책이 매 프레임 목표를 주므로 궤적 생성이 필요 없다. 우리는 아직
정책이 없어서 램프를 직접 만들어야 한다.

**2. 시리얼 vs CAN**
Feetech/Dynamixel은 시리얼 데이지체인이고 `sync_read`가 프로토콜에 내장되어 있다.
CAN은 브로드캐스트 버스라 "보내고 나중에 드레인"하는 패턴이 된다.

**3. 위험도**
팔은 잘못 움직여도 팔만 다친다. 이족 로봇은 넘어지면 전체가 손상되므로 안전 계층이
훨씬 두껍다.

---

## 13. 직접 해볼 것 (연습)

하드웨어 없이 할 수 있는 것들.

### 연습 1 — 호출 경로 추적 (30분)
`teleop_loop`의 `robot.send_action(...)` 한 줄에서 시작해, 실제 바이트가 나가는
지점까지 파일을 따라가며 **호출 경로를 손으로 그려라.**

```
send_action (so_follower.py:205)
  → ensure_safe_goal_position (robots/utils.py)
  → bus.sync_write (motors_bus.py)
    → _unnormalize
    → ...?
```

### 연습 2 — 새 관절 추가 상상 (20분)
SO-101에 7번째 관절을 추가한다면 **어느 파일들을 고쳐야 하나?** 목록을 만들어보라.
(힌트: `so_follower.py`의 모터 dict, 캘리브레이션, 정책의 출력 차원)

### 연습 3 — 두 tables.py 비교 (20분)
```
lerobot/src/lerobot/motors/robstride/tables.py
src/huphy/motors/robstride/tables.py
```
나란히 열고 차이를 정리하라. 특히 `MOTOR_LIMIT_PARAMS`와 `ENCODING`의 구조 차이,
그리고 RS02의 값이 왜 다른지.

### 연습 4 — 우리 코드에 LeRobot 패턴 적용 (60분)
`record_ranges_of_motion`이 우리에게 없다. `motors_bus.py`에서 그 함수를 읽고,
HUPHY의 `commissioning.py`에 추가한다면 어떤 시그니처가 될지 설계해보라.

### 연습 5 — 시뮬레이터로 실행 (90분)
```bash
cd lerobot
uv sync --extra all
# 가짜 로봇으로 테스트 (하드웨어 불필요)
uv run pytest tests/robots -v
```
`tests/mocks/`를 열어보면 하드웨어 없이 테스트하는 방법이 나온다. 우리 프로젝트에서도
쓸 수 있는 기법이다.

---

## 14. 함정과 주의

### 처음 보면 반드시 헷갈리는 것들

**① `# noqa: F401` import**
안 쓰는 것 같지만 지우면 `--robot.type`이 동작하지 않는다. 데코레이터 실행이 목적.

**② `SO100Follower = SOFollower`**
파일 끝의 별칭. 두 로봇이 같은 코드를 쓴다는 뜻.

**③ `@cached_property` vs `@property`**
매 사이클 불리는 것은 캐시한다. 값이 변하면 안 되는 것에만 쓸 것.

**④ `.pos` 접미사**
`"shoulder_pan"`과 `"shoulder_pan.pos"`가 섞여 나온다. 버스 레벨은 접미사 없이,
로봇 레벨은 접미사 있게. `removesuffix(".pos")`가 곳곳에 보이는 이유.

**⑤ 정규화된 값과 원시 값**
`sync_read("Present_Position")`이 돌려주는 것은 **정규화된 값**이다
(`normalize=True`가 기본). 원시 엔코더 값이 필요하면 `normalize=False`.

**⑥ `robot.name`은 클래스 변수**
인스턴스 이름이 아니라 로봇 **종류** 이름이다. 인스턴스 구분은 `robot.id`.

### 우리 프로젝트를 읽을 때

**⑦ LeRobot의 값을 그대로 믿지 말 것**
`robstride/tables.py`의 `MotorType.O2` 인코딩 범위는 RS02 공식 매뉴얼과 일치하지
않는다. **1차 출처는 항상 제조사 매뉴얼이다.**
([architecture.md](architecture.md) 부록 참고)

---

## 15. 참고 문서 지도

### LeRobot 저장소 안

| 파일 | 내용 |
|---|---|
| `AGENT_GUIDE.md` | **사용자 관점 워크플로.** 데이터 수집 요령, 정책 선택, 학습 시간 |
| `AGENTS.md` / `CLAUDE.md` | 개발자 규약. 모듈 책임, import 규칙 |
| `docs/source/il_robots.mdx` | 모방학습 로봇 튜토리얼 |
| `docs/source/integrate_hardware.mdx` | **새 하드웨어 추가하기** — 우리에게 직접 유용 |
| `docs/source/cameras.mdx` | 카메라 설정 |
| `docs/source/feetech.mdx`, `damiao.mdx` | 벤더별 모터 가이드 |
| `docs/source/hardware_guide.mdx` | 하드웨어 개요 |
| `examples/` | 실행 가능한 예제들 |

### 우리 저장소 안

| 파일 | 내용 |
|---|---|
| [architecture.md](architecture.md) | 계층 설계와 근거 |
| [refactor_layering.md](refactor_layering.md) | 분리 작업 기록, 원본↔신규 대응표 |
| [monitoring.md](monitoring.md) | 무엇을 모니터링하고 왜 |
| [option3_control_analysis.md](../leg_control/docs/option3_control_analysis.md) | **제어 이론 실전 사례.** PD 제어, 오차, 토크 상한 |
| `src/huphy/*/README.md` | 폴더별 구성요소 설명 |

> `option3_control_analysis.md`는 **로봇 제어 입문자에게 특히 좋은 교재**다.
> "명령을 어떻게 만드느냐가 낼 수 있는 토크를 결정한다"는 것을 실제 버그를 통해
> 보여준다. LeRobot을 읽기 전에 읽어도 좋다.

---

## 16. 학습 순서 요약 (체크리스트)

```
□ 1. lerobot_teleoperate.py의 teleop_loop 읽기 (188~237줄)
□ 2. robot.py의 추상 속성/메서드 목록 파악
□ 3. so_follower.py 전체 읽기 ← 가장 중요
     □ __init__의 모터 dict
     □ calibrate()의 6단계
     □ configure()의 torque_disabled 패턴
     □ send_action()의 안전 클램프
□ 4. motors_bus.py 발췌
     □ Motor / MotorCalibration / MotorNormMode
     □ _normalize의 세 모드
     □ sync_read vs read
□ 5. config.py + register_subclass + make_robot_from_config
□ 6. so_leader.py를 so_follower.py와 비교
□ 7. lerobot_record.py의 record_loop (dataset 부분만 추가됨)
□ 8. AGENT_GUIDE.md 통독
□ 9. 연습 1~4
□ 10. 우리 src/huphy/ 폴더별 README 읽으며 대조
```

**1~4단계까지가 핵심이다.** 여기까지 하면 "로봇을 코드로 어떻게 움직이는가"가
정리되고, 우리 프로젝트 코드도 훨씬 잘 읽힌다.
