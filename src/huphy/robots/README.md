# `robots/` — 관절 이름과 모터 id 를 잇는 곳

```
robots/
├── base.py    Robot 계약
├── leg.py     다리 하나
└── biped.py   팔다리 여럿을 로봇 하나로
```

`base` 와 `biped` 는 버스를 모름. `leg` 는 버스를 쓰므로 경로를 명시해 가져감.

`biped` 는 `leg` 를 고치지 않고 위에 얹힘 — 다리는 자기가 묶였다는 것을 모름.

---

## 여기가 경계임

```
control/    "무릎 30도"          관절 이름, cal 공간
robots/     <-- 경계 -->
motors/     "m10 에 62.79도"     모터 id, raw 공간
```

이 경계에서 네 가지가 일어남. **넷 다 여기서만 일어남.**

```
관절 이름 -> 모터 id        robot.yaml 의 매핑
cal -> raw                 캘리브레이션의 sign/offset
발목 pitch/roll -> a1/a2    기구학
한계·점프 검사              safety.guards (cal 공간)
```

위 계층은 관절 각도만 다루고, 아래 계층은 바이트만 다룸.

---

## `base.py` — `Robot` 계약

### 구조를 먼저 내놓게 함

```python
joint_names            이 로봇이 아는 관절
observation_features   무엇을 관찰할 수 있는지
action_features        무엇을 명령할 수 있는지
```

**실행 전에 알 수 있어야 함.** 텔레메트리와 기록이 이 목록으로 열을 만드는데,
실행해 보고 나서야 필드를 알면 로그 형식이 매 실행마다 달라짐.

### 캘리브레이션이 계약에 있음

```python
is_calibrated    제어에 쓸 만큼 실측값이 채워졌는지
calibrate()      실측값을 다시 읽어 들임
```

각 로봇이 제멋대로 판정하면 호출부가 **"이 로봇은 준비됐나" 를 물어볼 공통 방법이
없어짐** (이슈 #3). 미실측 상태로 토크를 넣는 것이 가장 위험한데, 그걸 막는 판정이
구현마다 다르면 막히지 않는 구현이 하나쯤 생김.

`calibrate()` 는 파일을 다시 읽는 것까지만 함. **재는 절차 자체는 사람이 함** —
`scripts/commission.py`.

### 계산·전송·수거를 나눔

```python
build_commands(action)   계산만. CAN 을 쓰지 않음
send(commands)           전송만
collect()                수거만
send_action(action)      셋을 합친 것
```

버스가 둘일 때 이 순서를 짜려면 나뉘어 있어야 함 (이슈 #10).

```python
left  = left_leg.build_commands(action_left)     # ① 계산을 먼저 다
right = right_leg.build_commands(action_right)
left_leg.send(left)                              # ② 전송을 몰아서
right_leg.send(right)
left_leg.collect()                               # ③ 그 다음에 수거
right_leg.collect()
```

다리 하나뿐이면 `send_action()` 하나로 충분함.

### `send_action` 이 실제로 나간 것을 돌려줌

명령한 것과 다를 수 있음 — 한계나 점프에 걸리면 잘림.

**무엇을 보냈는지가 아니라 무엇이 실행됐는지**를 기록해야 나중에 로그를 믿을 수 있음.

---

## `leg.py` — 다리 하나

### 관절과 모터가 1:1 이 아님

```
hip_pitch  hip_roll  hip_yaw  knee     모터 하나가 관절 하나
ankle_pitch  ankle_roll    모터 두 개가 관절 두 개를 만듦
```

명령은 **관절 6개**로 받고, 발목만 기구학을 거쳐 모터 두 개로 풀림.

```python
leg.joint_names   ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll')
leg.motor_names   ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_a', 'ankle_b')
```

설정에는 모터가 `ankle_a`/`ankle_b` 로 있음. **사람과 보행 궤적이 다루는 것은
관절이지 링키지가 아님.**

### 한 주기에 일어나는 일

```
1  관절 목표 (cal)
2  발목 pitch/roll -> a1/a2          기구학. 닫힌 해
3  모터별 cal 목표
4  한계·점프 검사                     safety.guards. cal 공간
5  cal -> raw                        캘리브레이션
6  MIT 프레임                         코덱
7  전송
8  수거 -> 상태 갱신
```

**4번이 cal 공간인 이유**: 한계가 cal 공간에 있음 (이슈 #2). raw 로 내린 뒤에
검사하면 `sign` 이 -1 인 관절에서 부호가 뒤집혀 한계가 반대로 걸림.

### 발목의 두 가지 실패는 다름

**IK 가 안 풀리면 통째로 버림.** 두 모터 다 직전 명령을 유지함. 한쪽만 새 명령을
받으면 두 로드가 서로 다른 자세를 요구해 관절이 비틀림.

**한계에 잘리는 것은 개별로 처리해도 됨.** 로드가 둘이고 자유도가 둘이라 어떤
`(a1, a2)` 조합이든 대응하는 발 자세가 하나 있음 — 잘린 각도 쌍도 비틀린 자세가
아니라 그냥 다른 자세임.

실제로 실행된 자세는 FK 로 되짚어 `last_sent` 에 냄.

```python
leg.build_commands({"ankle_pitch": 10.0, "ankle_roll": 5.0})
leg.last_sent    # {'ankle_pitch': 7.065, 'ankle_roll': 2.479}  <- a2 가 한계에 걸림
```

나머지 관절은 서로 독립이라 한 관절이 잘려도 다른 관절은 정상임.

### 발목은 pitch 와 roll 을 함께 줘야 함

```python
leg.build_commands({"ankle_pitch": 5.0})
# ValueError: 발목은 pitch 와 roll 을 함께 줘야 함
```

모터 두 개가 두 자유도를 같이 만들기 때문임. 하나만 주면 나머지가 뭔지 알 수 없음.

### 관찰은 모터 단위 + 발목각, 명령은 관절 단위

```python
leg.action_features        {'hip_pitch': float, ..., 'ankle_pitch': float, 'ankle_roll': float}
leg.observation_features   {'knee.pos': float, ..., 'ankle_a.pos': float,
                            'ankle_pitch.pos': float, 'ankle_roll.pos': float,
                            'stale_motors': int}
```

모터 단위가 기본인 이유: **실제로 측정되는 것이 그것임.** 발목 `pitch`/`roll` 만
FK 로 푼 값이고, 모델이 관절각을 입력으로 받으므로 같이 냄.

FK 는 `collect()` / `refresh()` 에서 **주기당 한 번** 풂. `get_observation()` 은 한
주기에 여러 번 불려서 거기서 풀면 같은 계산을 세 번 함. 한 번에 155us 임.

```python
leg.ankle_pose()    # (pitch, roll) 또는 None. 풀어 둔 것을 꺼내기만 함
leg.ankle_velocity()  # (dpitch, droll) 도/초. 모터 속도를 J^-1 로 풂
```

직전 결과를 추정으로 씀 — 같은 모터각 조합이 서로 다른 자세 둘에 대응하므로
추정이 가까워야 함. 못 풀면 `ankle_pose()` 가 `None` 이고, 관찰에는 마지막으로 알려진
자세가 나감.

### 발목만 토크로 보낼 수 있음

```python
Leg(limb, bus, ankle_output="torque", ankle_kp=(20.0, 20.0), ankle_kd=(0.5, 0.5))
```

MIT 프레임은 하나고 칸이 다섯임 (`q`, `dq`, `kp`, `kd`, `tau_ff`). 두 가지 명령이
있는 것이 아니라 같은 프레임을 다르게 채우는 것임.

```
position   q=목표각  kp=30  kd=1   tau_ff=0      모터 펌웨어가 PD 를 함
torque     q=0       kp=0   kd=0   tau_ff=tau    모터는 시킨 토크만 냄
```

나머지 4관절은 모터 1개 = 관절 1개라 늘 위치임. 발목만 갈리는 이유는 모터 각도에
게인을 걸면 로드 지렛대 비가 자세마다 달라 발목이 내는 강성이 자세마다 달라지기
때문임. 관절 토크를 만들어 야코비안으로 내리면 그 비가 보정됨.

`ankle_kp` / `ankle_kd` 는 **관절 각도**에 거는 게인임 (`(pitch, roll)`, Nm/rad).
모터 각도에 거는 `Motor.gains` 와 다른 값임.

한 실행 안에서 바뀌지 않음 — 브링업은 위치로, 정책 실행은 토크로 만듦.

지금 자세를 못 풀었으면 아무것도 안 보내고 `rejects["ankle_nopose"]` 를 셈. 야코비안이
지금 자세에서 나오므로 자세를 모르면 토크를 만들 수 없음.

보낸 토크는 `last_torque` 에 남음. 모터가 보고하는 `MotorState.torque_nm` 과 짝임 —
앞은 우리가 시킨 값, 뒤는 모터가 냈다는 값.

**한계 가드가 걸리지 않음.** `guards.apply` 는 목표 각도를 받아 자르는데 토크 명령에는
넘길 각도가 없음. 토크 쪽 가드는 아직 없음.

### 미실측이면 토크를 거부함

```python
leg.enable()
# RuntimeError: right_leg: 실측값이 채워지지 않음.
#   미설정 관절 (...), 미실측 관절 (...). 확인했으면 allow_uncalibrated=True 로 만들 것
```

**둘 다 봄** — 한계·게인이 채워졌는지(`config.is_configured`)와 영점 메모가
남아 있는지(`unmeasured`). 게인만 있으면 어디까지 가도 되는지 모르고, 한계만
있으면 토크가 안 나감.

`allow_uncalibrated=True` 면 경고만 하고 진행함. **커미셔닝 단계에서 필요함** —
실측 전에도 움직여야 실측을 할 수 있음.

### `hold()`

```python
leg.send(leg.hold())
```

목표를 현재 위치로 두므로 오차가 0이 되어 자세를 붙잡기만 함. 정지 직전이나
대기에 씀.

### 예외로 빠져나가도 토크가 끊김

```python
with leg:
    leg.enable()
    raise RuntimeError(...)     # 여기서 터져도 정지 명령이 나감
```

이게 없으면 모터가 마지막 명령을 계속 유지함 — 명령을 갱신하는 코드가 죽었으므로
사람이 전원을 뽑을 때까지 다리가 힘을 주고 있음 (이슈 #6).

---

## 거울상 다리

```python
right = Leg(cfg_right, bus_right)
left  = Leg(cfg_left, bus_left,
            kinematics=AnkleKinematics(AnkleGeometry().mirrored()))
```

같은 관절 명령에 양다리가 같은 물리 동작을 해야 함. 두 군데가 맞아야 성립함.

```
관절 각도의 부호     캘리브레이션의 sign
발목 링키지          AnkleGeometry.mirrored()
```

`sign` 은 재서 얻는 값이고 링키지 거울상은 계산이라, 서로 다른 파일에 있음.

---

## 쓰는 법

```python
from huphy.config import load_robot
from huphy.motors.canbus import CanBus
from huphy.motors.robstride.bus import RobStrideBus
from huphy.robots.leg import Leg

robot = load_robot("config/robot.yaml")
cfg = robot.limb("right_leg")
bus = RobStrideBus(CanBus(cfg.channel), cfg.motors_by_id())

with Leg(cfg, bus, safety=robot.safety) as leg:
    leg.enable()
    leg.send_action({"knee": 30.0, "ankle_pitch": 5.0, "ankle_roll": 0.0})
    leg.get_observation()["knee.pos"]
```

---

## 미구현

| 파일 | 용도 | 필요해지는 시점 |
|---|---|---|
| `arm.py` | 팔 하나 | 팔을 붙일 때 |
| `humanoid.py` | 다리·팔을 묶은 것 | 양다리를 같이 움직일 때 |

토크 명령에 걸리는 가드도 아직 없음. 지금 가드는 목표 각도를 받아 한계로 자르는데
토크에는 넘길 각도가 없음.

`humanoid.py` 도 같은 `Robot` 계약을 채우고, 안에서 `Leg` 둘을 들고 있음.
`build_commands` / `send` / `collect` 를 나눠 둔 것이 그때 쓰임.

---

## 테스트

```bash
PYTHONPATH=src python3 -m pytest tests/test_leg.py -q
```

42개. 가짜 모터가 붙은 가짜 CAN 버스를 씀. 확인하는 것은 **경계에서 일어나는
변환과 그 순서**임.

기하값이 실제 로봇과 맞는지는 여기서 확인되지 않음 (이슈 #13).

---

## `biped.py` — 팔다리 여럿을 로봇 하나로

`Robot` 계약을 채우므로 제어 루프는 이것이 다리 하나인지 양다리인지 모름.

```python
Biped([right_leg, left_leg], id="huphy")
```

맡는 것은 셋뿐임. 변환은 여전히 `Leg` 가 함.

```
이름 구분      right_leg/knee      무릎이 둘이라 구분이 필요함
전송 순서      전부 보낸 뒤 수거    두 다리의 명령 시각이 벌어지지 않게
전부 아니면 아무것도   연결·토크    반쪽 상태로 진행하지 않음
```

### 구분자가 `/` 인 이유

관찰 필드가 이미 `knee.pos` 처럼 `.` 을 씀. 같은 문자를 쓰면
`right_leg.knee.pos` 에서 어디까지가 팔다리 이름인지 갈리지 않음.

### 명령 꾸러미가 팔다리별로 나뉨

```python
{"right_leg": {10: MitCommand(...)}, "left_leg": {4: MitCommand(...)}}
```

모터 id 는 **버스 안에서만** 유일함. 채널이 다르면 같은 id 가 양쪽에 있을 수
있어, 한 사전에 담으면 한쪽이 조용히 덮임.

### 반쪽 상태를 만들지 않음

| | 하나가 실패하면 |
|---|---|
| `connect` | 이미 연 것을 닫고 예외를 올림 |
| `enable` | 이미 넣은 토크를 끊고 예외를 올림 |
| `disconnect` | 나머지를 마저 닫음 |
| `disable` | 나머지를 마저 끊음 |
| `collect` | 나머지를 마저 수거하고, 실패한 쪽은 무응답으로 셈 |

앞의 둘은 **한 다리만 힘이 들어간 로봇**을 막는 것이고, 뒤의 셋은 **한 다리에
토크가 남거나 상태가 멈추는 것**을 막는 것임. 방향이 반대라 처리도 반대임.
