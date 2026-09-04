# 아키텍처 — 계층과 의존 방향

패키지가 어떻게 나뉘어 있고 **왜 그렇게 나뉘었는지**. 각 폴더의 내용은 소스 옆
README 에 있고, 여기는 폴더들 사이의 관계만 다룸.

읽는 순서: [루트 README](../README.md) → 이 문서 → 폴더별 README.

---

## 1. 의존은 한 방향으로만 흐름

```
scripts/                진입점
   │
control/                주기와 안전
   │
robots/                 ─── 관절 이름 ↔ 모터 id, cal ↔ raw ───
   │  ├─ kinematics/
   │  ├─ safety/
   │  ├─ config/
   │  └─ calibration/
   │
motors/                 모터 id 와 raw 각도
   └─ robstride/
```

**위가 아래를 부름. 아래는 위를 모름.**

`telemetry/` 는 이 줄기에서 벗어나 있음 — `Robot` 계약만 읽고 아무도 부르지 않음.
관측이 제어에 끼어들지 않게 하려는 것임.

### 계층이 아는 것

| 계층 | 아는 것 | 모르는 것 |
|---|---|---|
| `control/` | 시간, 주기 | 관절 이름도, 모터도 모름 |
| `robots/` | 관절 이름, cal 각도 | 바이트, 프레임 |
| `motors/` | 모터 id, raw 각도 | "무릎" 이 무엇인지 |
| `canbus.py` | 8바이트와 CAN id | 바이트의 뜻 |

이 표가 **가장 중요한 규칙임.** 어떤 계층이 자기 줄 밖의 것을 알기 시작하면 그
계층만으로는 시험할 수 없게 됨.

---

## 2. 순수 계산을 아래로 밀어 둠

`python-can` 을 import 하는 파일이 **하나뿐임.**

```
motors/canbus.py    ← 여기
```

그것도 함수 안에서 import 함. 모듈 최상단에 두면 이 파일을 import 하는 것만으로
`python-can` 이 필요해지고, 순수 계층의 테스트까지 같이 막힘.

그래서 다음이 전부 하드웨어 없이 돌아감.

```
safety/          한계·점프·NaN 검사
kinematics/      발목 IK/FK
config/          설정 읽기
calibration/     실측값 읽기·쓰기
motors/base.py   자료형과 인터페이스
robstride/tables.py, codec/    벤더 사양, 프레임 배치
control/motions.py             무엇을 시킬지
```

**테스트 642개가 `python-can` 없이 돌아감.** 전송 계층은 가짜 모듈로 갈아끼워
시험함.

### 왜 이렇게까지 하나

로봇을 만질 수 있는 시간은 짧고, 만지는 동안은 코드를 고치기 어려움. 계산이 맞는지를
책상에서 확정해 두면 실물에서는 **배선·프로토콜·기하만** 보면 됨.

---

## 3. `robots/` 가 경계인 이유

관절 이름과 모터 id 는 다른 세계임.

```
control/    "무릎 30도"           사람과 궤적이 쓰는 말
robots/     ─── 여기서 번역 ───
motors/     "m10 에 62.79도"      배선과 프로토콜이 쓰는 말
```

네 가지가 **이 경계에서만** 일어남.

| | 무엇 |
|---|---|
| 관절 이름 → 모터 id | `robot.yaml` 의 매핑 |
| cal → raw | 캘리브레이션의 `sign`/`offset` |
| 발목 pitch/roll → a1/a2 | 기구학 |
| 한계·점프·NaN 검사 | `safety.guards` (cal 공간) |

한 군데라도 다른 계층으로 새면 같은 변환이 두 곳에 생기고, **한쪽만 고쳐짐.**

### 검사가 변환보다 먼저임

한계는 cal 공간에 있음. raw 로 내린 뒤 검사하면 `sign` 이 -1 인 관절에서 부호가
뒤집혀 **한계가 반대로 걸림.**

```python
leg.build_commands({"knee": 200.0})     # sign = -1

last_sent["knee"]            →   71.79    cal 로 잘림
commands[10].position_deg    →  -71.79    그 뒤에 raw
```

---

## 4. `motors/` 를 더 쪼갠 이유

```
motors/
├── base.py       벤더 중립 자료형
├── canbus.py     CAN 전송
└── robstride/
    ├── tables.py         벤더 사양
    ├── codec/mit.py      프레임 배치
    ├── bus.py            런타임 조작
    └── commissioning.py  조립할 때 한 번
```

### 인코딩 범위는 "프로토콜 × 모델" 임

같은 RS02 라도 프로토콜에 따라 속도 범위가 다름.

| 조합 | 위치 | 속도 | 토크 |
|---|---|---|---|
| RS02 / MIT | ±12.57 rad | **±44 rad/s** | ±17 N·m |
| RS02 / private | ±12.57 rad | **±44 rad/s** | ±17 N·m |
| RS00 / MIT | ±12.57 rad | ±33 rad/s | ±14 N·m |

이 축을 없애고 "모델별 사양" 하나로 뭉치면 **private 값을 MIT 에 가져다 쓰는
실수가 남.** 틀리면 속도 읽기가 44/33 = 1.33배 어긋남.

`base.py` 에 인코딩 범위를 두지 않은 이유이기도 함 — MIT 류 특유의 개념이라
벤더 중립이 아님. CANopen 계열은 pulse 단위를 쓰고 인코딩 범위라는 개념 자체가 없음.

### 커미셔닝과 런타임은 성격이 정반대임

| | `bus.py` | `commissioning.py` |
|---|---|---|
| 언제 | 매 주기 100Hz | 조립할 때 한 번 |
| 되돌리기 | 쉬움 | **어려움** |
| 무엇 | 토크, 명령, 상태 | 영점, CAN id, 프로토콜 |

`MotorsBus` 계약에 커미셔닝 조작이 없으므로 제어 코드에서 **부를 방법 자체가 없음.**

### 모션 명령 말고도 경로가 있음

```
동작 제어    MIT 프레임 8바이트          매 주기
제어 명령    [0xFF]*6 + F_CMD + 명령     활성·정지·고장
파라미터     29-bit 확장 프레임           미구현
```

세 번째가 아직 없어서 `zero_sta` 와 프로토콜 플래그를 코드로 읽을 수 없음
([이슈 #11](issues.md)).

---

## 5. 설정과 실측값을 나눈 이유

```
config/robot.yaml           사람이 적는 것    도면·배선·튜닝에서 옴
config/calibration/*.json   기계가 재는 것    실물을 측정해서 나옴
```

무릎 모터를 갈면 다시 재야 하는 것과 그대로인 것이 갈림.

| | 모터를 갈면 |
|---|---|
| `sign`, `offset_deg`, `zero_reference` | **다시 재야 함** |
| `limits_deg` | 다시 재야 함. 기계 영점이 옮겨감 |
| `kp`, `kd` | 그대로. 같은 모델이면 그대로 씀 |

**한 파일에 두면 한쪽을 고칠 때 다른 쪽을 덮어씀.** 캘리브레이션 절차는 파일을
통째로 새로 쓰므로, 그때 안 바뀌어야 할 값까지 같이 날아감.

### 한계값이 cal 공간에 있는 이유

관절 가동범위는 기구 설계에서 오는 값이라 **첫 동작 전에 이미 알고 있어야 함.**
움직여 보고 알아내는 값이 아님 — 다리를 손으로 하드스톱까지 훑는 것이 중력과
감속비 때문에 불가능하고, 발목은 두 모터가 폐루프로 물려 있어 한쪽만 훑으면 링크가
물림.

설계값이므로 **영점을 어디에 잡았는지와 무관함.** raw 에 두면 영점을 다시 잡을 때
숫자는 그대로인데 가리키는 물리적 위치가 달라짐 ([이슈 #2](issues.md)).

---

## 6. 계산·전송·수거를 나눈 이유

```python
left  = left_leg.build_commands(action_left)     # ① 계산. CAN 안 씀
right = right_leg.build_commands(action_right)
left_leg.send(left)                              # ② 전송을 몰아서
right_leg.send(right)
left_leg.collect()                               # ③ 그 다음에 수거
right_leg.collect()
```

한 함수가 셋을 다 하면 버스가 둘일 때 이렇게 됨.

```
can0 (왼다리)  [계산][전송][─── 수거 대기 ───]
can1 (오른다리)                                [계산][전송]
                                               ^ 여기서야 시작
```

두 버스는 물리적으로 독립이라 진짜로 겹쳐 보낼 수 있는데 그 병렬성을 못 씀.
**수거가 더 비쌈** — `recv()` 는 큐가 비면 타임아웃만큼 블로킹함.

다리 하나뿐이면 `send_action()` 하나로 충분함 ([이슈 #10](issues.md)). 양다리를
묶는 `robots/biped.py` 가 이 순서를 지휘하고, 버스마다 수신 스레드를 두어 수거
대기를 겹침 ([full_robot.md](full_robot.md)).

---

## 7. 관측이 제어 줄기에서 벗어나 있는 이유

```
telemetry.snapshot.build(robot)     Robot 계약만 읽음. 통신하지 않음
```

`get_observation()`, `last_sent`, `counters` 만 씀. 그래서 **어느 경로에서 부르든
같은 값이 나옴** — 제어 루프든, 대화형 메뉴든.

그리고 **예외를 던지지 않음.** 네트워크가 끊기거나 디스크가 차는 것은 로봇 입장에서
정상 상황임. 관측이 제어를 멈추면 관측할 대상이 없어짐.

### 필드 이름을 한 곳에서만 정함

UDP 와 CSV 가 같은 사전을 소비함. 두 군데에서 만들면 CSV 헤더에는 있는데 UDP 에는
없는 값이 생기고, 어느 쪽이 맞는지 알 수 없어짐.

`field_names()` 가 **실행 전에** 목록을 냄 — CSV 헤더를 첫 줄에 써야 하기 때문임.

---

## 8. 확장

### 팔이 붙으면

```
robots/
├── leg.py        지금
├── arm.py        추가
└── humanoid.py   Leg 둘과 Arm 둘을 들고 있음
```

`Humanoid` 도 같은 `Robot` 계약을 채움. `build_commands`/`send`/`collect` 를 나눠
둔 것이 그때 쓰임.

`config/schema.py` 는 그대로임 — `limbs` 에 항목이 늘어날 뿐이고,
`kind: arm` 으로 골라낼 수 있음.

### 벤더가 추가되면

```
motors/
├── base.py       그대로
├── canbus.py     그대로 (CAN 을 쓰는 벤더면)
├── robstride/
└── 새벤더/
```

`Motor.model` 이 `str` 인 이유임 — 여기서 enum 으로 좁히면 벤더를 추가할 때마다
중립 계층을 고쳐야 함. 유효한 모델인지는 벤더 버스가 생성자에서 판단함.

### 버스가 늘어나면

`LimbConfig` 하나가 CAN 채널 하나에 대응함. 팔다리를 추가하면 채널도 같이 늘어남.

`RobotConfig` 가 **채널을 넘어서** id 중복을 확인함 — 다른 팔다리라도 같은 채널이면
같은 선을 씀.

---

## 9. 미결정

| | 왜 아직 |
|---|---|
| 수신 전용 스레드 | 다리 하나에서는 순차 수거로 충분함. 양다리에서 다시 볼 것 |
| `codec/private.py` | 파라미터 읽기·쓰기가 필요해지면 ([이슈 #11](issues.md)) |
| 보행 궤적 | 서 있기가 되고 나서 |
| 진단 패킷 주기 설정 | 지금은 코드 기본값(10주기). 필요해지면 `robot.yaml` 로 |

---

## 10. 참고

| | |
|---|---|
| [루트 README](../README.md) | 사용법, 조정할 값, 계층별 설계 근거 |
| [flow_diagrams.md](flow_diagrams.md) | 호출 관계를 그림으로 |
| [issues.md](issues.md) | 미해결 항목과 근거 |
| [monitoring.md](monitoring.md) | 무엇을 왜 보는가 |
| `src/huphy/*/README.md` | 폴더별 상세 |
