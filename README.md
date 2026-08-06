# HUPHY

휴머노이드 로봇 제어. RobStride 액추에이터를 CAN 으로 제어함.

지금은 **다리 하나**(모터 6개, CAN 채널 1개)가 동작하고, 팔·상체가 붙어도 같은
구조가 그대로 늘어남.

```bash
sudo ip link set can1 up type can bitrate 1000000

python -m huphy.scripts.commission --limb right_leg scan
python -m huphy.scripts.bringup --limb right_leg
```

---

## 목차

- [무엇이 필요한가](#무엇이-필요한가)
- [처음부터 다리를 움직이기까지](#처음부터-다리를-움직이기까지)
- [조정해야 할 값](#조정해야-할-값)
- [지금 무엇이 비어 있나](#지금-무엇이-비어-있나)
- [계층 구조](#계층-구조)
- [한 주기에 무슨 일이 일어나나](#한-주기에-무슨-일이-일어나나)
- [왜 이렇게 나눴나](#왜-이렇게-나눴나)
- [테스트](#테스트)

---

## 무엇이 필요한가

**하드웨어**

```
라즈베리파이 + CAN HAT
RobStride RS02 x4 (고관절 3, 무릎 1)
RobStride RS00 x2 (발목 링키지)
```

**소프트웨어**

```bash
pip install -e .          # python-can, numpy, PyYAML
pip install -e ".[dev]"   # + pytest
```

**모터 설정** — 코드가 확인하지 않고 맞다고 가정하는 것들임.

| 전제 | 어긋나면 |
|---|---|
| MIT 프로토콜 (11-bit 표준 프레임) | **명령이 무시되고 에러도 안 남** |
| `zero_sta = 1` | 위치 보고가 `[0,360)` 이 되어 부호가 뒤집힘 |
| 비트레이트 1 Mbps | 통신이 아예 안 됨 |

지금은 MotorStudio 같은 외부 도구로 확인해야 함. 코드로 읽을 방법이 없음
([이슈 #11](docs/issues.md)).

---

## 처음부터 다리를 움직이기까지

```
1  CAN 채널 올리기       sudo ip link set can1 up type can bitrate 1000000
2  응답 확인             commission scan
3  고장 확인             commission fault
4  위치 확인             commission state
5  관절 매핑 확인         commission nudge <관절>      관절마다
6  영점 잡기             commission zero <관절>       관절마다
7  실측값 채우기          calibration/*.json 을 손으로
8  게인 튜닝             bringup                     그래프를 보며
```

**5번이 중요함.** 설정에는 `7=hipz 8=hipx 9=hipy 10=knee 11=ankle_a1 12=ankle_a2`
로 되어 있지만 실물로 확인된 적이 없음 ([이슈 #8](docs/issues.md)).

`nudge` 로 한 모터씩 움직여 어느 관절이 도는지 **눈으로** 봐야 함.

**6번 전에는 어느 자세가 0도인지 모름.** 영점을 안 잡으면 그 뒤가 전부 의미 없음.

### 각 단계에서 보는 것

**`commission scan`** — 응답 없는 모터가 있으면 원인 후보를 같이 냄.

```
응답 없음: ['ankle_a2']
  배선, 전원, CAN id, 프로토콜 모드가 후보임.
  이 넷은 여기서 구분되지 않음 -- 전부 조용히 빠짐.
```

**`commission nudge knee`** — 명령한 만큼 안 움직이면 알려줌.

```
시작    29.99
최대    34.72   (움직인 양 +4.73)
끝      30.23
```

**`bringup`** — 그래프를 보며 게인을 찾음.

```
1  loop_dt 부터 확인      주기를 못 지키면 게인 문제가 아님
2  자세 유지              처지나, 떨리나
3  계단 응답              여기서 대부분이 결정됨
4  사인파                 추종 지연과 진폭 감쇠
```

---

## 조정해야 할 값

### `config/robot.yaml` — 사람이 적는 것

| 값 | 언제 고치나 | 지금 |
|---|---|---|
| `limits_deg` | 기구 설계가 바뀌면 | 오른다리만 있음. 왼다리는 비어 있음 |
| `kp` / `kd` | 튜닝할 때 | **전부 0** |
| `command_margin_deg` | 게인을 바꾸면 다시 봄 | 3.0 |
| `max_delta_deg` | 주기를 바꾸면 다시 봄 | 50.0 |
| `channel` | 배선이 바뀌면 | `can1` / `can0` |
| `control_hz` | — | 100.0 |
| `telemetry.host` | 그래프를 볼 때 | 비어 있음 (UDP 꺼짐) |

**`kp`/`kd` 가 0인 이유** — 아직 아무도 안 재봤음. 그럴듯한 값을 넣어 두면 튜닝된
것처럼 보여서 그대로 실행하게 됨.

0은 "안 정해짐" 이 아니라 **"힘 없음"** 임. 명령을 보내도 아무 힘이 안 나가고,
`Motor.is_configured` 가 `False` 라 제어 진입 자체가 막힘.

### `config/calibration/*.json` — 조립을 재서 얻는 것

| 값 | 언제 고치나 | 지금 |
|---|---|---|
| `sign` | 모터를 다시 달면 | 전부 1.0 (미실측) |
| `offset_deg` | 영점을 다시 잡으면 | 전부 0.0 (미실측) |
| `zero_reference` | 영점을 잡을 때 | 전부 비어 있음 |

**두 파일을 나눈 이유** — 숫자에 두 종류가 있음.

```
도면 보고 적는다        ->  robot.yaml
로봇을 만져서 알아낸다   ->  calibration/*.json
```

무릎 모터를 갈면 `sign`/`offset` 은 다시 재야 하지만 `limits_deg` 는 그대로임 —
무릎 뼈대는 안 바꿨고 하드스톱은 쇳덩어리임. 한 파일에 두면 **한쪽을 고칠 때 다른
쪽을 덮어씀.**

자세한 것은 [`config/README.md`](config/README.md).

---

## 지금 무엇이 비어 있나

| | 무엇이 막히나 |
|---|---|
| **게인 미실측** (`kp = 0`) | 토크가 안 나감. 다리가 안 움직임 |
| **영점 미실측** (`zero_reference` 비어 있음) | `cal` 이 `raw` 와 같음. 좌표계가 없는 것과 같음 |
| **모터 매핑 미확인** | 명령한 관절이 아닌 것이 움직일 수 있음 |
| **발목 기하 출처 미확인** | 어느 다리 것인지 모름. 반대쪽은 계산으로 만든 거울상 |
| **왼다리 한계 없음** | 왼다리는 제어 진입이 막힘 |

**전부 실물이 있어야 채워지는 것들임.** 코드로 할 수 있는 것은 다 되어 있고,
`--allow-uncalibrated` 로 넘겨야 실측을 시작할 수 있음.

목록과 근거는 [`docs/issues.md`](docs/issues.md).

---

## 계층 구조

```
scripts/          터미널 진입점
   │
   ├─ commission.py   조립할 때 한 번 하는 조작
   └─ bringup.py      반복해서 움직여 보는 메뉴
   │
control/          제어 루프. 주기와 안전
   │
robots/           ─── 관절 이름 ↔ 모터 id, cal ↔ raw 경계 ───
   │
   ├─ kinematics/     발목 pitch/roll ↔ a1/a2
   ├─ safety/         한계·점프·NaN 검사
   ├─ config/         robot.yaml 읽기
   └─ calibration/    실측값 읽기·쓰기
   │
motors/           모터 id 와 raw 각도만 앎
   │
   ├─ base.py         벤더 중립 자료형
   ├─ canbus.py       CAN 전송. python-can 유일 사용처
   └─ robstride/      벤더 사양, 코덱, 버스, 커미셔닝
   │
telemetry/        옆에서 관찰. 제어를 방해하지 않음
```

### 어느 계층이 무엇을 아나

| 계층 | 아는 것 | 모르는 것 |
|---|---|---|
| `control/` | 시간, 주기 | 관절 이름도, 모터도 모름 |
| `robots/` | 관절 이름, cal 각도 | 바이트, 프레임 |
| `motors/` | 모터 id, raw 각도 | "무릎" 이 무엇인지 |
| `canbus.py` | 8바이트와 CAN id | 바이트의 뜻 |

**`robots/` 가 경계임.** 위는 관절로 말하고 아래는 모터로 말함.

### `python-can` 을 쓰는 곳

```
canbus.py    ← 여기 하나뿐
```

그 위는 `CanFrame` 만 다룸. 그래서 **테스트 642개가 `python-can` 없이 돌아감.**

---

## 한 주기에 무슨 일이 일어나나

```
ControlLoop.run()
  │
  ├─ motion(t, obs)                   무엇을 시킬지          control/motions.py
  │     -> {"knee": 30.0, ...}        관절 이름, cal 공간
  │
  ├─ leg.build_commands(action)       계산만. CAN 안 씀      robots/leg.py
  │     │
  │     ├─ 발목 pitch/roll -> a1/a2                          kinematics/ankle.py
  │     ├─ 한계·점프·NaN 검사 (cal 공간)                       safety/guards.py
  │     ├─ cal -> raw                                        calibration
  │     └─ MitCommand                                        robstride/bus.py
  │
  ├─ leg.send(commands)               전송만                 robstride/bus.py
  │     └─ pack_command -> 8바이트                            robstride/codec/mit.py
  │           └─ CanBus.send_many                            motors/canbus.py
  │
  ├─ leg.collect()                    수거. 상태 갱신
  │     └─ CanBus.drain -> decode_state
  │
  ├─ telemetry.record()               기록. 읽기만 함        telemetry/
  │
  └─ 다음 주기까지 기다림
```

### 계산·전송·수거를 나눈 이유

버스가 둘일 때 이 순서를 짜야 함.

```
왼다리 계산 -> 오른다리 계산 -> 왼다리 전송 -> 오른다리 전송 -> 수거
```

한 함수가 셋을 다 하면 **두 다리의 명령 시각이 벌어짐** — 수거는 큐가 빌 때까지
기다리므로 그 시간이 그대로 오른다리 전송 지연이 됨.

다리 하나뿐이면 `send_action()` 하나로 충분함.

---

## 왜 이렇게 나눴나

각 계층을 만들 때 무엇에 중점을 뒀는지.

### `safety/` — 조용한 실패를 막음

**NaN 하나가 720도 명령이 됨.**

```python
min(10, nan)                             # 10      비교가 False 라 통과
float_to_uint(nan, -12.57, 12.57, 16)    # 65535 = 720도
```

파이썬의 `min`/`max` 가 NaN 을 통과시키므로 인코딩 단계의 클램프가 무력화됨.
그래서 **유한값 검사가 첫 관문**임.

**버리지 않고 자름.** 명령을 버리면 그 모터만 직전 명령을 유지해 다리 자세가
어긋남 — 발목처럼 두 모터가 연동된 곳에서 특히 나쁨.

자른 것은 반드시 세어 내보냄. 클리핑은 **조용한 변조**이기 때문임.

### `motors/` — 벤더 중립과 전송 격리

**적는 것과 재는 것을 나눔.** `Motor` 는 사람이 적고 `MotorCalibration` 은 조립을
잼. 무효화 시점이 달라서 한 파일에 두면 한쪽을 고칠 때 다른 쪽을 덮어씀.

**`python-can` 을 `canbus.py` 안에 가둠.** 위 계층은 `CanFrame` 만 다룸.
`codec/mit.py` 가 라디안 변환을 혼자 떠안는 것과 같은 방식임.

**전송과 수거를 나눔.** `recv()` 는 큐가 비면 타임아웃만큼 블로킹하므로, 순차로
수거하면 그 시간이 버스 수만큼 곱해짐.

### `robstride/` — 벤더 사양을 데이터로

**프로토콜과 제어 모드는 다른 축임.** `Protocol` 은 프레임 포맷, `ControlMode` 는
무엇을 명령할지. 이름이 겹치지만 독립임.

**인코딩 범위가 `[프로토콜][모델]` 임.** 같은 RS02 라도 MIT 은 ±33 rad/s, private
은 ±44 rad/s. 이 축이 없으면 private 값을 MIT 에 가져다 쓰는 실수가 남.

**되돌리기 어려운 조작을 격리함.** 영점·CAN id·프로토콜 전환은 `commissioning.py`
로 감. `MotorsBus` 계약에 없으므로 제어 코드에서 **부를 방법 자체가 없음.**

### `config/` — 오타를 읽는 순간 잡음

YAML 은 모르는 키를 조용히 넘김.

```
contorl_hz: 200     ->  무시되고 기본값 100Hz 로 돎
```

**설정을 고쳤는데 아무것도 안 바뀜.** 증상이 "느리다" 로 나타나므로 원인을
설정에서 찾을 이유가 없어 오래 걸림. 그래서 모르는 키가 있으면 멈춤.

**기본값은 스키마에만 둠.** 두 군데 있으면 어느 쪽이 쓰이는지 알 수 없음.

### `kinematics/` — 자기일관성을 고정함

발목만 있음. 다른 관절은 모터 하나가 관절 하나를 돌리므로 변환할 것이 없음.

**두 모터 각도를 같은 규약(`[-180,180)`)으로 냄.** 한쪽만 `[0,360)` 이면 IK 가
340도를 돌려주고 모터는 -20도를 보고해 360도 차이가 남.

**FK 는 답이 하나가 아님.** 같은 모터각 조합이 서로 다른 자세 둘에 대응함.
링키지의 성질이지 버그가 아님 — 시험 범위 안에서는 문제가 없다는 것을 격자 187개로
확인함.

### `robots/` — 경계를 한 곳에 모음

네 가지가 **여기서만** 일어남: 관절 이름 → 모터 id, cal → raw, 발목 IK, 안전 검사.

**한계 검사가 cal 공간에서 일어남.** raw 로 내린 뒤 검사하면 `sign` 이 -1 인
관절에서 부호가 뒤집혀 한계가 반대로 걸림.

**실제로 나간 명령을 돌려줌.** 무엇을 보냈는지가 아니라 **무엇이 실행됐는지**를
기록해야 로그를 믿을 수 있음.

### `telemetry/` — 제어보다 먼저 만듦

게인을 튜닝하려면 목표와 실측을 겹쳐 봐야 함. **그래프가 없으면 게인을 찾을 수
없고, 게인이 없으면 다리가 안 움직임.**

**필드 이름을 한 곳에서만 정함.** 두 군데에서 만들면 CSV 헤더에는 있는데 UDP 에는
없는 값이 생김.

**예외를 던지지 않음.** 네트워크가 끊기거나 디스크가 차는 것은 정상 상황임. 관측이
제어를 멈추면 관측할 대상이 없어짐.

**패킷을 둘로 나눔.** 한 다리가 필드 66개면 MTU(1500)를 넘어 조각나고, 조각 하나만
잃어도 패킷 전체가 버려짐.

### `control/` — 주기를 정직하게 잼

**두 가지를 따로 봄.**

```
overruns   튀는 주기.     목표의 1.5배를 넘긴 횟수
kept_up    꾸준한 느림.   평균이 목표의 90% 미만
```

매 주기 24%씩 넘으면 **한 번도 "밀림" 으로 세지 않으면서** 주파수만 떨어짐.
주기가 밀리는데 게인을 튜닝하면 게인이 아니라 주기가 문제인데 게인을 계속 만지게 됨.

**마감 직전은 자지 않고 돌면서 기다림.** `time.sleep` 은 요청한 만큼 정확히 자지
않아서, 100Hz 에서 84.7Hz 가 나옴. 고치면 99.9Hz.

**멈출 때 자세를 먼저 붙잡음.** 서 있는 다리에서 힘이 갑자기 빠지면 주저앉음.
예외로 빠져나가도 같은 순서를 탐.

### `scripts/` — 메뉴가 루프를 탐

메뉴가 로봇을 직접 부르면 그 경로에서만 텔레메트리·주기 측정·정지 순서가 빠짐.
**그러면 그래프가 안 나오는데 텔레메트리가 고장난 줄 알게 됨.**

메뉴는 `Motion` 만 정하고 루프에 넘김. 테스트가 `ControlLoop.run` 을 감시해 이것을
고정함.

---

## 폴더별 문서

| | |
|---|---|
| [`config/`](config/README.md) | 설정 값. 두 파일을 나눈 이유 |
| [`src/huphy/config/`](src/huphy/config/README.md) | 설정 읽기 |
| [`src/huphy/calibration/`](src/huphy/calibration/README.md) | 실측값 읽기·쓰기 |
| [`src/huphy/safety/`](src/huphy/safety/README.md) | 명령의 최종 관문 |
| [`src/huphy/motors/`](src/huphy/motors/README.md) | 벤더 중립 자료형, CAN 전송, 하드웨어 전제 |
| [`src/huphy/motors/robstride/`](src/huphy/motors/robstride/README.md) | 벤더 사양, 코덱, 버스, 커미셔닝 |
| [`src/huphy/kinematics/`](src/huphy/kinematics/README.md) | 발목 링키지 |
| [`src/huphy/robots/`](src/huphy/robots/README.md) | 관절 ↔ 모터 경계 |
| [`src/huphy/telemetry/`](src/huphy/telemetry/README.md) | 관찰 |
| [`src/huphy/control/`](src/huphy/control/README.md) | 제어 루프, 게인 튜닝 |
| [`src/huphy/scripts/`](src/huphy/scripts/README.md) | 터미널 진입점 |
| [`tests/`](tests/README.md) | 무엇을 고정했나 |
| [`docs/issues.md`](docs/issues.md) | 미해결 항목과 근거 |

---

## 테스트

```bash
PYTHONPATH=src python3 -m pytest tests -q
```

```
642 passed in 11.0s
```

**하드웨어도 `python-can` 도 필요 없음.**

- 순수 계산 계층은 애초에 `python-can` 을 안 씀
- 전송 계층은 `import can` 이 함수 안에 있어 가짜 모듈로 갈아끼움
- 가짜 버스가 **명령에 응답함** — 실제 모터가 명령을 받은 뒤 답하는 것과 같은 순서

### 확인되지 않는 것

| | 왜 |
|---|---|
| 전송 지연, CAN 중재 | 실물의 물리 |
| 모터가 실제로 응답하는지 | 프로토콜 모드가 맞아야 함 |
| 게인 값이 적절한지 | 다리 무게와 감속비에 달림 |
| 발목 기하가 실물과 맞는지 | 발 각도를 재야 함 |
| 실제 제어 주기 | 스케줄러 정밀도와 부하 |

**자기일관성은 정확성이 아님.** 기하값이 틀려도 IK↔FK 왕복은 성립함.
