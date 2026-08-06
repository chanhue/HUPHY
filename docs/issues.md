# 이슈 로그

재작성을 진행하면서 발견한 버그·불일치·미확인 사항 기록.

각 항목: **무엇 / 근거 / 영향 / 조치**

| 상태 | 뜻 |
|---|---|
| 🔴 미해결 | 아직 손대지 않음 |
| 🟡 재작성 중 반영 | 다시 쓰는 단계에서 고칠 예정 |
| 🟢 해결 | 고침 |
| ⚪ 확인 필요 | 실물로 확인해야 판정 가능 |

---

## 🟢 #1 — 발목 IK 두 모터 각도의 규약

**해결됨** — `kinematics/ankle.py` 가 두 각도를 모두 `[-180, 180)` 으로 냄.

### 무엇이었나

두 모터 각도를 서로 다른 범위로 내면 한쪽이 모터의 보고 범위와 어긋남.

| | IK 반환 | 모터가 보고 (`zero_sta=1`) |
|---|---|---|
| `a1` (m11) | `[0, 360)` | `[-180, 180)` |
| `a2` (m12) | `[-180, 180)` | `[-180, 180)` |

`a1` 이 -5도 근처면 355도가 되는데, 그 값은 모터의 한계 밖임. IK 가 340도를
돌려주고 모터가 -20도를 보고하면 **360도 차이**가 남.

### 근거

RS02 매뉴얼 p.27 `0x7029 zero_sta` — 1로 설정 시 위치 보고 범위가 `-π ~ π`.
본 프로젝트는 `zero_sta = 1` 임.

### 조치

`_wrap180()` 하나로 두 각도를 같이 접음. `TestSolveIk` 의
`test_both_angles_are_in_one_convention` 이 시험 자세 8개에 대해 고정함.

이 규약이 맞아떨어져서 각도 감싸기 처리(`safety/wrap.py`)가 필요 없어짐 — 모든
관절 한계가 ±180 안에 있고 전 회전을 하지 않으므로, 두 공간의 규약만 맞으면 됨.

---

## 🟡 #13 — 발목 기하값의 출처가 확인되지 않음

**발견**: 6단계 `kinematics/ankle.py` 를 작성하면서

### 무엇

`AnkleGeometry` 의 기본값은 **실물 발목 하나를 실측한 좌표**임.

    a1     (-26.10,  74.000, -255.00)
    a2     (-80.90,  74.000, -313.00)
    origin (-53.50,  74.000, -393.00)
    c1      ( -8.10, 111.343, -398.00)
    c2     (-98.90, 111.343, -398.00)

세 가지가 확인되지 않음.

1. **어느 쪽 다리인지** — 왼쪽인지 오른쪽인지 기록이 없음
2. **반대쪽은 실측이 아님** — `mirrored()` 는 좌우 대칭을 가정한 계산 결과임
3. **회전 방향** — `rotation_sign_1/2` 가 실물과 맞는지 확인된 바 없음

### 영향

**왼다리와 오른다리 중 한쪽은 틀린 기하로 돌게 됨.**

기하가 틀리면 IK 가 내놓는 모터각이 실제로 요구한 자세를 만들지 않음. 계산은
멀쩡히 되고 에러도 안 나므로 **발이 엉뚱한 각도로 서 있는 것을 눈으로 봐야 알 수
있음.**

`rotation_sign` 이 반대면 발목이 목표에서 멀어지는 방향으로 감.

### 지금 확인되는 것

계산의 자기일관성뿐임. `tests/test_ankle.py` 가 고정하는 것:

    IK -> FK 왕복이 정확함
    시험 범위 격자 187개에서 다중해 문제가 없음
    거울상.solve_ik(pitch, -roll) == -원본.solve_ik(pitch, roll)
    두 각도가 같은 규약

**이 전부가 기하값이 틀려도 성립함.** 자기일관성은 정확성이 아님.

### 조치

실물에서 확인할 것.

    1. 어느 다리의 발목인지 도면·조립 기록에서 찾기
    2. commission nudge ankle_a1 -> 발이 어느 쪽으로 도는지 눈으로 확인
    3. IK 로 pitch 10도를 명령하고 실제 발 각도를 재기
    4. 반대쪽 다리를 실측해 mirrored() 결과와 대조

4번 전까지 왼다리 발목은 신뢰할 수 없음. 왼다리는 아직 연결되지 않았으므로
당장 막는 것은 없음.

---

## 🟢 #2 — 한계값이 raw 공간인지 cal 공간인지

**결론: cal 공간.** `motors/base.py` 의 `Motor.limits_deg` 로 확정함.
설정 파일 반영은 4~5단계에 남음.

### 두 공간

raw 는 모터가 CAN 으로 보고하는 각도임. 기준점은 그 모터의 기계 영점이고, 그건
사람이 `0xFE` 를 누른 그 순간의 자세임.

cal 은 사람과 기구학이 말하는 관절 각도임. 기준점은 우리가 정한 자세임.

    cal = sign * raw + offset

어긋나는 원인은 둘임.

**기준 자세가 다름 (`offset`)** — 다리가 무거워 `0xFE` 를 정확히 편 자세에서 누를
수 없음. 무릎을 12도 굽은 상태에서 영점 잡으면:

| 물리적 자세 | raw | cal |
|---|---|---|
| 완전히 편 상태 | -12 | 0 |
| 12도 굽힘 (영점 잡은 자세) | 0 | 12 |
| 하드스톱 | 62.79 | 74.79 |

**회전 방향이 다름 (`sign`)** — 양다리는 거울상이라, 같은 굽힘에 대해 한쪽은 CW
한쪽은 CCW 로 돔.

| 물리적 자세 | 오른 무릎 raw | 왼 무릎 raw | cal (양쪽 동일) |
|---|---|---|---|
| 편 상태 | 0 | 0 | 0 |
| 45도 굽힘 | +45 | -45 | 45 |

이것이 cal 공간이 존재하는 이유임. 보행 궤적이 "무릎 45도" 라고 하면 양다리가 같은
동작을 함. raw 로 말하면 다리마다 부호를 뒤집어야 하고, 그걸 잊는 자리가 코드
곳곳에 생김.

감속비 항은 없음 — RobStride 는 출력축 각도를 보고하므로 배율이 필요 없음.

### 무엇이 문제였나

한계값을 raw 로 볼지 cal 로 볼지 정해지지 않아, 비교하는 쪽마다 다른 공간의 값을
맞대고 있었음. 현재 12개 모터 전부 미실측이라 `sign=1, offset=0` 이고, 따라서
`cal == raw` 로 두 공간이 같은 숫자임. **어느 쪽으로 해석해도 지금은 똑같이
동작하므로 드러나지 않음.** 5단계에서 실측값을 넣는 순간 갈라짐.

### 검토한 것 -- LeRobot 은 raw 에 둠

`lerobot/motors/motors_bus.py:175`

    @dataclass
    class MotorCalibration:
        id: int
        drive_mode: int      # sign
        homing_offset: int   # offset
        range_min: int       # 한계 -- raw 엔코더 틱
        range_max: int

한계 검사가 따로 없고 `_unnormalize` 가 겸함 (`motors_bus.py:897`). 정규화 좌표를
`[-100, 100]` 으로 자르면 raw 가 자동으로 `[range_min, range_max]` 안에 들어옴.
한계를 넘는 명령을 만들 방법 자체가 없는 구조임.

`ensure_safe_goal_position` (`robots/utils.py:91`) 은 점프 가드 하나뿐이고 위치
한계와 무관함.

**LeRobot 에서 raw 가 안전한 이유**: 캘리브레이션이 하나의 절차라서 사람이 관절을
끝까지 훑는 동안 offset 과 range 가 같이 기록됨. 한쪽만 갱신하는 것이 불가능함.

### 왜 그대로 가져올 수 없나

LeRobot 의 range 는 훑어서 얻는 값이고, 본 프로젝트의 값은 설계에서 오는 값임.

| | SO-100 | 휴머노이드 다리 |
|---|---|---|
| 손으로 역구동 | 됨 | 중력·감속비 때문에 안 됨 |
| 하드스톱까지 훑기 | 안전 | 그게 피하려는 사고임 |
| 관절 독립성 | 각각 독립 | 발목은 두 모터 폐루프 |

발목이 결정적임. `a1` 을 혼자 끝까지 돌리면 링크가 물리므로 **두 모터를 따로 훑는
절차가 성립하지 않음.** 기록 방식이 여기서 막힘.

따라서 한계는 첫 동작 **전에** 알고 있어야 하고, 그건 설계값임. 설계값은 영점을
어디에 잡았는지와 무관하므로 cal 공간임.

    LeRobot   한계 = 측정 결과   ->  offset 과 한 몸  ->  raw
    HUPHY     한계 = 설계 입력   ->  offset 과 무관   ->  cal

하드스톱은 쇳덩어리라 움직이지 않는데, raw 로 적으면 영점을 다시 잡을 때마다 숫자가
바뀜. 영점을 3도 다른 자세에서 다시 잡으면 무릎 하드스톱의 raw 는 62.79 에서 59.79
가 되지만 cal 은 74.79 로 그대로임.

### 조치

`Motor.limits_deg` (cal 공간, `Optional`) 로 확정함. `MotorCalibration` 은 실측값
(`sign`/`offset`/`zero_reference`) 만 담음.

`lo < hi` 를 `__post_init__` 에서 검사함. cal 공간에는 `sign` 이 개입하지 않아 순서가
뒤집히지 않으므로, 변환할 때마다 `min/max` 로 재정렬할 필요가 없어짐.

`None` 은 "제한 없음" 이 아니라 "아직 모름" 임. `Motor.is_configured` 가 False 가
되어 상위에서 제어 진입을 막음.

**남은 작업 (4~5단계)**

    config/calibration/*.json   limit_lo_deg / limit_hi_deg 제거
    config/robot.yaml           모터별 limits 추가 (cal 공간)
    config/robot.yaml           헤더 주석 수정 -- 현재 "실측값(... limits ...)은
                                calibration/*.json 에 있다" 로 되어 있음

### 함께 확인된 것 -- LeRobot 에서 가져오지 않을 것

`DEGREES` 모드에는 한계 검사가 없음 (`motors_bus.py:874`). `range_min/max` 를 중점
계산에만 쓰고 자르지 않음. `RANGE_M100_100` 에서 공짜로 얻던 보호가 도 단위를 쓰는
순간 사라짐. **본 프로젝트는 도 단위를 씀.**

NaN 검사는 전 계층에 없음.

이 둘이 `safety/guards.py` 를 별도 관문으로 두는 이유임.

---

## 🔴 #3 — `calibrate()`가 `Robot` ABC 계약에 없음

### 무엇

`SingleLeg`에는 `calibrate()`가 있지만 `Robot` 추상 클래스가 요구하지 않는다.

### 근거

`git show main:src/huphy/robots/base.py` — 추상 메서드 목록에 없음.
LeRobot의 `robots/robot.py:141`은 `@abc.abstractmethod def calibrate()`로 강제한다.

### 영향

새 로봇(`Biped` 등)이 `calibrate()`를 안 만들어도 객체 생성이 통과한다.
그리고 `connect(calibrate=True)`가 그걸 부르는 순간 `AttributeError`로 죽는다.

**빠뜨린 것을 객체 생성 시점에 잡는 게 ABC의 존재 이유인데, 이 메서드는 그물에서 빠져 있다.**

### 조치

6단계에서 `Robot` ABC에 추가. 한 줄이면 된다.

---

## 🔴 #4 — 텔레메트리가 브링업 메뉴에 연결되지 않음

### 무엇

`control/loop.py`(`LegControlLoop`)를 **어디에서도 import하지 않는다.**
연쇄적으로 `SingleLeg.send_action()`, `telemetry_snapshot()`, `RejectCounters`가
실제로는 한 번도 실행되지 않는다.

### 근거

import 그래프 추출 결과 — `control/loop.py`로 들어오는 화살표가 없음.
`scripts/bringup.py`는 `TelemetrySink`를 만들지만 `sink.close()`만 부른다.

### 영향

**지금 상태로 브링업 메뉴를 써도 PlotJuggler에 데이터가 흐르지 않는다.**
게인 튜닝을 할 수 없다.

[monitoring.md](monitoring.md) §4.1-(6)에서 미리 지적한 "메뉴 경로는 제어 루프를
타지 않는다" 문제와 같다.

### 조치

9단계에서 메뉴가 제어 루프를 타게 하거나, `_move_to` 루프 안에서
`leg.telemetry_snapshot()`을 만들어 sink로 넘긴다.

---

## 🔴 #5 — `side` 하나가 종류·개체·기하 세 가지를 겸하고 있음

### 무엇

세 개념이 서로 다른데 `config.side` 하나로 뭉쳐 있다.

| 개념 | 뜻 | 예 |
|---|---|---|
| **종류** (`name`) | 코드가 같은 것들 | `"single_leg"` |
| **개체** (`id`) | 이 물리적 부품 | `"left"`, `"right"`, `"right_spare"` |
| **기하** (`side`) | 좌우 미러링 | `"left"`, `"right"` |

현재 `side`가 쓰이는 곳:

| 위치 | 실제로는 어느 개념 |
|---|---|
| `AnkleKinematics(side)` | **기하** — `x → -x` 미러링 |
| `calibration_path` 선택 | **개체** |
| 로그 `f"[{self.config.side}]"` | **개체** |
| `name = f"{side}_leg"` | **종류 + 개체 뒤섞임** |

그리고 `name`이 클래스 변수가 아니라 추상 property다.

```python
SingleLeg.name        # <property object>  ← 객체 없이 알 수 없다
leg.name              # "right_leg"
```

### 근거

LeRobot `BiSOFollower.__init__`이 하위 로봇에 id를 만들어 내려준다:

```python
left_arm_config = SOFollowerRobotConfig(
    id=f"{config.id}_left" if config.id else None,
    port=config.left_arm_config.port,
    ...
)
self.left_arm = SOFollower(left_arm_config)
```

둘 다 `name = "so_follower"`이고, 캘리브레이션은 `{name}/{id}.json`으로 자동 분리된다.

### 영향

**양다리 + CAN 버스 2개 구성에서 바로 걸린다.** 이 프로젝트는 왼다리 can0,
오른다리 can1로 갈 예정이므로 "같은 종류 2개, 각자 다른 버스·캘리브레이션"이
정확히 type/id 상황이다.

**`side`와 `id`가 갈라지는 실제 경우 — 예비 다리 교체**

오른쪽 자리에 다른 물리적 다리를 끼우면:
- `side = "right"` — 기하는 그대로 (미러링 동일)
- `id = "right_spare"` — **캘리브레이션은 달라야 한다** (개체마다 sign/offset/limit가 다름)

지금 구조면 `side`로 캘리브레이션 파일을 고르므로 덮어쓰거나 손으로 바꿔야 한다.

그 외:
- 로봇 목록 생성, 설정 레지스트리 도입 시 `name`이 클래스 변수여야 한다

### 조치

| 필드 | 어디에 | 역할 |
|---|---|---|
| `name = "single_leg"` | **클래스 변수** | 종류 |
| `id` | `LegConfig`에 신규 | 개체. 캘리브레이션 파일명·로그·텔레메트리 접두사 |
| `side` | `LegConfig`에 유지 | **기하 전용**으로 의미 축소 |

캘리브레이션 경로를 yaml에 직접 적는 대신 **`{name}/{id}.json` 규약**으로 만들면,
상위(`Biped`)가 하위 id를 만들어 내려주는 것만으로 자동 분리된다.

```python
class Biped(Robot):
    name = "biped"
    def __init__(self, config):
        self.left_leg  = SingleLeg(LegConfig(id=f"{config.id}_left",
                                             channel="can0", side="left", ...))
        self.right_leg = SingleLeg(LegConfig(id=f"{config.id}_right",
                                             channel="can1", side="right", ...))
```

---

## 🔴 #6 — `SingleLeg`/`CanBus`에 컨텍스트 매니저·소멸자가 없음

### 무엇

`__enter__`/`__exit__`/`__del__`이 `UdpTelemetry`와 `CsvLogger`에만 있다.
**정작 토크를 물고 있는 `SingleLeg`에는 없다.**

### 영향

라이브러리로 쓸 때 예외가 나면 토크가 물린 채 남는다.

```python
leg = build_leg(cfg)
leg.connect()
leg.bus.enable_torque()
do_something()          # ← 예외
# disconnect가 안 불린다. 모터는 계속 힘을 준다.
```

이족 로봇은 토크가 물린 채 방치되면 관절이 하드스톱을 밀거나 과열된다.

### 조치

6단계에서 `SingleLeg`에 `__enter__`/`__exit__`/`__del__`,
3단계에서 `CanBus`에 `__enter__`/`__exit__` 추가.
`disconnect()`가 이미 토크 차단까지 하므로 감싸기만 하면 된다.

---

## 🔴 #10 — `send_action`이 계산·전송·수거를 분리하지 않아 양다리에서 동기가 틀어짐

**발견**: 왼다리 can0 / 오른다리 can1 구성을 논의하면서

### 무엇

현재 `SingleLeg.send_action()`이 세 가지를 한 번에 한다.

```python
def send_action(self, action):
    frames = ...                    # ① 계산 (IK, 가드, 인코딩) -- CAN 안 씀
    bus.sync_write_mit(frames)      # ② 전송 + ③ 드레인
```

버스가 하나면 문제없다. **버스가 둘이면 순차로 돌아 두 다리의 명령 시각이 벌어진다.**

```
can0 (왼다리)  [계산]████████ 0.8ms
can1 (오른다리)         [계산]████████ 0.8ms
                        ↑ 왼다리 계산 + 전송이 끝난 뒤에야 시작
```

### 근거

- CAN 프레임 하나 ≈ **0.13 ms** (1 Mbps, 11-bit ID + 8바이트 + 스터핑)
- 6개 ≈ 0.8 ms. 여기에 계산 시간(발목 IK 포함)이 더해진다
- 100 Hz(10 ms)에서 8% 이상

두 버스는 **물리적으로 독립**이라 진짜 병렬 전송이 가능한데, 지금 구조로는 못 살린다.

### 영향

서 있기만 하면 무시할 만하다. **보행 중 양다리 동기가 필요한 국면에서 유의미하다.**

지금은 다리가 하나라 무해하다.

### 조치

세 단계를 분리한다.

```python
# Biped.send_action
left_frames  = self.left_leg.build_frames(left_action)     # ① 계산 (둘 다 먼저)
right_frames = self.right_leg.build_frames(right_action)

self.left_leg.bus.send(left_frames)                        # ② 전송 (연속)
self.right_leg.bus.send(right_frames)

self.left_leg.bus.drain()                                  # ③ 수거
self.right_leg.bus.drain()
```

**드레인이 더 큰 문제다** — `drain()`은 큐가 빌 때까지 읽고 마지막 `recv`가
타임아웃만큼 블로킹된다. 두 버스를 순차 드레인하면 2배다.

**버스마다 RX 전용 스레드**를 두면 제어 루프에서 드레인이 사라지고 계산+전송만 남는다.
다리 하나일 때는 선택이지만 양다리에서는 필요에 가까워진다.
(→ `src/huphy/motors/README.md`의 "RX 처리 3단계")

---

## 🟢 #7 — 모터의 통신 프로토콜 모드

**해결됨** — 사용자가 MIT 프로토콜로 설정 완료함.

### 무엇이었나

RobStride 공장 기본값은 **private(29-bit 확장 프레임)**인데 이 코드는
**MIT(11-bit 표준 프레임)**로 보냄.

### 왜 위험했나

안 맞으면 **명령이 무시되고 에러도 나지 않음.** 연결도 되고 코드도 안 죽는데
모터만 안 움직임 — 진단이 가장 어려운 종류의 실패임.

같은 조합(RobStride + LeRobot)으로 휴머노이드를 만든 다른 프로젝트가 정확히 이
함정에 빠짐. 모터가 응답도 안 하고 에러도 안 내서 원인을 찾는 데 오래 걸렸다고 함.

### 근거

RS02 매뉴얼 p.27 §4 "Protocol Switching" — `protocol_1` 또는 Type 25로 전환,
**재부팅 필요**.

### 남은 것

모터를 **교체하거나 추가할 때마다** 다시 설정해야 함. 팔·상체로 확장하면 모터가
20개를 넘어가므로 스크립트로 일괄 확인·설정하는 것이 필요해짐.

`tables.PARAM_PROTOCOL_FLAG (0x201F)` 를 읽어 확인할 수 있음.

전제 문서는 3단계에서 `motors/README.md` 에 작성함.

---

## 🔴 #11 — 하드웨어 전제를 코드로 확인할 수 없음

**발견**: 4단계 커미셔닝을 작성하면서

### 무엇

`motors/README.md` 의 하드웨어 전제 네 가지 중 둘을 코드로 읽을 방법이 없음.

| 전제 | 파라미터 | 읽을 수 있나 |
|---|---|---|
| MIT 프로토콜 | `PARAM_PROTOCOL_FLAG` `0x201F` | **불가** |
| `zero_sta = 1` | `PARAM_ZERO_STA` `0x7029` | **불가** |

### 근거

`tables.py` 의 파라미터 절에 적혀 있음.

    # 파라미터 인덱스 (private type 17/18로 접근)

파라미터 읽기·쓰기는 **29-bit 확장 프레임(private 프로토콜)** 을 씀. 이 코드는
11-bit 표준 프레임(MIT)만 보내므로 접근 경로가 없음.

`codec/private.py` 는 미구현임.

LeRobot 도 마찬가지임. `robstride/tables.py` 에 `CAN_CMD_QUERY_PARAM`(0x33),
`CAN_CMD_WRITE_PARAM`(0x55), `CAN_CMD_SAVE_PARAM`(0xAA), `CAN_PARAM_ID`(0x7FF) 를
정의해 두었으나 `robstride.py` 어디에서도 쓰지 않음. 참고할 구현이 없음.

### 영향

**설정이 맞는지 코드가 모른 채로 동작함.** 프로토콜이 어긋나면 명령이 무시되고
에러도 나지 않으므로, 이 확인이 없으면 "모터가 안 움직인다" 의 원인 후보가 배선,
전원, CAN id, 프로토콜 넷으로 남음.

`scan()` 이 응답 유무는 보지만 **응답 없음과 프로토콜 불일치를 구분하지 못함.**

모터 수가 늘어날수록 커짐. 다리 하나는 외부 도구로 여섯 번 확인하면 되지만,
팔·상체까지 가면 20개가 넘음.

### 현재 대응

MotorStudio 같은 외부 도구로 확인함. 사용자가 MIT 프로토콜로 설정 완료함 (#7).

### 조치

`codec/private.py` 를 구현하면 해소됨. 필요한 것:

    파라미터 읽기 / 쓰기 / 플래시 저장의 29-bit 프레임 배치
    확장 프레임의 id 필드 구성 (모터 id 가 어디에 실리는지)

매뉴얼 p.26~27 "Read and write a single parameter list" 참조. 단 그 절에는 오기재가
있음 (`docs/architecture.md:275` 참조).

우선순위는 낮음 — 지금은 외부 도구로 되고, 모터가 늘어날 때 다시 볼 것.

---

## 🟢 #12 — 기계 영점(`0xFE`)의 지속성

**해결됨** — 사용자가 실물에서 확인함. 전원을 끊었다 켜도 영점이 유지됨.

### 무엇이었나

`tables.py` 의 주석에 지속성에 대한 언급이 없었음.

    CMD_SET_ZERO = 0xFE      # Command 4. 비위치 모드에서만 동작

같은 모터의 `PARAM_ZERO_STA`(`0x7029`) 는 별도의 저장 명령이 있어야 유지되므로
(`tables.py` 주석의 "Type 22로 저장해야 유지됨"), `0xFE` 도 그럴 가능성이 있었음.

### 왜 중요했나

**절차가 통째로 달라짐.**

| 남는 경우 | 안 남는 경우 |
|---|---|
| 조립할 때 한 번 잡으면 끝 | 전원을 켤 때마다 다시 잡아야 함 |
| `offset` 실측이 의미 있음 | 매번 같은 자세를 재현해야 함 |
| `zero_reference` 메모가 재조립용 | 메모가 매일 쓰는 절차서가 됨 |

안 남는 쪽이었다면 `zero` 를 커미셔닝이 아니라 시작 절차에 넣어야 했음.

### 결론

남으므로 `commissioning.py` 에 두는 것이 맞음. `zero_reference` 메모는 모터를
교체하거나 재조립할 때 같은 자세를 재현하는 데 씀.

---

## ⚪ #8 — 모터 id ↔ 관절 매핑 실물 미확인

### 무엇

원본 `robot_constant.py`는 주석과 코드가 달랐다.

| 위치 | 매핑 |
|---|---|
| 상단 주석, `JOINT_LIMITS_DEG` 주석 | `7=hipz 8=hipx 9=hipy` ← 맞다고 확인됨 |
| `_make_leg_specs`, `hip_knee_ids` 언패킹 | `7=hipy 8=hipx 9=hipz` ← 틀림 |

`config/robot.yaml`은 주석 쪽(`7=hipz 9=hipy`)으로 정정했다.

### 영향

틀리면 `set_leg_action(hipz=30)`이 **엉뚱한 관절을 움직인다.**

### 조치

**실물에 토크를 걸기 전 4단계(커미셔닝)에서 한 모터씩 살짝 움직여 확인.**

---

## ⚪ #9 — 캘리브레이션 미실측 (`kp = 0`)

### 무엇

`config/calibration/right_leg.json`의 `kp`, `kd`, `sign`, `offset_deg`,
`zero_reference`가 전부 기본값이다. 한계값만 실측으로 보인다.

### 영향

**`kp = 0`이면 토크가 항상 0이다.** 지금 상태로 실물에 명령을 보내면
프레임은 나가는데 모터가 전혀 힘을 주지 않는다.

`is_complete()`가 `kp > 0`을 요구하는 이유가 이것이다.

### 조치

5단계에서 `sign`/`limit`/`zero_reference` 실측,
`kp`/`kd`는 **7단계(텔레메트리) 이후** 튜닝.

---

## 🟢 해결된 것

| # | 무엇 | 어떻게 |
|---|---|---|
| A | `ramp_profile`이 `round`를 써서 스텝이 `max_step_deg`를 초과 (10°를 3°씩 → 3.33°/스텝) | `ceil`로 수정. 테스트가 잡았다 |
| B | MIT 프로토콜의 RS02 `vmax`가 44로 잘못됨 (private 값) | 33으로 정정. 매뉴얼 p.37~38 근거. 속도 읽기가 1.33배 크게 나오던 문제 |
| C | `hipz`/`hipy` 매핑이 뒤바뀜 | `robot.yaml`에서 7↔9, 1↔3 교체 |
| D | `__init__.py` 조기 import로 순수 계층 테스트가 `python-can`을 요구 | PEP 562 `__getattr__` 지연 로딩 |
| E | LeRobot `tables.py`의 `MotorType.O2` 값이 RS02 매뉴얼과 불일치 | 우리 `tables.py`를 `[프로토콜][모델]`로 인덱싱해 구조적으로 방지 |
| F | **한계값이 하드스톱인데 `state` 여유를 바깥으로 더하고 있었다** | ↓ |
| G | **한계를 넘는 명령을 버려서 다리 자세가 어긋났다** | ↓ |

### F — 여유 방향이 잘못됨

**발견**: 1단계 `limits.py` 작성 중

원본 `state_in_bounds`가 여유를 **더했다**.

```python
return (lo - margin) <= deg <= (hi + margin)     # 한계 밖 5도까지 허용
```

이건 `limits`가 하드스톱보다 **안쪽**이라는 가정이다. 원본 주석도 그랬다:

```python
JOINT_LIMITS_DEG  # ... with margin before the physical hard-stop
```

**그런데 실측값은 하드스톱 그 자체다.** 그러면 하드스톱을 5도 넘어야 판정이
걸리는데 물리적으로 불가능하니 **위치로는 영원히 안 걸린다.**

**수정** — `limits`는 하드스톱 실측값이고, 여유는 하드스톱에서 **안쪽 방향**으로만
뺀다.

```
하드스톱                                    하드스톱
   |---3도---|                    |---3도---|
   |      명령 허용 구간           |
   lo                            hi
```

여유를 두는 이유는 셋이다. **명령을 하드스톱에 정확히 두면 부딪힌다.**

| | 왜 |
|---|---|
| 오버슛 | PD 제어는 목표를 지나친다. kd가 충분해도 0은 아니다 |
| 관성 | 빠르게 움직이는 중에 명령을 멈춰도 바로 서지 않는다 |
| 측정 오차 | 하드스톱 실측이 실제보다 크면 여유 없이는 닿는다 |

**3도는 임의값이다.** 게인 튜닝 후 목표를 한계 근처로 보내 텔레메트리에서
오버슛 크기를 재고, 거기에 안전계수를 더해 정해야 한다.

원본에 있던 `state`(E-STOP) 여유는 제거했다. E-STOP은 사람이 전원을 끊는 것이고
코드가 대신할 수 없다 — 코드가 고장나면 코드 안전장치도 같이 고장난다. 이름을
그렇게 붙이면 물리 버튼 없이도 안전하다는 착각을 만든다.

### G — 거부 대신 클리핑

**발견**: 같은 시점

원본은 한계를 넘으면 **프레임을 아예 안 보냈다** (`return None` -> `continue`).

**문제**: 6개 중 일부만 거부되면 그 모터만 직전 명령을 유지해 **다리 자세가
어긋난다.** 발목처럼 2모터가 연동된 곳에서 특히 나쁘다 — a1만 거부되면
pitch/roll이 엉뚱해진다.

또 정책이 계속 범위 밖을 요구하면 조용히 멈춘 채이고, MIT 타임아웃이 켜져
있으면 모터가 풀린다.

**수정** — 클리핑할 대상이 있으면 자르고, 없으면 안 보낸다:

| 사유 | 처리 | 왜 |
|---|---|---|
| 한계 초과 | **클리핑** | 한계까지는 갈 수 있다. 연속성 유지 |
| 점프 과대 | **클리핑** | max_delta만큼만 = 속도 제한 |
| 유효 상태 없음 | 전송 안 함 | 기준이 없어 클리핑 불가 |
| 발목 IK 불가 | 전송 안 함 | 클리핑할 각도 자체가 없다 |
| E-STOP | 전송 안 함 | 토크를 끊는 게 목적 |

점프 클리핑이 특히 중요하다 — 거부하면 먼 목표에 **영영 도달 못 하고**,
클리핑하면 `max_delta`씩 슬루해서 도달한다. LeRobot도 클리핑이다
(`ensure_safe_goal_position`).

클리핑은 **조용한 변조**이므로 `clamp()`가 `(값, 잘렸는지)`를 함께 돌려준다.
텔레메트리 필드도 나눈다:

```
clips_limit   clips_jump                       잘렸다 (명령은 나감)
rejects_nostate   rejects_ik   rejects_estop   안 보냈다
```

---

## 기록 규칙

- 발견하면 **바로 여기 적는다.** "나중에 고치지" 하고 넘기면 잊는다
- **근거를 남긴다** — 코드 위치, 매뉴얼 페이지. 나중에 "왜 이게 문제였지"를 반드시 묻게 된다
- 고치면 지우지 말고 **🟢 해결로 옮긴다.** 같은 실수를 반복하지 않기 위해
