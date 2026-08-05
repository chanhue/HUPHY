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

## 🟡 #1 — 발목 IK의 `a1` 정규화 규약 불일치

**발견**: 1단계 `wrap.py` 작성 중, `zero_sta` 펌웨어 설정 확인 과정에서

### 무엇

`kinematics/ankle.py`의 `solve_ik()`가 두 모터 각도를 **서로 다른 범위**로 반환한다.

```python
a1 = (a1 + np.pi) % (2 * np.pi) - np.pi
if a1 < 0:
    a1 += 2 * np.pi          # ← a1만 0~360으로 되돌린다
a2 = (a2 + np.pi) % (2 * np.pi) - np.pi   # ← a2는 -180~180
```

| | `solve_ik` 반환 | 모터가 보고 (`zero_sta=1`) |
|---|---|---|
| `a1` (m11) | **0 ~ 360** | −180 ~ 180 |
| `a2` (m12) | −180 ~ 180 | −180 ~ 180 |

### 근거

- 코드: `git show main:src/huphy/kinematics/ankle.py`의 `solve_ik` 말미
- 펌웨어: RS02 매뉴얼 p.27 `0x7029 zero_sta` — 1로 설정 시 전원 투입 위치 범위가 `-π~π`.
  이 프로젝트는 **`zero_sta = 1`로 설정되어 있다.**

### 영향

IK가 340°를 돌려주고 모터가 −20°를 보고하면 **360° 차이**가 난다.
지금은 `safety/wrap.resolve_target_near_current()`가 −20°로 되돌려 놓아
**동작은 하지만, wrap이 버그를 덮고 있는 상태다.**

정상 동작이 안전망에 의존하는 구조는 위험하다 — wrap을 손대거나 제거하면 즉시 드러난다.

### 조치

6단계(kinematics)에서 `a1`도 `a2`와 같은 −180~180으로 통일한다.

```python
a1 = (a1 + np.pi) % (2 * np.pi) - np.pi     # if a1 < 0: ... 두 줄 삭제
a2 = (a2 + np.pi) % (2 * np.pi) - np.pi
```

⚠️ **발목 기하값이 확정된 뒤에, IK↔FK 왕복 테스트로 검증하면서 고칠 것.**
`solve_fk`의 초기 guess와 안 맞을 수 있다.

### 결정 — `safety/wrap.py`를 두지 않는다

이 이슈를 파다가 wrap 계층 자체가 불필요하다는 결론이 나왔다.

**전제 (README "하드웨어 전제" 참고)**
- `zero_sta = 1` → 모터가 `[-180, 180)`로 보고 (플래시 저장 확인됨)
- 관절 한계가 전부 ±180 안 (최대 |126.66|)
- 실제 동작 중 한 바퀴를 도는 관절이 없다

**따라서 정규화가 필요한 지점이 딱 하나뿐이다 — `solve_ik`의 a1.**

| 값이 들어오는 곳 | ±180 밖일 수 있나 |
|---|---|
| 모터 읽기 | ✘ |
| `solve_ik` 출력 | ✔ **a1이 0~360** ← 유일 |
| 목표 계산 | ✘ |
| setpoint 램프 | ✘ |

그래서 `wrap_near` / `wrap_into_interval` / `interval_near` /
`resolve_target_near_current` 네 함수가 전부 죽은 코드가 된다.

**그리고 죽은 코드가 아니라 해로운 코드다** — `resolve_target_near_current`가
a1의 340°를 −20°로 조용히 고쳐줘서 위 버그가 드러나지 않았다.

**wrap이 없는 쪽이 더 안전하다:**

| 상황 | wrap 있을 때 | wrap 없을 때 |
|---|---|---|
| 영점 미설정 모터가 150° 보고 | −210°로 고쳐 한계 안 → **통과** | 한계 밖 → **거부 / E-STOP** |

한계 검사가 이미 `±180` 밖을 걸러낸다(한계가 전부 ±180 안이므로). 별도 검증도 불필요.

**연쇄 단순화**
- `safety/limits.py` — `wrap_into_interval` 호출 제거, 단순 비교로
- `robots/leg.py` `send_action` — `resolve_target_near_current` 호출 통째로 제거

**전제를 코드가 아니라 문서와 절차로 지킨다**
- README "하드웨어 전제"에 명시
- `tables.py` 주석
- 커미셔닝 스크립트가 시작할 때 `0x7029`를 **읽어서 확인**하고 다르면 경고

---

## 🔴 #2 — 한계값이 raw 공간인지 cal 공간인지 코드가 일관되지 않음

**발견**: `right_leg.json`의 필드 설명 중

### 무엇

`limit_lo/hi_deg`는 **raw 공간**으로 정의했는데, 비교하는 쪽이 세 가지로 갈린다.

| 위치 | 비교 값 | 한계 | 판정 |
|---|---|---|---|
| `send_action` | raw | raw | ✅ |
| `check_state_bounds` | **cal** | **raw** | ❌ |
| `update_damping` | **cal** | **raw** | ❌ |
| `limit_margins` | **cal** | **raw** | ❌ |
| `motor_to_joint` | **cal** | **raw** | ❌ |
| `in_range_report` | cal | **cal로 변환** | ✅ |

### 근거

`git show main:src/huphy/robots/leg.py` 의 해당 줄들.
`cal = sign * raw + offset` 이고 현재 `sign=1.0, offset=0.0` → **`cal == raw`라 우연히 맞는다.**

### 영향

**`sign`이나 `offset`에 실측값을 넣는 순간 E-STOP·감쇠 전환·margin이 전부 틀어진다.**
잠복 버그다. 지금 값이 항등함수라 안 보일 뿐이다.

`sign = -1`이면 문제가 하나 더 있다 — `raw_to_cal(lo) > raw_to_cal(hi)`가 되어
변환 후 `min/max` 재정렬이 필요하다.

### 조치

재작성 시 **raw로 통일**한다. 한계는 무동력으로 하드스톱까지 밀어 raw를 읽어 얻는
값이라 raw가 자연스럽고, **사람에게 보여주는 지점에서만** cal로 변환한다.

5~6단계에서 결정을 확정하고 전 코드에 일관되게 적용할 것.

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

## ⚪ #7 — 모터의 통신 프로토콜 모드 미확인

### 무엇

RobStride 공장 기본값은 **private(29-bit 확장 프레임)**인데
이 코드는 **MIT(11-bit 표준 프레임)**로 보낸다.

### 근거

- RS02 매뉴얼 p.27 §4 "Protocol Switching" — `protocol_1` 또는 Type 25로 전환,
  **재부팅 필요**
- 같은 조합(RobStride + LeRobot)으로 휴머노이드를 만든 다른 프로젝트가 정확히 이
  함정에 빠졌다. 모터가 응답도 안 하고 에러도 안 내서 원인을 찾는 데 오래 걸렸다고 함

### 영향

안 맞으면 **명령이 무시되고 에러도 나지 않는다.** 연결도 되고 코드도 안 죽는데
모터만 안 움직인다 — 진단이 가장 어려운 종류의 실패.

### 조치

**3단계 시작 전에 확인할 것.**
1. motorstudio로 파라미터 `0x201F protocol_1` 읽기
2. 또는 11-bit `enable`(0xFC)에 응답이 오는지

private이면: A) `set_protocol`로 MIT 전환 + 전원 재투입, 또는 B) `codec/private.py` 구현

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

**그런데 실측값은 하드스톱 그 자체다.** 그러면 하드스톱을 5도 넘어야 E-STOP인데
물리적으로 불가능하니 **위치로는 E-STOP이 영원히 안 걸린다.**

**수정** — 세 여유를 전부 하드스톱에서 **안쪽 방향**으로 통일:

```
하드스톱                                              하드스톱
   |-1도-|--3도--|----8도----          ----|--|--|-------|
   | state command  near_stop
   | E-STOP 클리핑   감쇠전환
```

순서가 `state <= command <= near_stop`으로 바뀐다 (전부 안쪽 거리).
원본의 `command <= state <= near_stop`은 `state`만 바깥 방향이라 애초에
일관되지 않았다.

E-STOP이 가장 바깥인 것은 역할이 다르기 때문이다 — `command` 클리핑은 **예방**,
`state` E-STOP은 가드를 뚫었을 때의 **사후** 수단.

부수 효과로 세 판정이 같은 형태가 되어 함수 하나로 합쳐졌다:
```python
within(deg, limits, margin_deg=cfg.state_deg)      # E-STOP 아님
within(deg, limits, margin_deg=cfg.command_deg)    # 명령 허용
within(deg, limits, margin_deg=cfg.near_stop_deg)  # 감쇠 아님
```

> 남은 확인: `near_stop = 8도`를 양쪽에 적용하면 m9 hipy(가동폭 73도)는
> **22%가 감쇠 구간**이 된다. 좁은 관절엔 부담일 수 있다. 게인 튜닝하며
> 실제로 걸리는지 보고 전역 여유를 줄이거나 관절별 여유를 둘지 정한다.

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
