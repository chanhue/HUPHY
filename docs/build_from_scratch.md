# 처음부터 다시 작성하기 — 작성 순서 가이드

`src/huphy/`를 직접 손으로 써보며 익히기 위한 순서표.

기존 코드는 지우지 말고 **다른 이름(`src/myhuphy/` 등)으로 새로 시작**할 것. 막히면
꺼내 볼 정답지가 있는 편이 낫고, 다 쓰고 나서 비교하면 왜 그렇게 했는지가 보인다.

---

## 0. 작업 규칙 — 이걸 지켜야 순서가 의미를 갖는다

### ① 한 파일 쓰면 바로 검증한다

파일 10개를 쓰고 나서 돌리면 어디가 틀렸는지 못 찾는다. **한 파일 → 확인 → 다음 파일.**

### ② 하드웨어 없이 되는 건 반드시 테스트를 먼저 짠다

순수 함수는 테스트가 싸다(0.03초). 나중에 실물에서 디버깅하면 비싸다.
**테스트를 먼저 쓰면 "이 함수가 뭘 해야 하는지"가 먼저 정리된다.**

### ③ 매뉴얼에서 옮긴 값은 출처를 주석에 남긴다

```python
# RS02 매뉴얼 p.38 Command 3: Byte2+Byte3[7:4] 목표속도 [0~4096] <-> (-33~33 rad/s)
vmax_rad_s = 33.0
```

나중에 "이 33이 어디서 왔지?"를 반드시 묻게 된다. 실제로 이 프로젝트에서 44와 33을
혼동해 한참 헤맸다.

### ④ 아래 계층부터 올라간다

위에서 시작하면 아래가 없어서 못 돌린다. **의존성이 없는 것부터.**

### ⑤ 각 단계 끝에 "돌아가는 것"이 하나 나와야 한다

돌릴 게 없는 단계는 너무 크게 잡은 것이다.

---

## 전체 지도

```
0단계  뼈대                     하드웨어 ✗   반나절
1단계  순수 계산 (safety)        하드웨어 ✗   1~2일   ← 여기가 핵심
2단계  프로토콜 (tables, codec)  하드웨어 ✗   1일
─────────────────────────────── 여기까지 실물 없이 완결 ───────────────
3단계  통신 (canbus, bus)        하드웨어 ✔   1일     ← 첫 실물 접촉
4단계  커미셔닝                  하드웨어 ✔   1일
5단계  설정 · 캘리브레이션       하드웨어 ✔   1~2일
6단계  로봇 (kinematics, leg)    하드웨어 ✔   2일
7단계  관찰 (telemetry)          하드웨어 ✔   1일     ← control보다 먼저!
8단계  제어 (control)            하드웨어 ✔   2일
9단계  스크립트                  하드웨어 ✔   1일
```

**1~2단계가 전체의 절반**이다. 여기서 시간을 쓰는 게 나중에 제일 싸다.

---

## 0단계 — 뼈대

### 만들 것
```
pyproject.toml
src/myhuphy/__init__.py
tests/
```

`pyproject.toml`은 최소한 이것만:
```toml
[build-system]
requires = ["setuptools>=64"]
build-backend = "setuptools.build_meta"

[project]
name = "myhuphy"
version = "0.1.0"
requires-python = ">=3.9"
dependencies = ["python-can>=4.0", "numpy>=1.21", "PyYAML>=6.0"]

[tool.setuptools.packages.find]
where = ["src"]
```

### 완료 기준
```bash
PYTHONPATH=src python3 -c "import myhuphy; print('ok')"
PYTHONPATH=src python3 -m pytest tests -q     # 0 tests도 통과로 친다
```

---

## 1단계 — 순수 계산 ★ 가장 중요

**하드웨어가 없어도 되고, 나중에 가장 디버깅하기 어려운 부분이다.**

### 순서

```
safety/wrap.py      ← 아무것도 import하지 않음. 여기서 시작
safety/limits.py    ← wrap을 씀
safety/guards.py    ← limits를 씀
control/trajectory.py  ← 아무것도 import하지 않음
```

### 1-1. `safety/wrap.py`

**풀어야 할 문제**: 엔코더는 절대 각도를 보고한다. 350°와 −10°는 같은 자세인데
숫자로는 360° 떨어져 있다. 잘못 다루면 **한 바퀴 반대로 도는 명령**이 나간다.

| 함수 | 하는 일 |
|---|---|
| `wrap_near(value, reference)` | reference에 가장 가까운 표현 |
| `wrap_into_interval(value, lo, hi, step)` | `[lo,hi]` 안에(또는 가장 가까이) |
| `interval_near(lo, hi, reference)` | 구간 자체를 옮김 |
| `resolve_target_near_current(target, current, pmax_deg)` | 전송용 표현 선택 |

**테스트를 먼저 쓴다:**
```python
def test_picks_short_way_around():
    # 현재 350도에서 목표 10도 -> 370도(+20도)여야지 10도(-340도)면 안 된다
    assert wrap_near(10.0, 350.0) == pytest.approx(370.0)
```

**함정 3가지**
- `resolve_target_near_current`는 **인코딩 범위가 거리보다 우선**이다. 범위를 벗어나면
  프레임에서 잘리기 때문
- `wrap_into_interval`의 `step_deg`는 **음수일 수 있다** (`sign = -1`인 경우)
- 후보가 여럿이면 **구간 중심에 가까운 쪽**을 고른다

### 완료 기준
```bash
pytest tests/test_wrap.py -q     # 통과
grep -n "^import\|^from" src/myhuphy/safety/wrap.py
# typing 말고 아무것도 없어야 한다. numpy조차 필요 없다
```

### 1-2. `safety/limits.py`

**세 종류의 여유**를 구분하는 게 핵심이다.

```
command_deg   (3°)  명령을 이만큼 안쪽으로     ← 여유를 뺀다
state_deg     (5°)  이만큼 넘으면 E-STOP       ← 여유를 더한다
near_stop_deg (8°)  이만큼 가까우면 감쇠 전환
```

**왜 명령은 빼고 상태는 더하나**: 측정값은 이미 일어난 일이라 명령보다 관대해야
정상 동작 중 오탐이 안 난다.

**꼭 넣을 것**: `any_near_stop()`이 bool이 아니라 **범인 모터 id를 반환**하게 할 것.
하나만 걸려도 다리 전체가 감쇠로 가므로, 나중에 "왜 힘이 빠졌나"를 못 찾는다.

**함정**: 가동범위가 여유의 2배보다 좁으면 `[lo+3, hi-3]`이 뒤집힌다. 그때는 여유를
포기해야 한다. 안 그러면 **어떤 명령도 통과하지 못한다.**

### 1-3. `safety/guards.py`

**핵심 결정**: 거부를 `None` 반환이 아니라 **사유(enum)로 돌려줄 것.**

```python
class RejectReason(str, Enum):
    NO_STATE = "nostate"
    OUT_OF_LIMITS = "limit"
    JUMP_TOO_LARGE = "jump"
    UNREACHABLE = "ik"
```

원본은 `print`만 하고 조용히 건너뛰었는데, `print`가 0.5초 스로틀이라 **100Hz에서
200번 거부돼도 콘솔엔 1줄**만 떴다. 사유를 값으로 돌려주면 세어서 그래프로 볼 수 있다.

### 1-4. `control/trajectory.py`

**여기가 제어의 핵심 개념이 들어가는 곳이다.** 먼저
[option3_control_analysis.md](../leg_control/docs/option3_control_analysis.md)를 읽을 것.

```python
# 하면 안 되는 것 (재앵커)
nxt = 실측 + clamp(목표 - 실측, ±step)
# → 오차가 구조적으로 step을 넘을 수 없다 → 토크가 kp·step에 묶인다

# 맞는 것 (절대 setpoint 램프)
sp = sp + clamp(목표 - sp, ±step)
# → 모터가 뒤처지면 오차가 쌓여 필요한 만큼 토크가 나온다
```

**회귀 테스트를 반드시 쓸 것:**
```python
def test_setpoint_advances_regardless_of_measurement():
    ramp = SetpointRamp.starting_at(0.0, max_step_deg=2.0)
    for _ in range(10):
        ramp.advance(100.0)          # 실측이 전혀 안 움직여도
    assert ramp.setpoint_deg > 2.0   # setpoint는 계속 전진해야 한다
```

**함정**: `ramp_profile`에서 분할 수를 `round`가 아니라 **`ceil`**로 할 것.
`round`면 10도를 3도씩 나눌 때 3분할이 되어 스텝이 3.33도가 된다 — 속도 제한을 넘는다.

### 1단계 완료 기준
```bash
pytest tests -q          # 30개 이상 통과
# python-can 없이 돌아야 한다
```

---

## 2단계 — 프로토콜

여전히 하드웨어가 필요 없다. **매뉴얼이 필요하다.**

### 2-1. `motors/robstride/tables.py`

**RS02 매뉴얼을 열고 값을 옮긴다.** 이 단계의 90%는 문서 읽기다.

⚠️ **인코딩 범위는 `[프로토콜][모델]`로 인덱싱할 것.** 같은 RS02라도:

| 프로토콜 | 위치 | 속도 | 토크 | 매뉴얼 |
|---|---|---|---|---|
| private (29-bit) | ±12.57 rad | ±44 rad/s | ±17 N·m | p.20~21 |
| **MIT (11-bit)** | ±12.57 rad | **±33 rad/s** | ±17 N·m | p.37~38 |

"모델별 사양" 하나로 뭉치면 다른 프로토콜 값을 가져다 쓰는 실수가 난다.
**이 프로젝트와 LeRobot 양쪽에서 실제로 일어났다.**

담을 것: 인코딩 범위, CAN 명령 바이트, 고장 비트, 파라미터 인덱스, 열/부하 한계

### 2-2. `motors/robstride/codec/mit.py`

8바이트 ↔ 숫자 5개.

```
Byte0~1              목표 각도  16bit
Byte2 + Byte3[7:4]   목표 속도  12bit
Byte3[3:0] + Byte4   Kp         12bit
Byte5 + Byte6[7:4]   Kd         12bit
Byte6[3:0] + Byte7   목표 토크  12bit
```

**테스트로 검증할 것:**
- 왕복: `pack` 후 위치 필드를 꺼내면 원래 값이 나오나 (오차 0.03° 이내)
- 클램프: 범위를 넘으면 잘리는가 (감싸지 않는가)
- **속도 0은 vmax와 무관하게 같은 비트** — 이걸 고정해두면 나중에 vmax를 고쳐도
  명령 바이트는 안 바뀐다는 걸 안다

### 2단계 완료 기준
```bash
pytest tests -q     # 50개 이상. 여전히 python-can 불필요
```

**여기까지가 실물 없이 완결되는 부분이다.** 전체의 절반쯤 왔다.

---

## 3단계 — 통신 ★ 첫 실물 접촉

### 먼저 할 것 — CAN 인터페이스 올리기

```bash
sudo ip link set can1 up type can bitrate 1000000
ip -details link show can1
candump can1        # can-utils. 프레임이 보이나
```

**코드를 쓰기 전에 이게 먼저 되어야 한다.**

### ❗ 그 다음 — 프로토콜 모드 확인

**RobStride 공장 기본값은 private(29-bit)이다.** 우리 코드는 MIT(11-bit)를 보낸다.
안 맞으면 **명령이 무시되고 에러도 안 난다.**

```
motorstudio로 파라미터 0x201F protocol_1 읽기
또는 11-bit enable(0xFC)에 응답이 오는지
```

private이면 두 갈래:
- **A.** `set_protocol`로 MIT 전환 + 전원 재투입 (모터마다)
- **B.** `codec/private.py`도 구현

**이걸 확인하지 않고 3단계를 진행하면 하루를 날린다.**

### 3-1. `motors/canbus.py`

전송 계층만. **프레임의 의미는 모른다.**

| 메서드 | 주의 |
|---|---|
| `send(id, data, extended)` | |
| `send_many(frames)` | **락 한 번으로 연속 전송.** 안 그러면 프레임이 섞여 응답이 유실된다 |
| `recv(timeout)` | |
| `drain(handler)` | 큐가 빌 때까지. **마지막 recv가 timeout만큼 블로킹**된다 |
| `recv_matching(pred, handler)` | 기다리는 동안 지나가는 프레임도 handler로 넘길 것 |

TX/RX 락을 **따로** 둘 것. 송신과 수신은 서로 막을 이유가 없다.

### 3-2. `motors/base.py`

`MotorState`, `MotorCalibration`, `MotorsBus(ABC)`.

**`MotorSpec` 같은 인코딩 범위는 여기 두지 말 것** — MIT류 특유 개념이라
벤더 중립 자리에 안 맞는다. `robstride/tables.py`로.

### 3-3. `motors/robstride/bus.py`

명령 바이트 규약:
```
data[0:6] = 0xFF,  data[6] = F_CMD,  data[7] = 명령코드

0xFC + F_CMD=0xFF -> Enable      / 0xFC + F_CMD=mode  -> 제어모드
0xFD + F_CMD=0xFF -> Stop        / 0xFD + F_CMD=proto -> 프로토콜(커미셔닝)
0xFB + F_CMD=0xFF -> 고장 클리어  / 0xFB + F_CMD=그외  -> 고장 조회
```

**상태 캐시를 둘 것.** 제어 루프가 물어볼 때 배선까지 나가지 않고 메모리를 읽게.

### 3단계 완료 기준
```python
bus.connect()
failed = bus.enable_torque()
print("무응답:", failed)          # [] 이어야 한다
missing = bus.sync_read_states()
print(bus.states())               # 각 모터의 pos/vel/tau/temp
bus.disable_torque()
```

**"모터 6개가 전부 응답한다"가 이 단계의 목표다.** 아직 움직이지 않는다.

---

## 4단계 — 커미셔닝

**되돌리기 어려운 조작을 별도 파일로 격리한다.**

```
motors/robstride/commissioning.py
```

| 함수 | 성격 |
|---|---|
| `set_mechanical_zero` | 플래시 저장. 전원 꺼도 유지 |
| `set_can_id` | 즉시 적용. 이후 새 ID로만 응답 |
| `set_protocol` | **전원 재투입 후** 적용 |

**규칙: 런타임 모듈은 이 파일을 import하지 않는다.** 제어 루프 코드가 실수로
CAN ID를 바꾸지 못하게.

### 이 단계에서 실제로 할 일

1. **모터 ID ↔ 관절 매핑 확인** — 어느 모터가 무릎인지. 한 개씩 살짝 움직여 보며
2. **기계 영점 잡기** — 관절을 의도한 홈 자세에 놓고 `0xFE`
3. **어느 자세에서 잡았는지 기록** — 모터에도 코드에도 안 남는다

### 완료 기준
전원을 껐다 켜도 각 모터가 0 근처를 보고하는가

---

## 5단계 — 설정과 캘리브레이션

### 5-1. `config/` — 스키마와 값을 나눈다

```
src/myhuphy/config/robot.py    dataclass (스키마)
config/robot.yaml              값
```

**왜 나누나**: 조립이 바뀔 때마다 소스를 고치면 측정 결과가 코드 변경 이력에 섞인다.

### 5-2. `calibration/store.py`

```
config/calibration/right_leg.json
```

**전역 `CALIBRATED = True/False` 플래그를 만들지 말 것.**
`is_complete(calibration, motor_ids)`로 **실제 데이터를 보고 판정**할 것:
- 한계값이 있는가
- `kp > 0`인가  ← **kp가 0이면 토크가 항상 0이라 안 움직인다**

### 5-3. 실측

| 항목 | 방법 |
|---|---|
| `sign` | 무동력으로 관절을 + 방향으로 밀고 raw가 증가하는지 |
| `limit_lo/hi` | 무동력으로 하드스톱까지 → raw 읽기 → 여유 빼기 |
| `kp`/`kd` | **7단계 이후.** 텔레메트리 없이는 못 한다 |

### ⚠️ 여기서 결정할 것 — 한계값을 어느 공간에 둘 것인가

`limit`을 **raw 공간**에 둘지 **cal 공간**(sign/offset 적용 후)에 둘지 **처음에 정하고
전 코드에서 일관되게** 쓸 것.

기존 코드는 이게 섞여 있다 — `sign=1, offset=0`이라 지금은 우연히 맞지만 실측값을
넣는 순간 갈라진다. **같은 실수를 반복하지 말 것.**

권장: **raw로 통일.** 한계는 무동력으로 하드스톱까지 밀어 raw를 읽어 얻는 값이라
raw가 자연스럽고, 사람에게 보여주는 지점에서만 cal로 변환하면 된다.

---

## 6단계 — 로봇

### 6-1. `kinematics/ankle.py`

발목 2모터 링키지 IK/FK. **순수 계산이라 하드웨어가 필요 없다** — 사실 1단계에
넣어도 된다.

**FK가 뉴턴 반복(최대 120회)이라 비싸다.** 직전 해를 `guess`로 넘겨 반복을 줄일 것.

**테스트로 IK↔FK 왕복을 확인할 것** — `solve_ik(p, r)` 결과를 `solve_fk`에 넣으면
`(p, r)`이 나와야 한다. (기존 코드에 이 테스트가 없다. 넣는 게 좋다)

### 6-2. `robots/base.py` — `Robot` ABC

무엇을 계약에 넣을지 결정한다. 최소한:
```
observation_features / action_features / is_connected / is_calibrated
connect / disconnect / configure / calibrate / get_observation / send_action
```

**`calibrate()`를 빠뜨리지 말 것** — 기존 코드가 빠뜨렸다.

`name`은 **클래스 변수**로, 개체 구분은 `id`로 나눌 것. 기존 코드는 하나로 합쳐놔서
같은 종류 두 대를 구분할 수 없다.

### 6-3. `robots/leg.py` — `SingleLeg`

**유일하게 "무릎"을 아는 계층.**

핵심 결정 3가지:
- **목표를 관절 공간으로 보관할 것.** 모터 공간으로 미리 바꾸면 캘리브레이션이
  갱신될 때 목표의 의미가 조용히 달라진다
- **실제로 프레임에 실린 값을 따로 남길 것** (`_last_sent_raw`). `err = tgt − pos`의
  `tgt`가 이것이어야 모터 PD가 보는 오차와 일치한다
- 발목 a1/a2는 **sign/offset을 적용하지 않는다** — `AnkleKinematics`가 그 역할을 한다

### 완료 기준
```python
leg.connect()
print(leg.joint_state())     # 관절 각도가 나오나
leg.set_action(knee=5.0)
leg.send_action()            # 5도 움직이나
```

**여기서 처음으로 모터가 관절 이름으로 움직인다.**

---

## 7단계 — 관찰 ★ 제어보다 먼저

**순서를 바꾸지 말 것.** 게인 튜닝을 하려면 응답을 봐야 하고, 그러려면 텔레메트리가
먼저 있어야 한다. **측정 수단 없이 튜닝하는 건 눈감고 하는 것이다.**

먼저 [monitoring.md](monitoring.md)를 읽을 것 — 무엇을 왜 보는지가 정리되어 있다.

### 7-1. 스키마를 한 곳에서 정의

`robots/leg.py`에 `telemetry_snapshot()` — 평면 dict 하나.
**CSV 헤더도 UDP 필드도 여기서 나오게.** 두 군데 정의하면 반드시 어긋난다.

### 7-2. `telemetry/udp.py`

- 논블로킹, **모든 예외를 삼킬 것** (뷰어가 꺼져 있으면 ICMP로 sendto가 실패한다)
- 다만 `drop_count`는 셀 것 — 조용히 죽는 것과 구분되게
- 호스트 미지정이면 소켓조차 만들지 말 것

### 7-3. `telemetry/csv_log.py`

- **헤더를 첫 스냅샷의 `keys()`로 만들 것** — 그래야 정의가 한 곳
- flush는 N사이클마다 + **E-STOP 시 즉시**

### 완료 기준
PlotJuggler에 그래프가 뜨는가. `t`를 timestamp 필드로 지정할 것.

---

## 8단계 — 제어

### 8-1. `control/loop.py`

**실제 주기를 잴 것** (`loop_dt`). 원본은 밀렸을 때 조용히 넘어가서 느려져도
아무도 몰랐다.

모드 둘:
- `state_only` — 토크 끄고 상태만. **하드 안전장치로 `disable_torque()` 강제**
- `control` — 명령 전송

### 8-2. 이제 게인 튜닝

kp를 낮게 시작해 올리며 그래프를 본다. 볼 것:
- `err` — 정상상태 오차가 줄어드나
- `vel` — 진동이 생기나 (kd 부족)
- `tau` vs `kp·err − kd·vel` — 이론과 실측이 맞나
- `tau`가 상한(17 N·m)에 붙었나 — **붙었으면 kp를 올려도 소용없다**

---

## 9단계 — 스크립트

`scripts/bringup.py` — 대화형 메뉴.

**커미셔닝 항목에 `[영구]` 표시를 붙일 것.** 되돌리기 어려운 조작임을 화면에서도 구분.

⚠️ **메뉴가 제어 루프를 타게 할 것.** 기존 코드는 메뉴가 버스를 직접 호출해서
텔레메트리가 흐르지 않는다. 같은 실수를 반복하지 말 것.

### 순서를 앞당긴 부분

커미셔닝 진입점이 5단계 뒤로 당겨졌다. 4단계에서 `commissioning.py` 를 만들었지만
실행할 방법이 없었고, 실행하려면 설정 파일을 읽어야 했기 때문이다.

    4단계 커미셔닝  ->  5단계 설정  ->  커미셔닝 진입점  ->  실물 확인

`scripts/bringup.py` 는 그대로 9단계에 둔다. 커미셔닝 진입점과 달리 제어 루프와
텔레메트리가 다 있어야 한다.

---

## 10단계 — 최종 문서

전 단계가 끝나면 **루트 `README.md`** 를 쓴다. 지금까지의 폴더별 README 는 각
계층이 무엇인지 설명하지만, 로봇을 처음 만지는 사람이 **무엇을 순서대로 해야
하는지**는 어디에도 없다.

### 담을 것

**사용법** — 설치부터 다리가 움직이기까지의 순서

    CAN 채널 올리기        ip link
    프로토콜·영점 확인      하드웨어 전제
    모터 매핑 확인          어느 id 가 어느 관절인지
    영점 잡기
    캘리브레이션 실측
    게인 튜닝
    제어 실행

**조정해야 할 값** — 어느 파일의 어느 줄을 왜 고치는지

| 값 | 파일 | 언제 |
|---|---|---|
| `limits_deg` | `config/robot.yaml` | 기구 설계가 바뀌면 |
| `kp` / `kd` | `config/robot.yaml` | 튜닝할 때. 지금 0 |
| `command_margin_deg` | `config/robot.yaml` | 게인을 바꾸면 다시 봄 |
| `sign` / `offset_deg` | `config/calibration/*.json` | 모터를 다시 달면 |
| `zero_reference` | `config/calibration/*.json` | 영점을 잡을 때 |
| `channel` | `config/robot.yaml` | 배선이 바뀌면 |

**미실측 값이 어디에 남아 있는지** — 지금 무엇이 아직 0이거나 비어 있는지, 그것이
동작을 어떻게 막고 있는지.

### 완료 기준

이 저장소를 처음 받은 사람이 README 만 보고 다리를 움직일 수 있다.

---

## 하지 말 것

| | 왜 |
|---|---|
| 전부 다 쓰고 나서 돌려보기 | 어디가 틀렸는지 못 찾는다 |
| 테스트 없이 1~2단계 넘어가기 | 나중에 실물에서 디버깅하면 10배 비싸다 |
| 프로토콜 모드 확인 없이 3단계 | 하루를 날린다 |
| 텔레메트리 없이 게인 튜닝 | 눈감고 하는 것 |
| 한계값 공간(raw/cal)을 안 정하고 시작 | 실측값 넣는 순간 갈라진다 |
| 캘리브레이션을 코드에 박기 | 측정할 때마다 소스를 고치게 된다 |

---

## 참고 순서

| 언제 | 읽을 것 |
|---|---|
| 시작 전 | [architecture.md](architecture.md) — 계층과 의존 방향 |
| 1-4단계 전 | [option3_control_analysis.md](../leg_control/docs/option3_control_analysis.md) — 제어의 기본 |
| 2단계 전 | RS02 매뉴얼 p.20~21, p.37~38 |
| 7단계 전 | [monitoring.md](monitoring.md) — 무엇을 왜 보나 |
| 막힐 때 | [flow_diagrams.md](flow_diagrams.md) — 호출 관계 그림 |
| 비교할 때 | 기존 `src/huphy/` + 각 폴더 README |
