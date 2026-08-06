# `tests/`

```
tests/
├── test_safety.py          safety/                     42개
├── test_codec.py           robstride 사양·코덱          31개
├── test_base.py            motors/base.py              42개
├── test_canbus.py          motors/canbus.py            33개
├── test_robstride_bus.py   robstride/bus.py            40개
├── test_commissioning.py   robstride/commissioning.py  33개
├── test_config.py          config/                     45개
├── test_calibration.py     calibration/                35개
└── test_commission_cli.py  scripts/commission.py       30개
```

## 실행

```bash
PYTHONPATH=src python3 -m pytest tests -q
```

```
........................................................................ [ 87%]
...........................................                              [100%]
331 passed in 2.97s
```

**하드웨어도 `python-can` 도 필요 없음.**

순수 계산 계층은 애초에 `python-can` 을 쓰지 않고, 전송 계층은 `import can` 이
함수 안에 있어 가짜 모듈로 갈아끼울 수 있음.

설정과 캘리브레이션 테스트는 **저장소의 실제 파일을 읽되 쓰지 않음.** 쓰기가 필요한
것은 임시 폴더에 사본을 만들어 씀.

일부만 실행:

```bash
PYTHONPATH=src python3 -m pytest tests/test_canbus.py -q
PYTHONPATH=src python3 -m pytest tests -q -k TestApply
PYTHONPATH=src python3 -m pytest tests -v          # 이름까지 출력
```

---

## 실물이 필요한 값을 쓰는 곳

한계값은 실제 오른다리 값을 씀. **비대칭이라** 좌우 구분이 필요한 검사에 적합함.

```
무릎 m10   -20.65 ~ 74.79
```

버스 테스트는 다리 구성 그대로 씀 — RS02 4개(7~10)와 RS00 2개(11, 12)가 한 채널에
물려 있고 **토크 범위가 다름**(17 vs 14 N·m).

---

## 구성

### `test_safety.py` — 42개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestSafeWindow` | 3 | `limits.safe_window` |
| `TestClamp` | 5 | `limits.clamp` |
| `TestMarginToLimit` | 5 | `limits.margin_to_limit` |
| `TestClosestToLimit` | 3 | `limits.closest_to_limit` |
| `TestIsFinite` | 4 | `guards.is_finite` |
| `TestNanIsDangerous` | 3 | 유한값 검사의 근거 |
| `TestClampJump` | 5 | `guards.clamp_jump` |
| `TestApply` | 10 | `guards.apply` — 세 관문 전체 |
| `TestGuardCounters` | 4 | `guards.GuardCounters` |

### `test_codec.py` — 31개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestEncodingTables` | 6 | `tables.encoding_for`, 프로토콜 축 |
| `TestCommandBytes` | 3 | 겹치는 명령 바이트 |
| `TestQuantization` | 5 | `float_to_uint` / `uint_to_float` |
| `TestPackCommand` | 9 | 명령 8바이트 배치 |
| `TestDecodeState` | 4 | 상태 프레임 해석 |
| `TestDecodeFault` | 4 | 고장 워드 해석 |

### `test_base.py` — 42개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestGains` | 4 | `Gains`, `scaled` |
| `TestMotor` | 9 | 한계 순서 검증, `is_configured` |
| `TestMotorCalibration` | 12 | raw ↔ cal 변환 |
| `TestMotorState` | 5 | 신선도 판정 |
| `TestMotorFault` | 3 | 비트 필드 |
| `TestResolveMotorList` | 5 | 대상 모터 확정 |
| `TestMotorsBusContract` | 4 | ABC 강제, 컨텍스트 매니저 |

### `test_canbus.py` — 33개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestCanFrame` | 3 | 표준 프레임 기본값, frozen |
| `TestInterface` | 2 | socketcan 기본값 |
| `TestLifecycle` | 8 | 중복 연결·해제, 미연결 조작 거부 |
| `TestSend` | 6 | 순서 유지, 실패 후 계속 |
| `TestDrain` | 8 | 조기 반환, 응답 누락, 상한, `flush_rx` |
| `TestDrainAll` | 3 | 채널별 묶음, 전송이 수거보다 먼저 |
| `TestCounters` | 3 | 0이어도 전 키 출력 |

### `test_robstride_bus.py` — 40개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestConstruction` | 5 | 모델별 인코딩, 모델 오타 거부 |
| `TestCommandFrames` | 2 | 프레임 배치, `0xFB` 의 두 뜻 |
| `TestTorque` | 6 | 활성·정지·고장 클리어 |
| `TestLifecycle` | 4 | 정지 후 종료 |
| `TestSendMit` | 7 | 순서, 위치 왕복, 모델별 토크 스케일 |
| `TestCollect` | 7 | 캐시 갱신, 무응답 보고, 깨진 프레임 |
| `TestRefreshStates` | 4 | `PASSIVE` 전송, 게인 0, 큐 비우기 |
| `TestFault` | 5 | 비트 해석, 무응답과 정상의 구분 |

### `test_commissioning.py` — 33개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestSetControlMode` | 3 | `set_control_mode` |
| `TestSetZero` | 6 | 메모 필수, 토크 켜진 상태 거부 |
| `TestSetCanId` | 6 | 범위·중복·동일 id 거부 |
| `TestSetProtocol` | 4 | 전원 재투입 경고 |
| `TestNudge` | 10 | 시작 복귀, 토크 정리, 진폭 상한 |
| `TestScan` | 4 | 응답자 수집 |

### `test_config.py` — 45개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestRealFile` | 8 | 실제 `config/robot.yaml` |
| `TestUnknownKeys` | 5 | 오타 거부 |
| `TestStructure` | 8 | 필수 항목 누락 |
| `TestValues` | 6 | 한계 순서·개수, 0 이하 값 |
| `TestIdCollision` | 4 | 채널 안·채널 넘어 |
| `TestDefaults` | 6 | 빠진 절이 기본값을 쓰는지 |
| `TestSchema` | 8 | frozen, 조회 도우미 |

### `test_calibration.py` — 35개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestRealFiles` | 6 | 실제 두 파일, `robot.yaml` 과 대조 |
| `TestLoadRejects` | 11 | 형식 번호, 한계·게인 키 잔존, `sign=0` |
| `TestSave` | 7 | 왕복, 크래시 후 원본 생존 |
| `TestAttach` | 6 | 모터 id 재키잉, 관절 이름 불일치 |
| `TestUnmeasured` | 5 | 메모 기준 판정 |

### `test_commission_cli.py` — 30개

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestScan` | 4 | 무응답 표시, 원인 후보 |
| `TestState` | 3 | raw/cal 병기 |
| `TestFault` | 3 | 비트 이름, 무응답과 정상의 구분 |
| `TestNudge` | 5 | 왕복, 안 움직임 경고, 진폭 상한 |
| `TestDangerous` | 6 | `--yes` 요구, 승인 전 무전송 |
| `TestZero` | 4 | 메모 저장, 토크 차단 순서 |
| `TestTargeting` | 5 | `--limb` 요구, 설정 오류가 버스보다 먼저 |

---

## 하드웨어 없이 어떻게 확인하나

### 순수 계층 — 그냥 부름

`safety/`, `tables.py`, `codec/mit.py`, `base.py` 는 `python-can` 을 import하지
않음. 인자를 넣고 반환값을 보면 됨.

### 전송 계층 — 가짜 모듈로 갈아끼움

`canbus.py` 가 `import can` 을 **함수 안에서** 하므로 `sys.modules` 에 가짜를 넣을
수 있음.

```python
mod = types.ModuleType("can")
mod.Message = FakeMessage
mod.interface = types.SimpleNamespace(Bus=FakeCanBus)
monkeypatch.setitem(sys.modules, "can", mod)
```

### 가짜 버스는 명령에 **응답함**

```python
def send(self, msg):
    self.sent.append(msg)
    reply = self.responses.get(msg.arbitration_id)
    if reply is not None:
        self.rx.append(reply)
```

미리 큐에 넣어 두는 방식은 `refresh_states` 와 `read_fault` 에서 재현이 깨짐 —
둘 다 `flush_rx()` 를 먼저 부르므로 그 프레임이 지워짐. 실제 모터가 명령을 받은
뒤에 답하는 것과 같은 순서로 맞춤.

### 확인되지 않는 것

가짜 버스는 타이밍을 재현하지 않음. **순서·개수·집계·프레임 내용**까지가 한계임.

| | 확인 방법 |
|---|---|
| 전송 지연, CAN 중재 | 실물 |
| 모터가 실제로 응답하는지 | 실물 |
| 게인 값이 적절한지 | 실물 |
| `zero_sta` 가 켜져 있는지 | 실물 |

---

## 고정한 것

### `TestNanIsDangerous` — 유한값 검사의 근거

파이썬 `min`/`max` 가 NaN을 통과시키는 것과, `limits.clamp` 도 NaN을 못 잡는 것을
**직접 실행해 확인함.**

```python
def test_python_minmax_passes_nan_through(self):
    assert min(10, NAN) == 10

def test_clamp_does_not_catch_nan(self):
    value, clipped = limits.clamp(NAN, KNEE, margin_deg=3)
    assert math.isnan(value)
    assert clipped is False        # 잘렸다고 보고하지도 않음

def test_guards_catches_it(self):
    r = guards.apply(NAN, 0.0, ...)
    assert r.reject is guards.RejectReason.NOT_FINITE
```

**"그래서 `guards` 가 따로 검사해야 한다"는 근거가 코드로 남음.** 나중에 이 검사를
지우려는 사람이 왜 있는지 알게 됨.

### `test_limit_applied_before_jump` — 클리핑 순서

목표 200, 현재 60, `max_delta` 50 에서 **순서에 따라 결과가 달라짐.**

```
한계 먼저 (현재 구현)
  clamp(200) -> 71.79
  clamp_jump(71.79, 60, 50) -> 차이 11.79 < 50 이므로 그대로
  결과 71.79, clips = (LIMIT,)          ← 점프는 걸리지 않음

점프 먼저 (잘못된 순서)
  clamp_jump(200, 60, 50) -> 110        ← 한계(74.79)를 넘은 값
  clamp(110) -> 71.79
  결과 71.79, clips = (JUMP, LIMIT)     ← 불필요한 점프 클리핑이 기록됨
```

값은 같지만 경로가 다름. 역순은 중간에 한계 밖 값을 만들고, 실제로는 걸릴 필요가
없는 점프 클리핑을 카운터에 남김. `clips` 튜플로 순서를 고정함.

### `test_output_may_be_outside_limits_while_recovering`

현재가 이미 한계 밖(200)이면 **한 번에 복귀하지 않고** `max_delta` 씩 돌아옴.

```python
r = guards.apply(0, 200, ...)
assert r.value == pytest.approx(150.0)
assert r.value > KNEE[1]          # 아직 한계 밖
```

의도된 동작임 — 한 번에 뛰면 위험함. 나중에 "왜 한계 밖 값이 나가지?"라며 고치려는
것을 막음.

### `test_reaches_far_target_over_cycles`

버리지 않고 자르므로 먼 목표에도 결국 도달함.

```python
cur = 0.0
for _ in range(3):
    cur, _ = guards.clamp_jump(100, cur, 50)
assert cur == pytest.approx(100.0)
```

버리는 방식이면 도달 불가. **클리핑 = 속도 제한**임을 고정함.

### `test_nan_passes_the_clamp` — 코덱은 막지 못함

```python
assert mit.float_to_uint(NAN, -12.57, 12.57, 16) == 65535
```

65535는 최대값, 즉 **720° 목표 명령**임. "코덱이 알아서 막겠지"라는 오해를 막음.

### `test_protocol_axis_is_real` — 프로토콜별 속도 범위

```python
assert mit_rs02.vmax_rad_s == 33.0    # 매뉴얼 p.37~38
assert pri_rs02.vmax_rad_s == 44.0    # 매뉴얼 p.20~21
```

이 축을 없애고 "모델별 사양" 하나로 뭉치면 private 값을 MIT에 가져다 쓰는 실수가 남.
틀리면 속도 읽기가 44/33 = 1.33배 어긋남.

### `test_hard_stop_moves_in_raw_but_not_in_cal` — 한계값을 cal 에 둔 근거

영점을 3° 다른 자세에서 다시 잡으면 raw 는 달라지고 cal 은 그대로임.

```python
before = MotorCalibration(motor_id=10, offset_deg=12.0)
after  = MotorCalibration(motor_id=10, offset_deg=15.0)

assert before.cal_to_raw(74.79) == pytest.approx(62.79)
assert after.cal_to_raw(74.79)  == pytest.approx(59.79)
```

하드스톱은 쇳덩어리라 움직이지 않는데 raw 숫자만 바뀜 (이슈 #2).

### `test_mirrored_legs_share_one_cal_number`

```python
right = MotorCalibration(motor_id=10, sign=1.0)
left  = MotorCalibration(motor_id=4,  sign=-1.0)

assert right.raw_to_cal(45.0)  == pytest.approx(45.0)
assert left.raw_to_cal(-45.0)  == pytest.approx(45.0)
```

보행 궤적이 "무릎 45°" 라고 하면 양다리가 같은 동작을 함. **cal 공간이 존재하는
이유임.**

### `test_identity_when_unmeasured` — 지금 두 공간이 같은 이유

`sign=1, offset=0` 이라 `cal == raw` 임. 어느 쪽으로 해석해도 동작이 같아서 이슈 #2
가 드러나지 않음. 실측값을 넣는 순간 갈라짐.

### `test_disconnects_even_on_exception`

제어 중 예외가 나도 토크가 끊겨야 함 (이슈 #6). 없으면 모터가 마지막 명령을 계속
유지함 — 사람이 전원을 뽑을 때까지 다리가 힘을 주고 있음.

### `test_send_all_precedes_drain_all` — 이슈 #10의 순서

모든 가짜 버스가 공유하는 이벤트 순서로 확인함.

```python
kinds = [kind for kind, _ in FakeCanBus.events]
assert kinds == ["send"] * 6              # 수거가 전송 사이에 끼지 않음
```

버스별 `recv` 호출 수로는 확인되지 않음 — `drain` 은 항상 최소 한 번 `recv` 를 부름.

### `test_passive_command_carries_no_effort`

```python
leg.refresh_states([10])
assert ((data[3] & 0x0F) << 8) | data[4] == 0        # Kp
assert (data[5] << 4) | (data[6] >> 4) == 0          # Kd
```

게인이 0이 아니면 **상태를 읽는 것만으로 다리가 움직임.**

### `test_model_decides_torque_scaling`

같은 토크 값이 모델에 따라 다른 바이트가 됨. RS00은 범위가 좁아(14 N·m) RS02(17
N·m)보다 큰 수가 나감.

### `test_uses_query_variant` — 고장 조회

```python
assert raw.sent[0].data[6] == T.F_CMD_FAULT_QUERY
```

`F_CMD` 가 `0xFF` 면 **클리어**임. 조회하려다 지우면 원인을 잃음.

### `test_nothing_is_sent_without_yes` — 승인이 버스보다 먼저

```python
with pytest.raises(SystemExit):
    run("--limb", "right_leg", "zero", "knee", "--note", "편 상태")
assert FakeBus.instances == []
```

되돌리기 어려운 조작은 **모터에 아무것도 보내지 않은 상태에서** 멈춰야 함.
설정 오류도 마찬가지임 — 오타가 있는 설정으로 부르면 채널을 열지 않음.

### `test_disables_torque_first` — 영점 잡기 순서

```python
stop = next(i for i, m in enumerate(sent) if m.data[7] == T.CMD_STOP)
zero = next(i for i, m in enumerate(sent) if m.data[7] == T.CMD_SET_ZERO)
assert stop < zero
```

영점을 잡으면 좌표계가 통째로 옮겨가는데 직전 목표각은 **옛 좌표계 값**임. 그대로
유지되면 그 차이만큼 관절이 튐.

### `test_original_survives_a_crash` — 실측값 보호

`os.replace` 를 실패시켜 원본이 남는지 확인함. 실측값을 잃으면 다시 재는 수밖에
없는데, 그건 로봇을 분해해야 하는 작업일 수 있음. 임시 파일도 남지 않음.

### `test_missing_joint_is_an_error` — 관절 이름 불일치

관절 하나가 조용히 항등변환으로 도는 것이 가장 나쁨. `sign` 이 반대인 관절이
그렇게 되면 목표에서 **멀어지는 방향**으로 토크가 걸림.

### `test_limb_typo` — 설정 오타

```
ConfigError: limbs.right_leg: 모르는 키 ['contorl_hz']
```

YAML 은 모르는 키를 조용히 넘김. 고쳤는데 아무것도 안 바뀌고, 증상이 "느리다" 로
나타나므로 원인을 설정에서 찾을 이유가 없어 오래 걸림.

### `test_all_keys_always_present`

카운터가 0이어도 모든 키를 출력하는지. 필드가 나타났다 사라지면 PlotJuggler
레이아웃과 CSV 헤더가 깨짐. `safety` 와 `can` 양쪽에 있음.

---

## 미커버

| 대상 | 사유 |
|---|---|
| `kinematics/`, `robots/` | 아직 작성 전 (6단계) |
| `telemetry/` | 아직 작성 전 (7단계) |
| `control/` | 아직 작성 전 (8단계) |
| `scripts/bringup.py` | 아직 작성 전 (9단계) |

작성 순서는 [docs/build_from_scratch.md](../docs/build_from_scratch.md) 참조.
각 단계마다 해당 계층의 테스트를 여기에 추가함.
