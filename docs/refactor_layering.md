# 계층 분리 리팩토링 기록 (2026-07-31)

[architecture.md](architecture.md)에서 설계한 구조를 실제 코드로 옮긴 작업 기록.
무엇이 왜 섞여 있었고, 어떻게 나눴고, 무엇이 검증되었고 무엇이 안 되었는지를 남긴다.

**범위**: `src/huphy/` 신규 패키지 생성. 기존 `leg_control/`은 **그대로 둔다**
(실물 하드웨어 검증 전이므로 참조용으로 남긴다).

---

## 1. 무엇이 섞여 있었나

### 1.1 `single_leg_controller.py` — 1,566줄에 8가지 역할

| 역할 | 해당 코드 |
|---|---|
| CAN 전송 계층 | `_send8`, `_drain_rx_states`, `_recv_motor_reply` |
| 프로토콜 명령 | `enable`, `disable`, `set_zero`, `_send_cmd_ff`, `_request_state` |
| 프레임 코덱 사용 | `_prepare_mit_payload` |
| 안전 로직 | `_raw_to_cal_near_interval`, `_resolve_raw_target_near_current`, `_raw_command_in_limits`, `_is_near_stop`, `_passes_action_update_jump_guard` |
| 기구학 접착 | `_motor_to_joint`, `_joint_to_motor` |
| 제어 루프 | `_run_loop` |
| 궤적 생성 | `_step_motor_toward_raw` |
| 진단 · CLI · 로깅 | `run_joint_max_rom_sweep`, `run_full_rom_diagnostic`, `_menu_*`, `_log_state` |

결과적으로 **안전 로직이 CAN 버스와 락에 얽혀 있어 하드웨어 없이 검증할 수 없었다.**
360도 wrap 규칙(±4바퀴 탐색, 구간중심 tie-break)처럼 미묘한 코드가 테스트 없이
방치되어 있었다.

### 1.2 `robot_constant.py` — 성격이 다른 3가지가 한 파일에

```python
MOTORS, CAN_CMD_*                          # 벤더 사양   — 데이터시트, 안 변함
LEG_*_MOTOR_IDS, LEG_JOINT_NAMES           # 토폴로지    — 조립 구성
MOTOR_SIGN, MOTOR_OFFSET_DEG,              # 실측값      — 자주 바뀜
JOINT_LIMITS_DEG, DEFAULT_GAINS
COMMAND_MARGIN_DEG 등                      # 안전 파라미터
```

`CALIBRATED = False`가 풀리지 않은 구조적 이유가 이것이다 — 값 하나를 실측할 때마다
소스를 고쳐야 하고, 좌우 다리가 하나의 전역을 공유하며, 측정 결과가 코드 리뷰
대상이 된다. 어떤 값이 실측이고 어떤 값이 플레이스홀더인지 구분도 사라졌다.

### 1.3 커미셔닝 조작이 3곳에 분산

기계 영점(0xFE)처럼 **플래시에 영구 저장되고 되돌리기 어려운** 조작이
`test_zero_persistence.py`(독립 스크립트), `_menu_set_zero`(메뉴 함수),
`set_zero_all`/`zero_here`(컨트롤러 메서드) 세 군데에 있었다. 제어 루프 코드가
같은 클래스에서 이 API에 손을 댈 수 있는 상태였다.

---

## 2. 어떻게 나눴나

```
scripts ──→ robots ──→ { motors, kinematics, safety, config, calibration }
                └────→ telemetry   (단방향)
```

| 계층 | 아는 것 | 모르는 것 | 파일 |
|---|---|---|---|
| `motors/` | 모터 ID, 원시 각도, CAN 프레임 | "무릎"이 무엇인지 | base, canbus, robstride/* |
| `kinematics/` | 링크 기하 | CAN, 모터 ID | ankle |
| `safety/` | 숫자, 한계값 | CAN, 상태 (**순수 함수**) | wrap, limits, guards |
| `config/` | 값의 스키마 | 나머지 전부 | robot, loader |
| `calibration/` | JSON 입출력 | 나머지 전부 | store |
| `robots/` | 관절 이름 ↔ 모터 매핑 | 프레임 포맷 | base, leg, factory |
| `control/` | 궤적, 주기 | 프레임 인코딩 | trajectory, loop |
| `telemetry/` | dict 하나 | **로봇 구조 전체** | udp, csv_log |

---

## 3. 파일별 내용

### `motors/base.py` — 벤더 중립
`MotorState`, `MotorCalibration`, `MotorFault`, `MotorsBus(ABC)`.

`MotorSpec`을 **일부러 여기 두지 않았다.** `pmax/vmax/tmax`는 MIT류 프로토콜 특유의
개념이고 CANopen 계열에는 존재하지 않는다. 벤더 중립 자리에 벤더 특유 개념이
올라가면 안 된다.

`MotorCalibration`에 `zero_reference` 필드를 새로 넣었다 — 기계 영점을 **어느
자세에서 잡았는지**는 모터에도 코드에도 남지 않는데, 그 정보 없이는 모든 한계값이
의미를 잃는다.

### `motors/canbus.py` — 전송 계층만
프레임의 **의미를 모른다.** 8바이트를 보내고 받고, TX/RX 락으로 직렬화하고, RX 큐를
비운다.

`send_many()`를 새로 넣었다 — 모터별로 락을 잡았다 풀면 그 사이에 다른 스레드
프레임이 끼어들어 응답 순서가 섞인다. 한 사이클분 명령은 락 한 번으로 연속 전송한다.

### `motors/robstride/tables.py` — 벤더 사양
**인코딩 범위를 `[프로토콜][모델]`로 인덱싱한다.** 같은 RS02라도 MIT 프로토콜은
±33 rad/s, private은 ±44 rad/s다. 이 축을 없애고 "모델별 사양" 하나로 뭉치면 다른
프로토콜의 값을 가져다 쓰는 실수가 나고, 실제로 이 프로젝트와 LeRobot 양쪽에서
발생했다.

그 외에 CAN 명령 바이트, 고장 비트 정의, 파라미터 인덱스, 열/부하 한계(정격 6 N·m,
피크 17 N·m가 10초, 과열 130°C, 토크상수 1.22 N·m/Arms)를 담는다.

### `motors/robstride/codec/mit.py` — 프레임 코덱
`pack_command` / `decode_state` / `decode_fault`. 로봇을 전혀 모르고 숫자와 바이트만
안다. 인코딩 범위를 인자로 받으므로 모델·프로토콜에 묶이지 않는다.

### `motors/robstride/bus.py` — 런타임 전용
`enable_torque`, `disable_torque`, `sync_read_states`, `sync_write_mit`,
`read_fault`, `set_control_mode`, 상태 캐시.

MIT 표준 프레임의 `F_CMD` 규약을 코드에 명시했다 — `data[6]`이 `0xFF`면 기본 동작,
다른 값이면 변형 동작이다. `0xFC+0xFF`=Enable / `0xFC+mode`=제어모드 설정,
`0xFB+0xFF`=고장 클리어 / `0xFB+그외`=고장 조회.

### `motors/robstride/commissioning.py` — 1회성 · 영구
`set_mechanical_zero`, `set_can_id`, `set_protocol`, `save_to_flash`.

**런타임 모듈은 이 파일을 import하지 않는다.** `robstride/__init__.py`에서도
일부러 노출하지 않아, 쓰려면 `from ...robstride import commissioning`처럼 명시적으로
가져가야 한다.

### `kinematics/ankle.py`
기존 `ankle_kinematics.py`를 옮기고 회전 행렬 생성을 `_rotate()`로 묶어 IK와 FK가
공유하게 했다(원래 두 곳에 중복). 좌우 미확정·대칭 가정 TODO는 그대로 유지.

### `safety/wrap.py` · `limits.py` · `guards.py` — 순수 함수
컨트롤러에서 추출. 상태도 락도 CAN도 없으므로 **하드웨어 없이 테스트된다.**

`limits.any_near_stop()`은 bool이 아니라 **범인 모터 id를 반환**하도록 바꿨다. 하나만
걸려도 다리 전체가 감쇠로 가므로, "왜 갑자기 힘이 빠졌나"를 알려면 어느 모터인지가
필요하다.

`guards.check_command()`는 통과/거부가 아니라 **거부 사유(enum)를 반환**한다.
`RejectCounters`가 사유별로 집계한다.

### `config/robot.py` · `loader.py`
`MotorConfig`, `SafetyConfig`, `TelemetryConfig`, `LegConfig`, `RobotConfig` dataclass와
YAML 로더. `LegConfig.__post_init__`이 role 누락·중복, 모터 id 중복을 잡는다.
`SafetyConfig`는 여유 순서(command ≤ state ≤ near_stop)를 검증한다.

### `calibration/store.py`
JSON 입출력 + `is_complete()` / `missing_report()`.

**전역 `CALIBRATED` 플래그를 없앴다.** 대신 실제 데이터를 보고 판정하므로 다리마다
따로 성립할 수 있고, 무엇이 빠졌는지 모터별로 알려준다.

### `robots/leg.py` — 유일하게 "무릎"을 아는 계층
관절공간 ↔ 모터공간 변환, 목표 관리, 안전 적용, **텔레메트리 스키마 정의**.

목표를 **관절 공간으로 보관**한다(모터 공간으로 미리 바꿔두지 않는다). 미리 바꾸면
캘리브레이션이 갱신될 때 목표의 의미가 조용히 달라진다.

`_last_sent_raw`에 **실제로 프레임에 실린 값**을 남긴다. `err = tgt − pos`의 `tgt`가
이것이어야 모터 펌웨어 PD가 보는 오차와 일치한다.

### `control/trajectory.py` — 옵션3 수정 지점
`SetpointRamp`. 아래 §4.2 참조.

### `control/loop.py`
`LoopTiming`이 실제 주기와 오버런을 잰다. 원본은 밀렸을 때 `next_tick`을 리셋하고
따라잡기를 포기하면서 **아무 기록도 남기지 않았다.**

### `telemetry/udp.py` · `csv_log.py`
UDP는 논블로킹 + 전 예외 삼킴 + `drop_count` 집계. 호스트 미지정이면 소켓조차 만들지
않는다. CSV는 **첫 스냅샷의 `keys()`로 헤더를 만든다** — 그래야 스키마 정의가
`robots/leg.py` 한 곳에만 존재한다.

---

## 4. 동작이 바뀐 것

### 4.1 `vmax` 44 → 33 rad/s (MIT 프로토콜)

RS02 매뉴얼 p.37~38 (Command 3 / Response Command 1)이 MIT 프레임의 속도 범위를
**±33 rad/s**로 명시한다. 기존 값 44는 p.20~21의 **private 프로토콜** 표에서 온
것이었다.

| | 위치 | 속도 | 토크 |
|---|---|---|---|
| private (29-bit 확장) | ±12.57 rad | ±44 rad/s | ±17 N·m |
| **MIT (11-bit 표준)** ← 사용 중 | ±12.57 rad | **±33 rad/s** | ±17 N·m |

**영향**: 속도 **읽기**가 실제보다 1.33배 크게 나오던 것이 교정된다. 위치·토크는
영향 없다. 속도 피드포워드가 0이라 **명령 바이트는 바뀌지 않는다**(0은 vmax와 무관하게
같은 비트로 인코딩된다 — `test_zero_velocity_encodes_to_midpoint`로 고정).

토크 17 N·m는 매뉴얼 4곳에서 교차 확인된다: MIT 섹션, private 섹션, p.10 파라미터
`0x2007 limit_torque`(max 17 / 기본 17), p.3 Peak load 17 N·m.

### 4.2 궤적 생성을 절대 setpoint 램프로 교체

```python
# 변경 전 (재앵커):  nxt = 실측 + clamp(목표 − 실측, ±step)
# 변경 후 (절대):    sp  = sp   + clamp(목표 − sp,   ±step)
```

재앵커 방식은 위치 오차가 구조적으로 `step`을 넘을 수 없어(오차 = 명령 − 실측 =
clamp(...) ≤ step) 토크가 `kp·step`으로 묶였다. 중력을 거스르는 구간에서 필요 토크가
그보다 크면 목표 직전에 정지하고, 도달 판정이 안 나므로 방향 전환도 일어나지 않았다.
(→ [option3_control_analysis.md](../leg_control/docs/option3_control_analysis.md))

도달 판정도 **setpoint 기준**으로 바꿨다. 실측 기준이면 부하가 큰 구간에서 영영
도달하지 못해 왕복이 멈춘다.

회귀 테스트를 고정해뒀다 — 실측이 전혀 안 움직여도 setpoint는 전진하고 오차가
step을 넘어야 한다.

### 4.3 거부 사유별 집계

원본은 거부 시 `_prepare_mit_payload`가 `None`을 반환하고 `_send_action_all`이
`continue`로 **조용히 건너뛰었다.** `print`마저 모터당 0.5초 스로틀이라 100Hz에서
200번 거부돼도 콘솔에는 1줄만 떴다 — **거부가 산발적인지 지속적인지 알 방법이 없었다.**

이제 `rejects_nostate` / `rejects_limit` / `rejects_jump` / `rejects_ik` /
`rejects_estop`으로 나눠 세어 텔레메트리로 나간다. 합계만 보면 어느 가드가 걸렸는지
알 수 없다.

### 4.4 루프 주기 계측

`loop_dt`, `loop_dt_max`, `loop_overruns`. 평균은 거의 쓸모없다 — 평균 10ms인데 가끔
50ms 튀는 것이 문제다.

### 4.5 CSV flush 정책

매 사이클 flush(디스크 I/O가 제어 주기 예산에 들어감) → N사이클 버퍼링 +
**E-STOP 변화 시 즉시 flush**. 가치가 가장 높은 구간을 잃지 않으면서 평시 부하를 줄인다.

---

## 5. 작업 중 발견한 것

### 5.1 `ramp_profile`의 스텝 초과 버그

분할 수를 `round(distance/step)`로 계산해서 나머지가 절반 미만일 때 스텝이
`max_step_deg`를 **초과했다** (10도를 3도씩 → 3분할 → 3.33도/스텝). `step_deg`는
속도 제한 역할이므로 넘으면 안 된다. `ceil`로 수정. 테스트가 잡았다.

### 5.2 `__init__.py` 조기 import가 계층 분리를 무효화

처음 작성했을 때 `__init__.py`들이 하위 모듈을 전부 즉시 import해서, **순수 계층
테스트가 `python-can`을 요구했다.** 계층을 나눈 목적이 무너지는 문제라 PEP 562
지연 로딩(`__getattr__`)으로 바꿨다.

지금은 `safety`, `kinematics`, `control.trajectory`, `motors.base`,
`motors.robstride.tables`, `codec.mit`이 CAN 없이 import된다.

---

## 6. 검증

### 한 것

| 항목 | 결과 |
|---|---|
| 단위 테스트 | **54개 통과** (safety 순수 함수, MIT 코덱, setpoint 램프 회귀) |
| 모듈 import | **22/22** (python-can 스텁 사용, 환경 미변경) |
| 설정 로드 | `robot.yaml` → `RobotConfig`, role↔id 매핑 확인 |
| 캘리브레이션 게이트 | 미완 상태에서 `allow_uncalibrated=False`면 생성 차단 확인 |
| 텔레메트리 스냅샷 | 83필드 = JSON **1219 B** (MTU 안전선 1400 B 이내) |
| UDP 비활성 경로 | 호스트 미지정 시 `send()` 무해 확인 |

### 안 한 것

- **실물 하드웨어 검증** — CAN 통신, 실제 모터 응답, 제어 동작 전부 미확인
- `python-can` 설치 (환경을 건드리지 않기 위해 스텁으로 대체)
- 기존 `leg_control/`과의 동작 동등성 비교

---

## 7. 남은 작업

### 아직 옮기지 않은 것

| 원본 | 상태 |
|---|---|
| `run_joint_max_rom_sweep`, `run_full_rom_diagnostic` | 미이관 |
| `run_ankle_trajectory` | 미이관 (`interpolate_waypoints`는 준비됨) |
| `verify_and_start` | 일부만 이관 (`in_range_report()`로) |
| `move_motor10.py` | 미이관 (원래도 `commission_motor` 의존으로 import 불가) |
| `test_zero_persistence.py` | 미이관 (`commissioning.py`에 원시 함수는 준비됨) |

### 설계에는 있으나 미구현

- `motors/robstride/params.py` — 파라미터 R/W + 플래시 저장
- `motors/robstride/codec/private.py` — 29-bit 확장 프레임

### 미연결

**`LegControlLoop`와 텔레메트리가 브링업 메뉴에 아직 연결되지 않았다.** 메뉴는
루프를 돌리지 않고 버스를 직접 호출하므로, 지금 상태로는 메뉴를 써도 그래프에
데이터가 흐르지 않는다. 이는 [monitoring.md](monitoring.md) §4.1-(6)에서 지적한
"메뉴 경로는 `_run_loop`를 타지 않는다" 문제와 같다.

### 확인 필요

- ~~**모터 id ↔ 관절 매핑**~~ — **해결됨.** 원본은 주석(`7=hipz`)과 코드(`7=hipy`)가
  달랐고, **코드 쪽이 틀린 것**으로 확인되었다. `robot.yaml`을 `7=hipz 8=hipx 9=hipy`
  (왼쪽 `1=hipz 3=hipy`)로 정정했다. 원본 컨트롤러는 `hip_knee_ids` 언패킹이
  기능적이라 `set_leg_action(hipz=...)`가 모터 9를 움직이는 실제 버그였다.
- **RS00 MIT 인코딩 범위** — RS00 매뉴얼로 직접 확인하지 못했다
- **발목 기하값의 좌/우** 및 반대쪽 대칭 가정
- `README.md`가 `leg_control/` 기준이라 새 구조를 반영하지 않는다

---

## 8. 원본 → 신규 대응표

| 원본 | 신규 |
|---|---|
| `utils/mit_codec.py` | `motors/robstride/codec/mit.py` |
| `robot_constant.MOTORS`, `CAN_CMD_*` | `motors/robstride/tables.py` |
| `robot_constant.MOTOR_SIGN/OFFSET/LIMITS/GAINS` | `config/calibration/*.json` |
| `robot_constant.LEG_*_IDS`, `LEG_JOINT_NAMES` | `config/robot.yaml`, `config/robot.py` |
| `robot_constant.COMMAND_MARGIN_DEG` 등 | `config/robot.yaml` (`SafetyConfig`) |
| `robot_constant.CALIBRATED` | `calibration.store.is_complete()` |
| `ankle_kinematics.py` | `kinematics/ankle.py` |
| `_send8`, `_drain_rx_states`, `_recv_motor_reply` | `motors/canbus.py` |
| `_request_state`, `enable`/`disable`, `_send_cmd_ff` | `motors/robstride/bus.py` |
| `set_zero`, `set_zero_all`, `zero_here`, `_menu_set_zero` | `motors/robstride/commissioning.py` |
| `_raw_to_cal_near_interval`, `_resolve_raw_target_near_current`, `_raw_limits_near_ref`, `_wrap_near_center` | `safety/wrap.py` |
| `_raw_command_in_limits`, `_is_near_stop`, `_recompute_command_limits` | `safety/limits.py` |
| `_passes_action_update_jump_guard` | `safety/guards.py` |
| `_motor_to_joint`, `_joint_to_motor` | `robots/leg.py` |
| `_prepare_mit_payload` | `robots/leg.py` + `safety/guards.py`로 분해 |
| `_run_loop` | `control/loop.py` |
| `_step_motor_toward_raw` | `control/trajectory.py` (**동작 변경**) |
| `_log_state`, `_open_log` | `telemetry/csv_log.py` + `robots/leg.telemetry_snapshot()` |
| `__main__` 메뉴 | `scripts/bringup.py` |

---

## 9. 실행 방법

```bash
# 테스트 (python-can 불필요)
PYTHONPATH=src python3 -m pytest tests -q

# 브링업 메뉴 (python-can 필요)
pip install -e .
huphy-bringup --side right --allow-uncalibrated
# 또는
PYTHONPATH=src python3 -m huphy.scripts.bringup --side right --allow-uncalibrated
```

`--allow-uncalibrated`는 **벤치/시뮬 전용**이다. 실물에 토크를 걸기 전에
`config/calibration/*.json`의 sign/offset/limits/gains를 실측으로 채울 것.
