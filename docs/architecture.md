# 패키지 구조 설계 — 단일 다리에서 로봇 전체로

지금은 다리 하나(6모터·CAN 1버스)만 있지만, 이후 **양다리 → 로봇 전체**로 확장된다는
전제로 계층을 나눈다. 참고한 것은 [LeRobot](https://github.com/huggingface/lerobot)의
`motors/` · `robots/` · `configs/` 3층 분리이고, 이 프로젝트 사정에 맞춰
`safety/` · `control/` · `kinematics/`를 추가로 분리했다.

---

## 1. 계층과 의존 방향

의존은 **한 방향으로만** 흐른다. 이 규칙이 깨지면 구조가 무의미해진다.

```
scripts ──→ robots ──→ { motors, kinematics, safety, config }
                │
                └──→ telemetry   (단방향. telemetry는 아무것도 import 하지 않음)
```

| 계층 | 아는 것 | 모르는 것 |
|---|---|---|
| `motors/` | 모터 ID, 원시 각도, CAN 프레임 | "무릎"·"다리"가 무엇인지 |
| `kinematics/` | 링크 기하, 각도 | CAN, 모터 ID |
| `safety/` | 숫자, 한계값 | CAN, 상태 (**순수 함수, 상태 없음**) |
| `config/` | 값의 스키마 | 나머지 전부 |
| `robots/` | 관절 이름 ↔ 모터 매핑 | CAN 프레임 포맷 |
| `control/` | 궤적, 주기 | 프레임 인코딩 |
| `telemetry/` | dict 하나 | **로봇 구조를 전혀 모름** |

### 왜 `kinematics/` · `safety/`를 순수 함수로 빼는가

**하드웨어 없이 테스트하기 위해서다.** 현재 360° wrap 로직(`_raw_to_cal_near_interval`,
`_resolve_raw_target_near_current`)은 ±4바퀴 탐색과 구간중심 tie-break 같은 미묘한
규칙을 담고 있는데, CAN 버스와 락에 얽혀 있어 검증할 방법이 없다. 순수 함수로 빼면
단위 테스트가 가능해지고, 이후 리팩토링의 안전망이 된다.

### 왜 `telemetry/`에 스키마가 없는가

스키마는 "이 로봇이 무슨 관절을 갖는가"를 알아야 하는데, telemetry가 그걸 알면 로봇에
의존하게 된다. **LeRobot의 `observation_features` 패턴을 따른다** — 로봇이 자기 스키마를
선언하고, telemetry는 받은 dict를 직렬화만 한다. 그래야 다리 하나든 로봇 전체든 같은
telemetry 코드가 그대로 돈다. (→ [monitoring.md](monitoring.md) §4.4 "스키마 단일 정의")

---

## 2. 폴더 구조

```
HUPHY/
├── pyproject.toml                 # 단일 진실 공급원. sys.path 해킹 제거
├── src/huphy/
│   ├── motors/                          ← 모터 자체의 레코드 + 버스
│   │   ├── base.py                      #   벤더 중립: MotorState, MotorCalibration, MotorsBus(ABC)
│   │   ├── canbus.py                    #   python-can 래핑, TX/RX 락, 드레인
│   │   └── robstride/
│   │       ├── tables.py                #   [프로토콜][모델] 인코딩 범위, CAN 명령바이트,
│   │       │                            #   파라미터 인덱스, 열/부하 한계, 고장비트 정의
│   │       ├── codec/
│   │       │   ├── mit.py               #   MIT 표준 프레임 (11-bit ID) — 현재 사용
│   │       │   └── private.py           #   private 확장 프레임 (29-bit ID)
│   │       ├── params.py                #   파라미터 읽기/쓰기/플래시 저장
│   │       ├── commissioning.py         # ★ 1회성·영구 조작. 런타임 코드는 import 금지
│   │       └── bus.py                   #   런타임 전용: connect, enable/disable, 제어모드
│   │                                    #   전환, sync_read/sync_write, 고장 읽기
│   │
│   ├── kinematics/
│   │   └── ankle.py                     #   2모터 링키지 IK/FK
│   │
│   ├── safety/                          ← 전부 순수 함수
│   │   ├── wrap.py                      #   360° 표현 해소
│   │   ├── limits.py                    #   클램프, margin 계산
│   │   └── guards.py                    #   점프 가드, near-stop 판정
│   │
│   ├── config/                          ← 스키마(dataclass). 값이 아님
│   │   ├── motor.py                     #   MotorConfig(id, model, sign, offset, limits, gains)
│   │   ├── robot.py                     #   LegConfig, BipedConfig, RobotConfig(base)
│   │   └── loader.py                    #   YAML/JSON → dataclass
│   │
│   ├── robots/
│   │   ├── base.py                      #   Robot(ABC) + observation_features/action_features
│   │   ├── leg.py                       #   SingleLeg(Robot)
│   │   └── biped.py                     #   Biped(Robot) — Leg 2개 조합
│   │
│   ├── control/
│   │   ├── trajectory.py                #   절대 setpoint 램프 ← 옵션3 수정이 여기
│   │   └── loop.py                      #   제어 루프, 주기 관리
│   │
│   ├── telemetry/
│   │   ├── udp.py                       #   UdpTelemetry
│   │   └── csv_log.py                   #   CSV writer
│   │
│   └── scripts/
│       ├── bringup.py                   #   대화형 메뉴
│       ├── rom_diagnostic.py
│       └── zero_persistence.py
│
├── config/                        ← 실제 값 (코드 아님)
│   ├── robot.yaml
│   └── calibration/{left,right}_leg.json
├── layouts/                       # PlotJuggler 레이아웃 .xml
└── tests/
```

### 2.1 `motors/` 내부를 더 쪼개는 이유

벤더 드라이버를 파일 하나로 두는 것도 흔한 선택이다 (LeRobot의 `robstride.py`는
1,086줄 단일 파일). 그럼에도 나누는 이유가 셋 있다.

#### (1) 인코딩 범위는 "모델별"이 아니라 "프로토콜 × 모델"이다

RS02 하나에도 범위가 두 벌이다:

| 프로토콜 | 위치 | 속도 | 토크 |
|---|---|---|---|
| private (29-bit 확장 프레임) | ±12.57 rad | ±44 rad/s | ±17 N·m |
| **MIT (11-bit 표준 프레임)** | ±12.57 rad | **±33 rad/s** | ±17 N·m |

"모터 사양"이라는 단일 개념으로 뭉뚱그리면 프로토콜 축이 사라지고, 다른 프로토콜의
값을 가져다 쓰는 실수가 난다. **실제로 이 프로젝트와 LeRobot 양쪽에서 이미 발생한
오류다** (→ 부록). `tables.py`를 `[프로토콜][모델]`로 인덱싱하고 `codec/`을 프로토콜별로
나누면 구조가 이 실수를 막는다.

#### (2) 커미셔닝과 런타임은 성격이 정반대다

| | 커미셔닝 | 런타임 |
|---|---|---|
| 빈도 | 조립 시 1회 | 100 Hz |
| 지속성 | **플래시에 영구 저장** | 휘발 |
| 되돌리기 | 어려움 (전원 재투입 필요) | 즉시 |
| 예 | CAN ID 변경, 프로토콜 전환, 기계영점, 파라미터 저장, 엔코더 캘리브레이션 | enable/disable, 모션 명령, 상태 읽기 |

한 클래스에 섞으면 제어 루프 코드가 되돌리기 어려운 API에 손이 닿는다. 파일로 격리해
**런타임 모듈은 `commissioning.py`를 import하지 않는다**는 규칙을 둔다.

현재는 이 성격의 코드가 세 군데에 흩어져 있다 — `test_zero_persistence.py`(독립 스크립트),
`_menu_set_zero`(메뉴 함수), `set_zero_all`/`zero_here`(컨트롤러 메서드).

#### (3) 모션 명령 외의 경로가 둘 더 있다

MIT 프레임으로 위치를 보내는 것만이 모터와의 대화가 아니다.

- **파라미터 R/W** (`params.py`) — `limit_torque`, `CAN_TIMEOUT`, 속도루프 게인 등.
  인덱스 기반이고 모션 명령과 완전히 다른 프레임을 쓴다. 플래시 저장도 여기.
- **제어 모드 전환** (`bus.py`) — 모터는 MIT / Position / Velocity 세 모드를 지원하고
  명령으로 전환한다. 모드마다 입력 구성이 다르다 (MIT은 5개 파라미터, Position은
  목표위치+속도제한). 예를 들어 "영점으로 이동"은 모터가 궤적을 자체 생성하는
  Position 모드가 더 단순하고 안전할 수 있다.

#### `MotorSpec`을 `base.py`에서 뺀 이유

`base.py`는 벤더 중립이어야 하는데 `MotorSpec(pmax_rad, vmax_rad_s, tmax_nm)`은
**MIT류 프로토콜 특유의 개념**이다. CANopen 계열은 pulse 단위를 쓰고 이런 인코딩 범위
개념 자체가 없다. 벤더 중립 자리에는 `MotorState`/`MotorCalibration`/`MotorsBus(ABC)`만
두고, 인코딩 사양은 `robstride/tables.py`로 내린다.

---

## 3. 현재 코드의 이동 경로

| 현재 | 이동 후 | 성격 |
|---|---|---|
| `utils/mit_codec.py` | `motors/robstride/codec/mit.py` | 거의 그대로 |
| `robot_constant.py` `MOTORS`, `CAN_CMD_*` | `motors/robstride/tables.py` | **벤더 사양** (프로토콜 × 모델) |
| `robot_constant.py` `MOTOR_SIGN/OFFSET/LIMITS/GAINS` | `config/calibration/*.json` | **코드 → 데이터** |
| `robot_constant.py` `LEG_*_IDS`, `LEG_JOINT_NAMES` | `config/robot.yaml` | 토폴로지 |
| `robot_constant.py` `COMMAND_MARGIN_DEG` 등 | `config/robot.yaml` | 안전 파라미터 |
| `ankle_kinematics.py` | `kinematics/ankle.py` | 그대로 |
| `_send8`, `_drain_rx_states`, `_recv_motor_reply` | `motors/canbus.py` | 벤더 무관 |
| `_request_state`, `enable`/`disable`, `_send_cmd_ff` | `motors/robstride/bus.py` | **런타임** |
| `set_zero`, `set_zero_all`, `zero_here`, `_menu_set_zero` | `motors/robstride/commissioning.py` | **1회성·영구** |
| `test_zero_persistence.py` | `scripts/` + `commissioning.py` | 〃 |
| `_raw_to_cal_near_interval`, `_resolve_raw_target_near_current`, `_raw_limits_near_ref` | `safety/wrap.py` | **순수 함수화** |
| `_raw_command_in_limits`, `_is_near_stop`, `_recompute_command_limits` | `safety/limits.py` | 순수 함수화 |
| `_passes_action_update_jump_guard` | `safety/guards.py` | 순수 함수화 |
| `_motor_to_joint`, `_joint_to_motor` | `robots/leg.py` | 로봇 의미론 |
| `_run_loop` | `control/loop.py` | |
| `_step_motor_toward_raw` | `control/trajectory.py` | **여기서 절대 setpoint 램프로 수정** |
| `_log_state`, `_open_log` | `telemetry/csv_log.py` + `robots/base.py` 스키마 | |
| `__main__` 메뉴 | `scripts/bringup.py` | |
| `run_full_rom_diagnostic` | `scripts/rom_diagnostic.py` | |

### 핵심: `robot_constant.py`는 세 갈래로 쪼개진다

지금 한 파일에 **벤더 사양 · 로봇 토폴로지 · 캘리브레이션**이 섞여 있다. 이것이
`CALIBRATED = False`가 풀리지 않는 구조적 이유다 — 값을 하나 실측할 때마다 소스를
고쳐야 하고, 좌우 다리가 하나의 전역을 공유하며, 측정 결과가 코드 리뷰 대상이 된다.

| 갈래 | 성격 | 위치 |
|---|---|---|
| 벤더 사양 (pmax/vmax/tmax, CAN 명령바이트) | 데이터시트에서 옴. 절대 안 바뀜 | `motors/robstride/tables.py` |
| 로봇 토폴로지 (모터 ID ↔ 관절, 버스 배치) | 조립 구성. 드물게 바뀜 | `config/robot.yaml` |
| 캘리브레이션 (sign/offset/limits/gains) | **실측값. 자주 바뀜** | `config/calibration/*.json` |

---

## 4. 확장 시나리오 검증

설계가 맞는지 보는 방법 — 확장할 때 **건드리는 파일이 적어야** 한다.

| 확장 | 건드리는 곳 |
|---|---|
| 다리 1개 → 2개 | `config/robot.yaml`만. `Biped`가 `SingleLeg` 2개를 들면 끝 |
| 팔 추가 | `robots/arm.py` 신규 + yaml. motors/safety/kinematics **무변경** |
| 다른 벤더 모터 | `motors/<vendor>/` 신규. `MotorsBus` 인터페이스만 맞추면 robots **무변경** |
| RS02 → RS03 교체 | yaml의 `model`만. `tables.py`에 이미 있음 |
| 고장비트 필요 | `robstride/private_codec.py` 추가. bus가 두 프로토콜 병용 |
| 발목 기구 변경 | `kinematics/ankle.py`만 |
| IMU 추가 | `sensors/` 신규 + `observation_features`에 필드 추가 |

---

## 5. 이행 순서

**한 번에 하지 않는다.** 검증 수단이 없는 상태에서 1,566줄을 통째로 쪼개면 되돌릴 수
없다. 다만 **뼈대는 먼저 잡는다** — 빈 패키지 생성은 비용이 0이고 `git mv`도 싸다.

| 단계 | 내용 | 위험도 |
|---|---|---|
| 0 | 첫 커밋 | — |
| 1 | `pyproject.toml` + 디렉터리 뼈대 + **순수 이동만** (`mit_codec`, `ankle_kinematics`). 로직 변경 0 | 낮음 |
| 2 | `tables.py` 분리 — 벤더 사양을 `robot_constant`에서 빼기 | 낮음 |
| 3 | **`telemetry/` 작성** — 제자리에. `SingleLeg`는 아직 통짜여도 됨 | 낮음 |
| 4 | `safety/` 추출 + **테스트 작성** ← 검증 수단 확보 | 중간 |
| 5 | `config/` + 캘리브레이션 JSON 외부화 | 중간 |
| 6 | `motors/robstride/bus.py` 추출 | 높음 |
| 7 | `control/` 분리 + **옵션3 수정** (텔레메트리로 전후 비교) | 높음 |
| 8 | `robots/biped.py` | — |

### 3번이 4번보다 앞인 이유

텔레메트리가 있어야 6~7단계 리팩토링에서 **"바꿔도 응답이 같다"를 확인**할 수 있다.
3번 시점의 `SingleLeg`는 현재 클래스를 파일만 옮긴 상태여도 무방하다.

---

## 6. 미결정 사항

| # | 항목 | 선택지 |
|---|---|---|
| 0 | **모터의 통신 프로토콜 모드** ❗ | RobStride 공장 기본값은 private(29-bit)이고 현재 코드는 MIT(11-bit)를 가정한다. 틀리면 **명령이 무시되고 에러도 안 난다** — 연결도 되고 코드도 안 죽는데 모터만 안 움직인다. 확인: motorstudio로 `0x201F protocol_1` 읽기, 또는 11-bit `enable`에 응답이 오는지. 결과에 따라 A(`set_protocol`로 전환 + 전원 재투입) 또는 B(`codec/private.py` 구현) 선택 |
| a | `MotorsBus` 인터페이스를 LeRobot과 맞출 것인가 | `sync_read`/`sync_write`/`enable_torque`/`read_calibration` 이름을 그대로 쓰면 이후 LeRobot 생태계 연결 시 어댑터가 얇아진다. 대신 현재 코드와 이름이 달라 이행 비용이 든다 |
| b | config 포맷 | YAML / JSON / draccus dataclass. draccus는 의존성이 늘지만 CLI 인자가 공짜로 생긴다 (LeRobot 방식) |
| c | 캘리브레이션 파일 단위 | 다리별 / 모터별 / 로봇 통짜. LeRobot은 로봇 1대당 1파일. 현재는 오른쪽 다리만 연결되어 있어 다리별이 맞아 보인다 |

---

## 부록: LeRobot 대조

참고용으로 클론해둔 `lerobot/`(저장소 루트, git 추적 제외)의 대응 위치.

| HUPHY | LeRobot |
|---|---|
| `motors/base.py` | `src/lerobot/motors/motors_bus.py` (`Motor`, `MotorCalibration`, `MotorsBusBase`) |
| `motors/robstride/` **전체** | `src/lerobot/motors/robstride/robstride.py` **단일 1,086줄** |
| `robots/base.py` | `src/lerobot/robots/robot.py` (`Robot` ABC) |
| `config/robot.py` | `src/lerobot/robots/config.py` (`RobotConfig`) |
| `safety/` | `src/lerobot/robots/utils.py`의 `ensure_safe_goal_position` (HUPHY는 안전 로직이 훨씬 많아 별도 패키지로 분리) |

### ⚠️ 인코딩 범위는 프로토콜에 따라 다르다

RS02 매뉴얼 기준:

| 프로토콜 | 위치 | 속도 | 토크 | 출처 |
|---|---|---|---|---|
| private (29-bit 확장) | ±12.57 rad | ±44 rad/s | ±17 N·m | p.20~21 Communication Type 1 / 2 |
| **MIT (11-bit 표준)** | ±12.57 rad | **±33 rad/s** | ±17 N·m | p.37~38 Command 3 / Response Command 1 |

HUPHY는 11-bit 표준 프레임을 쓰므로 **MIT 행(±33)이 맞다.**

- 현재 `robot_constant.py`의 hip/knee `vmax = 44.0`은 **private 프로토콜 값**이라 수정
  대상이다 (속도 읽기가 실제보다 1.33배 크게 나옴). 위치·토크는 영향 없음.
- LeRobot의 `MotorType.O2: (12.57, 33, 20)`은 속도가 맞고 **토크가 틀렸다.**
  17 N·m는 매뉴얼 4곳에서 교차 확인된다 — MIT 섹션, private 섹션,
  p.10 파라미터 `0x2007 limit_torque`(max 17 / 기본 17), p.3 Peak load 17 N·m.
- 매뉴얼 자체에도 오기재가 있다. p.26 "Read and write a single parameter list"의
  `limit_torque 0 to 14Nm`은 RS00 값으로 보인다.
- 과열 임계는 MIT 섹션이 130°C, private 섹션이 135°C로 다르다. 보수적으로 130 채택.

**사양값은 반드시 "어느 프로토콜의 표에서 왔는지"와 함께 기록할 것.**
