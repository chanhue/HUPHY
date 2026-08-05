# `config/` — 설정 값

**코드가 아니라 데이터다.** 스키마(dataclass)는 `src/huphy/config/`에 있다.

```
config/
├── robot.yaml              로봇 토폴로지 + 안전 파라미터
└── calibration/
    ├── right_leg.json      실측 캘리브레이션
    └── left_leg.json
```

---

## 값이 세 군데로 나뉘는 이유

| 성격 | 예 | 위치 | 변경 빈도 | 출처 |
|---|---|---|---|---|
| 벤더 사양 | 인코딩 범위, CAN 명령 바이트 | `src/huphy/motors/robstride/tables.py` | 모터를 바꿔야 | **데이터시트** |
| 로봇 토폴로지 | 모터 ID ↔ 관절, 버스 배치 | `robot.yaml` | 조립이 바뀌면 | **사람이 정함** |
| 실측 캘리브레이션 | sign, offset, limits, gains | `calibration/*.json` | **자주** | **실측** |

원본 `robot_constant.py`는 이 셋이 한 파일에 섞여 있었다. 값 하나를 실측할 때마다
소스를 고쳐야 했고, 그것이 `CALIBRATED = False`가 풀리지 않은 구조적 이유였다.

---

## `robot.yaml`

### `safety` (전역 기본값)

```yaml
safety:
  margins:
    command_deg: 3.0      # 명령을 한계에서 이만큼 안쪽으로
    state_deg: 5.0        # 측정값이 이만큼 넘으면 E-STOP
    near_stop_deg: 8.0    # 이만큼 가까우면 감쇠 전용으로 전환
  max_cmd_delta_deg: 50.0
  damping_kd: 1.0
```

**`command ≤ state ≤ near_stop` 순서가 강제된다.** `near_stop`이 가장 커야 "한계에
닿기 한참 전에 부드럽게 감속"이 성립한다. 다리별로 `safety` 블록을 두면 전역을 덮어쓴다.

### `telemetry`

```yaml
telemetry:
  host: null                   # 비우면 UDP 비활성. 환경변수 PJ_HOST로도 지정 가능
  port: 9870
  csv_path: ../logs/leg.csv    # yaml 위치 기준 상대 경로
  csv_flush_every: 50
  joint_field_decimation: 1    # 발목 FK 비용 때문에 낮출 수 있다
```

### `legs`

```yaml
legs:
  right:
    channel: can1
    control_hz: 100.0
    rom_scale: 0.3
    calibration: calibration/right_leg.json
    motors:
      - {id: 7,  role: hipz,     model: RS02}
      - {id: 8,  role: hipx,     model: RS02}
      - {id: 9,  role: hipy,     model: RS02}
      - {id: 10, role: knee,     model: RS02}
      - {id: 11, role: ankle_a1, model: RS00}
      - {id: 12, role: ankle_a2, model: RS00}
```

**role**: `hipy` / `hipx` / `hipz` / `knee` / `ankle_a1` / `ankle_a2` (6개 전부 필요)
**model**: `tables.py`의 `Model` enum 값

로더가 role 누락·중복, 모터 id 중복을 검사한다.

### 모터 id ↔ 관절 매핑 — 원본 코드의 버그를 바로잡았다

원본 `robot_constant.py`는 **주석과 코드가 서로 달랐다.**

| 위치 | 종류 | 매핑 |
|---|---|---|
| 상단 토폴로지 주석 | 주석 | `7=hipz 8=hipx 9=hipy` ← **맞음** |
| `JOINT_LIMITS_DEG` 주석 | 주석 | `7=hipz 8=hipx 9=hipy` ← 맞음 |
| `_make_leg_specs`의 `joint_order` | 코드 | `7=hipy 8=hipx 9=hipz` ← 틀림 |
| 컨트롤러 `hip_knee_ids` 언패킹 | 코드 | `7=hipy 8=hipx 9=hipz` ← 틀림 |

**코드에서 `hipz`와 `hipy`가 뒤바뀌어 있었다.** `_make_leg_specs` 쪽은 표시용 이름만
틀렸지만, 컨트롤러의 언패킹은 기능적이라 실제 버그였다 — 그 상태로
`set_leg_action(hipz=30)`을 부르면 **모터 9가 움직인다.**

여기서는 올바른 쪽(주석)을 따른다: `7=hipz 8=hipx 9=hipy`, 왼쪽은 `1=hipz 3=hipy`.

`hipx`(8, 2)와 `knee`(10, 4)는 양쪽이 일치해서 손대지 않았다.

> 캘리브레이션 JSON은 **모터 id로 키가 잡혀 있어** 이 수정의 영향을 받지 않는다.
> 한계값은 이름이 아니라 모터에 붙어 있다.

---

## `calibration/*.json`

### 형식

```json
{
  "schema_version": 1,
  "note": "사람이 남기는 메모",
  "motors": {
    "7": {
      "sign": 1.0,
      "offset_deg": 0.0,
      "limit_lo_deg": -117.07,
      "limit_hi_deg": 21.07,
      "kp": 0.0,
      "kd": 0.0,
      "zero_reference": ""
    }
  }
}
```

`calibrated_deg = sign * raw_deg + offset_deg`
한계값은 **raw 공간**이다.

### 각 항목 측정 방법

| 항목 | 방법 |
|---|---|
| `zero_reference` | 기계 영점을 **어느 자세에서 잡았는지** 기록. 모터에도 코드에도 안 남는다 |
| `sign` | 무동력으로 관절을 + 방향으로 밀고 raw가 증가하는지 관찰 |
| `offset_deg` | 기준 자세에서 읽은 raw 값 |
| `limit_lo/hi_deg` | 무동력으로 하드스톱까지 움직여 raw를 읽고 여유를 뺀다 |
| `kp` / `kd` | 낮게 시작해 응답을 보며 올린다 (**텔레메트리 필요**) |

> `sign`이 반대면 목표에서 **멀어지는 방향**으로 토크가 걸린다. 가장 먼저 확인할 값이다.

### 현재 상태

| 다리 | 한계 | sign/offset | kp/kd | zero_reference |
|---|---|---|---|---|
| right | 실측으로 보임 | 미실측 (1.0 / 0.0) | **미실측 (0.0)** | 비어 있음 |
| left | 없음 (`null`) | 미실측 | 미실측 | 비어 있음 |

양쪽 모두 `is_complete() == False`이므로 `allow_uncalibrated=True` 없이는 다리 객체를
만들 수 없다. **의도된 동작이다.**

`limit`이 `null`이면 "한계 검사만 건너뛰고 동작"하는 것이 아니라 **제어 진입 자체가
막힌다.**

---

## 이 파일들을 누가 읽나

```
config/robot.yaml
    │  huphy.config.loader.load_robot_config()
    │      ← huphy.robots.factory.build()
    │          ← huphy.scripts.bringup.main()
    ▼
RobotConfig / LegConfig / SafetyConfig / TelemetryConfig
    │
    ├──→ robots/leg.py       모터 매핑, 안전 마진, 발목 id, rom_scale
    ├──→ robots/factory.py   버스 생성 (channel, interface, recv_timeout_s)
    ├──→ control/loop.py     control_hz, joint_field_decimation
    └──→ telemetry/          host, port, csv_path, csv_flush_every

config/calibration/*.json
    │  huphy.calibration.store.load()
    │      ← huphy.robots.leg.SingleLeg.__init__ / .calibrate()
    ▼
dict[int, MotorCalibration]
    │
    ├──→ raw_to_cal / cal_to_raw   좌표 변환 (robots/leg.py)
    ├──→ .limits                    한계 검사, margin (safety/limits.py 경유)
    ├──→ .kp / .kd                  MIT 프레임에 실림 (motors/robstride/codec)
    └──→ .zero_reference            사람이 읽는 기록

    ▲  huphy.calibration.store.save()
    └── scripts/bringup.py 메뉴 1번 (zero_reference 메모 갱신)
```

**경로 해석 규칙**: yaml 안의 상대 경로(`calibration: calibration/right_leg.json`,
`csv_path: ../logs/leg.csv`)는 **yaml 파일이 있는 디렉터리 기준**으로 풀린다
(`config/loader.py`의 `base = p.parent`).

**쓰기가 일어나는 유일한 곳은 `scripts/`다.** 런타임 제어 경로는 설정과
캘리브레이션을 읽기만 한다 — 제어 중에 실측값이 바뀌면 안 되기 때문이다.

---

## 전제조건: CAN 인터페이스

설정 파일이 아니라 시스템에서 먼저 올려야 한다.

```bash
sudo ip link set can1 up type can bitrate 1000000
ip -details link show can1
```
