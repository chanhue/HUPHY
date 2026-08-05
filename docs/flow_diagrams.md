# 호출 관계와 흐름도

`src/huphy/`의 구조를 그림으로. 소스에서 추출한 실제 import 관계와 호출 순서다.

> GitHub에서는 바로 렌더링된다. VSCode에서 보려면 Markdown Preview Mermaid Support
> 확장이 필요하다.

---

## 1. 모듈 의존 그래프 (정적)

화살표 = **"A가 B를 import한다"**. 의존은 위에서 아래로만 흐른다.

```mermaid
flowchart TD
    subgraph L5["진입점"]
        BRINGUP["scripts/bringup.py"]
    end

    subgraph L4["로봇 · 제어"]
        FACTORY["robots/factory.py"]
        LEG["robots/leg.py<br/>SingleLeg"]
        RBASE["robots/base.py<br/>Robot ABC"]
        LOOP["control/loop.py<br/>LegControlLoop"]
        TRAJ["control/trajectory.py<br/>SetpointRamp"]
    end

    subgraph L3["설정 · 실측값"]
        CFG["config/robot.py<br/>loader.py"]
        CAL["calibration/store.py"]
    end

    subgraph L2["순수 계산"]
        SAFE["safety/<br/>wrap · limits · guards"]
        KIN["kinematics/ankle.py"]
    end

    subgraph L1["모터 · 통신"]
        BUS["motors/robstride/bus.py<br/>RobStrideBus"]
        COMM["motors/robstride/<br/>commissioning.py"]
        CODEC["motors/robstride/<br/>codec/mit.py"]
        TABLES["motors/robstride/<br/>tables.py"]
        CAN["motors/canbus.py<br/>CanBus"]
        MBASE["motors/base.py"]
    end

    subgraph L0["관찰"]
        TELE["telemetry/<br/>udp · csv_log"]
    end

    BRINGUP --> FACTORY & LEG & TRAJ & SAFE & COMM & CAL & TABLES
    FACTORY --> CFG & LEG & CAN & BUS & TELE
    LOOP --> LEG
    LEG --> RBASE & CFG & KIN & SAFE & CAL & BUS & MBASE
    CFG --> SAFE & TABLES
    CAL --> MBASE
    BUS --> MBASE & CAN & CODEC & TABLES
    COMM --> CAN & CODEC & TABLES
    CODEC --> TABLES

    classDef pure fill:#e8f5e9,stroke:#43a047,color:#111
    classDef danger fill:#fff3e0,stroke:#fb8c00,color:#111
    classDef orphan fill:#eee,stroke:#999,stroke-dasharray:4,color:#111
    class SAFE,KIN,TRAJ,TABLES,CODEC,MBASE pure
    class COMM danger
    class LOOP orphan
```

| 표시 | 의미 |
|---|---|
| 🟩 초록 | **순수 계층** — `python-can` 없이 import·테스트된다 |
| 🟧 주황 | **커미셔닝** — 되돌리기 어려운 영구 조작. `scripts/`만 부른다 |
| ⬜ 점선 | **미연결** — 아직 아무도 import하지 않는다 |

**규칙 확인**: `safety/`·`kinematics/`가 `motors/`를 가리키는 화살표가 없고,
`telemetry/`에서 나가는 화살표가 하나도 없다. `motors/`에서 위로 올라가는 화살표도 없다.

---

## 2. 조립 순서 — `build()`가 하는 일

```mermaid
sequenceDiagram
    autonumber
    participant S as scripts/bringup
    participant F as robots/factory
    participant LD as config/loader
    participant CS as calibration/store
    participant C as motors/CanBus
    participant B as RobStrideBus
    participant L as SingleLeg

    S->>F: build("right", allow_uncalibrated)
    F->>LD: load_robot_config()
    LD-->>F: RobotConfig (legs, telemetry, safety)
    F->>C: CanBus(interface, channel)
    F->>B: RobStrideBus(can_bus, models)
    Note over B: 모터별 EncodingRange 결정<br/>encoding_for(model, MIT)
    F->>L: SingleLeg(leg_cfg, bus)
    L->>CS: load(calibration_path)
    CS-->>L: dict[int, MotorCalibration]
    L->>CS: is_complete() ?
    alt 미완 && not allow_uncalibrated
        L-->>S: RuntimeError (모터별 미완 사유)
    end
    F->>F: build_telemetry() → TelemetrySink
    F-->>S: (cfg, leg, sink)
    S->>L: connect()
    L->>C: connect() → SocketCAN 열기
```

---

## 3. 제어 한 사이클 — `LegControlLoop.step()`

```mermaid
flowchart TD
    START([사이클 시작]) --> DT["loop_dt 측정<br/>LoopTiming.tick"]
    DT --> RESET["rejects.reset()"]
    RESET --> MODE{mode?}

    MODE -->|state_only| SO1["clear_estop()<br/>(설정에 따라)"]
    SO1 --> SO2["bus.sync_read_states()"]
    SO2 --> SO3["disable_torque()<br/>최초 1회만"]
    SO3 --> EMIT

    MODE -->|control| C1{"stale 상태<br/>있나?"}
    C1 -->|있음| C2["sync_read_states(stale)"]
    C1 -->|없음| C3
    C2 --> C3["check_state_bounds()"]
    C3 --> C4{한계 초과?}
    C4 -->|초과| ESTOP["trip_estop()<br/>disable_torque()"]
    ESTOP --> EMIT
    C4 -->|정상| C5{"목표<br/>초기화됨?"}
    C5 -->|아니오| C6["latch_target_from_state()<br/>현재 자세를 목표로"]
    C5 -->|예| C7
    C6 --> C7{"update_damping()<br/>한계 근접?"}
    C7 -->|근접| DAMP["send_damping()<br/>kp=0, kd만"]
    C7 -->|여유| ACT["send_action()"]
    DAMP --> EMIT
    ACT --> EMIT

    EMIT["_emit()<br/>telemetry_snapshot + mode/loop_dt/cycle"]
    EMIT --> SINK["on_snapshot(snap)"]
    SINK --> SLEEP["다음 tick까지 sleep<br/>밀렸으면 overruns++"]
    SLEEP --> START

    classDef danger fill:#ffebee,stroke:#e53935,color:#111
    classDef warn fill:#fff3e0,stroke:#fb8c00,color:#111
    class ESTOP danger
    class DAMP warn
```

**`update_damping()`이 하나라도 걸리면 다리 전체가 감쇠로 간다.** 범인 모터는
`_damping_culprit`에 기록되어 스냅샷으로 나간다.

---

## 4. `send_action()` 내부 — 관절 공간에서 CAN 프레임까지

```mermaid
flowchart LR
    T["목표 (관절 공간)<br/>hipz/hipx/hipy/knee<br/>ankle_pitch/ankle_roll"] --> J2M

    subgraph J2M["joint_to_motor()"]
        HK["hip·knee<br/>cal.cal_to_raw()"]
        AK["발목<br/>ankle.solve_ik()"]
    end

    J2M -->|"AnkleUnreachableError"| REJ0["rejects_ik++<br/>전송 없음"]
    J2M --> PER

    subgraph PER["모터별 (6회)"]
        direction TB
        P1["bus.state(mid)<br/>캐시된 실측"] --> P2{"유효?"}
        P2 -->|아니오| RJ1["rejects_nostate++"]
        P2 -->|예| P3["wrap.resolve_target_near_current<br/>인코딩 범위 안 표현 선택"]
        P3 --> P4["guards.check_command"]
        P4 -->|거부| RJ2["rejects_limit / jump ++"]
        P4 -->|통과| P5["bus.build_mit_frame<br/>codec.pack_command"]
    end

    PER --> SW["bus.sync_write_mit(frames)"]
    SW --> SEND["can.send_many()<br/>락 1회로 연속 전송"]
    SEND --> DRAIN["can.drain(_ingest)<br/>응답 일괄 수거"]
    DRAIN --> CACHE["_state 캐시 갱신"]
    SW --> LAST["_last_sent_raw 저장<br/>← err 계산의 기준"]

    classDef rej fill:#ffebee,stroke:#e53935,color:#111
    class REJ0,RJ1,RJ2 rej
```

> `_last_sent_raw`가 **실제로 프레임에 실린 값**이다. `err = tgt − pos`의 `tgt`가
> 이것이어야 모터 펌웨어 PD가 보는 오차와 일치한다.

---

## 5. 브링업 메뉴 경로 — 지금 실제로 도는 것

⚠️ **메뉴는 §3, §4를 거치지 않는다.** 버스를 직접 호출한다.

```mermaid
flowchart TD
    M{"메뉴 선택"}

    M -->|1| Z1["bus.disable_torque"]
    Z1 --> Z2["commissioning.set_mechanical_zero<br/>0xFE — 플래시에 저장"]
    Z2 --> Z3["calstore.save<br/>zero_reference 메모"]

    M -->|2·3| MV

    subgraph MV["_move_to() — 절대 setpoint 램프"]
        direction TB
        V1["bus.sync_read_states<br/>실측 1회"] --> V2["wrap.wrap_near<br/>최단 경로 목표"]
        V2 --> V3["SetpointRamp.starting_at<br/>setpoint 초기화"]
        V3 --> V4["ramp.advance(goal)<br/>← 직전 setpoint에서 전진"]
        V4 --> V5["bus.state(mid) 캐시 읽기"]
        V5 --> V6["wrap.resolve_target_near_current"]
        V6 --> V7["guards.check_command"]
        V7 --> V8["bus.send_mit → can.drain"]
        V8 --> V9{"ramp.at_goal?<br/>setpoint 기준"}
        V9 -->|아니오| V4
        V9 -->|예| VDONE([도달])
    end

    M -->|4| S1["bus.sync_read_states<br/>leg.limit_margins<br/>calstore.missing_report"]
    M -->|5| F1["bus.read_fault<br/>고장 비트 해석"]

    classDef comm fill:#fff3e0,stroke:#fb8c00,color:#111
    class Z2 comm
```

**§4와 비교했을 때 빠지는 것**: 관절 공간 변환, 발목 IK, `rejects` 집계,
`telemetry_snapshot`. 메뉴는 모터 하나만 다루므로 관절 공간이 필요 없어서인데,
그 결과 텔레메트리가 흐르지 않는다.

---

## 6. 데이터 흐름 — 설정과 캘리브레이션

```mermaid
flowchart LR
    subgraph FILES["파일 (코드 아님)"]
        YAML["config/robot.yaml<br/>토폴로지 · 안전 마진"]
        JSON["config/calibration/*.json<br/>sign · offset · limits · gains"]
    end

    subgraph VENDOR["코드 (데이터시트)"]
        TB["tables.py<br/>인코딩 범위 · 명령 바이트"]
    end

    YAML -->|load_robot_config| RC["RobotConfig<br/>LegConfig · SafetyConfig"]
    JSON -->|"store.load()"| MC["dict int→MotorCalibration"]

    RC --> LEG2["SingleLeg"]
    MC --> LEG2
    TB --> ENC["encoding_for(model, MIT)"]
    ENC --> BUS2["RobStrideBus"]
    BUS2 --> LEG2

    LEG2 --> USE1["raw_to_cal / cal_to_raw"]
    LEG2 --> USE2["한계 검사 · margin"]
    LEG2 --> USE3["kp · kd → MIT 프레임"]

    BRING["scripts/bringup<br/>메뉴 1번"] -.->|"store.save()"| JSON

    classDef file fill:#e3f2fd,stroke:#1e88e5,color:#111
    class YAML,JSON file
```

**쓰기가 일어나는 유일한 곳은 `scripts/`다.** 런타임 제어 경로는 읽기만 한다.

---

## 7. 텔레메트리 경로 — 그리고 끊긴 지점

```mermaid
flowchart TD
    SNAP["robots/leg.telemetry_snapshot()<br/>스키마의 유일한 정의 지점<br/>83필드 ≈ 1.2KB"]
    SNAP --> EMIT2["control/loop._emit()<br/>+ mode · loop_dt · cycle"]
    EMIT2 --> SINK2["TelemetrySink(snapshot)"]
    SINK2 --> UDP["UdpTelemetry.send<br/>JSON → UDP :9870"]
    SINK2 --> CSV["CsvLogger.write<br/>헤더 = 첫 스냅샷의 keys()"]
    UDP --> PJ["PlotJuggler<br/>우분투 PC"]
    CSV --> DISK["보드 디스크<br/>N사이클 버퍼링<br/>E-STOP 시 즉시 flush"]

    BR["scripts/bringup<br/>메뉴 루프"] -.->|"❌ 호출 안 함"| SNAP
    BR -->|"sink.close()만"| SINK2

    classDef broken fill:#ffebee,stroke:#e53935,stroke-dasharray:5,color:#111
    class BR broken
```

**`control/loop.py`를 아무도 쓰지 않으므로 이 경로 전체가 아직 안 돈다.**
`factory.build()`가 `TelemetrySink`를 만들지만 `bringup.py`는 `close()`만 부른다.

잇는 방법 두 가지:
1. `_move_to` 루프 안에서 `leg.telemetry_snapshot()`을 만들어 sink로
2. 메뉴를 `LegControlLoop` 기반으로 다시 쓰기

---

## 8. CAN 프레임 한 왕복

```mermaid
sequenceDiagram
    autonumber
    participant L as SingleLeg
    participant B as RobStrideBus
    participant CD as codec/mit
    participant C as CanBus
    participant M as 모터 (RS02)

    L->>B: build_mit_frame(mid, pos, kp, kd)
    B->>CD: pack_command(..., enc)
    Note over CD: 위치 16bit / 속도 12bit<br/>kp 12 / kd 12 / 토크 12<br/>= 8바이트
    CD-->>B: bytes(8)
    L->>B: sync_write_mit({mid: frame})
    B->>C: send_many(frames)
    Note over C: TX 락 1회로 연속 전송<br/>프레임 섞임 방지
    C->>M: 11-bit ID + 8바이트
    Note over M: τ = kp·(cmd−meas)<br/>  + kd·(0−vel) + τ_ff
    M-->>C: 응답 프레임 (id, pos, vel, tau, temp)
    B->>C: drain(_ingest)
    Note over C: 큐가 빌 때까지.<br/>마지막 recv가 timeout만큼 블로킹
    C->>B: _ingest(msg)
    B->>CD: decode_state(data, enc)
    CD-->>B: (id, pos_deg, vel_deg_s, tau_nm, temp_c)
    B->>B: _state[mid] 갱신 (락)
    L->>B: states() → 즉시 반환 (배선 안 탐)
```

**상태 읽기가 O(1)인 이유**: `states()`는 캐시를 읽을 뿐 버스로 나가지 않는다.
다만 캐시를 **채우는** `drain()`이 현재는 제어 루프 안에서 돈다
(→ [motors/README.md](../src/huphy/motors/README.md)의 RX 3단계).

---

## 9. 한눈에 — 계층별 책임

```mermaid
flowchart LR
    A["관절 각도<br/>hipz = 30°"] -->|"robots/leg"| B["모터 raw 각도<br/>m7 = 30°"]
    B -->|"safety/wrap"| C["360° 표현 해소<br/>m7 = 30° or 390°"]
    C -->|"safety/guards"| D{"한계 · 점프<br/>통과?"}
    D -->|"codec/mit"| E["8바이트<br/>0x1A 0x2B ..."]
    E -->|"motors/canbus"| F["CAN 프레임<br/>11-bit ID"]
    F --> G["모터 펌웨어 PD"]

    classDef reject fill:#ffebee,stroke:#e53935,color:#111
    D -.->|거부| X["rejects++"]
    class X reject
```

각 화살표가 한 계층이다. **위로 갈수록 사람의 언어, 아래로 갈수록 기계의 언어.**
