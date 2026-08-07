# 호출 관계와 흐름도

누가 누구를 부르는지. 계층을 나눈 **이유**는 [architecture.md](architecture.md),
사용법은 [루트 README](../README.md).

---

## 1. 모듈 의존 그래프

```mermaid
graph TD
    subgraph 진입점
        COMMISSION[scripts/commission.py]
        BRINGUP[scripts/bringup.py]
    end

    subgraph 제어
        LOOP[control/loop.py]
        MOTIONS[control/motions.py]
    end

    subgraph 경계
        LEG[robots/leg.py]
        RBASE[robots/base.py]
    end

    subgraph 순수계산
        ANKLE[kinematics/ankle.py]
        GUARDS[safety/guards.py]
        LIMITS[safety/limits.py]
    end

    subgraph 값
        CFG[config/loader.py]
        SCHEMA[config/schema.py]
        CAL[calibration/store.py]
    end

    subgraph 모터
        MBASE[motors/base.py]
        CANBUS[motors/canbus.py]
        RBUS[robstride/bus.py]
        CODEC[robstride/codec/mit.py]
        TABLES[robstride/tables.py]
        COMM[robstride/commissioning.py]
    end

    subgraph 관측
        TELE[telemetry/]
    end

    BRINGUP --> LOOP
    BRINGUP --> MOTIONS
    BRINGUP --> LEG
    BRINGUP --> TELE
    BRINGUP --> CFG
    COMMISSION --> COMM
    COMMISSION --> CFG
    COMMISSION --> CAL

    LOOP --> RBASE
    LOOP -.기록.-> TELE
    MOTIONS -.순수.-> MOTIONS

    LEG --> RBASE
    LEG --> ANKLE
    LEG --> GUARDS
    LEG --> CAL
    LEG --> SCHEMA
    LEG --> RBUS

    GUARDS --> LIMITS
    CFG --> SCHEMA
    SCHEMA --> MBASE
    CAL --> MBASE

    RBUS --> MBASE
    RBUS --> CANBUS
    RBUS --> CODEC
    RBUS --> TABLES
    COMM --> RBUS
    CODEC --> TABLES

    TELE -.읽기만.-> RBASE

    style CANBUS fill:#ffe0e0
    style TELE fill:#e0f0ff
```

**`canbus.py` 만 `python-can` 을 씀** (붉은색). 그것도 함수 안에서 import 함.

**`telemetry/` 는 아무도 부르지 않음** (파란색). `Robot` 계약을 읽기만 함.

---

## 2. 조립 — `bringup.build_leg()`

```mermaid
graph LR
    Y[config/robot.yaml] --> L[load_robot]
    L --> RC[RobotConfig]
    RC --> LC[LimbConfig]

    LC --> G{side}
    G -->|left| MIR[AnkleGeometry.mirrored]
    G -->|right| ORI[AnkleGeometry]
    MIR --> AK[AnkleKinematics]
    ORI --> AK

    LC --> CB[CanBus]
    CB --> RB[RobStrideBus]
    LC -->|motors_by_id| RB

    J[calibration/*.json] --> CL[calibration.load]
    CL --> AT[attach]
    LC -->|motors| AT

    RB --> LEG[Leg]
    AK --> LEG
    AT --> LEG
    RC -->|safety| LEG
```

`motors_by_id()` 가 경계임 — **여기서 관절 이름을 버리고 모터 id 만 넘김.**
`RobStrideBus` 는 "무릎" 이 무엇인지 모름.

`attach()` 도 같음 — 관절 이름 키의 캘리브레이션을 모터 id 키로 다시 잡음.
양쪽 관절 이름이 정확히 같아야 하고, 하나만 빠져도 에러임.

---

## 3. 제어 한 사이클 — `ControlLoop.step()`

```mermaid
sequenceDiagram
    participant L as ControlLoop
    participant M as Motion
    participant R as Leg
    participant B as RobStrideBus
    participant C as CanBus
    participant T as Telemetry

    L->>R: get_observation()
    R-->>L: {knee.pos: ..., ...}   cal 공간

    alt CONTROL 모드
        L->>M: motion(t, obs)
        M-->>L: {knee: 30.0, ...}  관절 이름
        L->>R: build_commands(action)
        Note over R: 계산만. CAN 안 씀
        R-->>L: {10: MitCommand, ...}
        L->>R: send(commands)
        R->>B: send_mit()
        B->>C: send_many()
        L->>R: collect()
        R->>B: collect(expect=n)
        B->>C: drain(expect=n)
        C-->>B: [CanFrame]
        B-->>R: 응답 없는 id
    else OBSERVE 모드
        L->>R: refresh()
        Note over R,B: 힘이 나가지 않는 명령을 보내고 응답을 받음
    end

    L->>T: record(loop_dt_ms)
    Note over T: Robot 계약만 읽음. 통신하지 않음
    L->>L: 다음 주기까지 기다림
```

**관찰 모드도 통신을 함.** MIT 프로토콜에는 읽기 전용 명령이 없어서, 아무것도
보내지 않으면 아무것도 오지 않음.

---

## 4. `build_commands()` 내부 — 관절에서 프레임까지

```mermaid
graph TD
    A["action  {knee: 30.0, ankle_pitch: 5.0, ankle_roll: 2.0}"] --> B{관절 이름 확인}
    B -->|모르는 이름| ERR[ValueError]
    B --> C{발목 둘 다 있나}
    C -->|하나만| ERR2[ValueError]
    C -->|둘 다| IK[kinematics.solve_ik]
    IK -->|안 풀림| DROP[발목 통째로 버림]
    IK -->|풀림| T["모터별 cal 목표  {knee: 30.0, ankle_a1: ..., ankle_a2: ...}"]
    C -->|발목 없음| T

    T --> G[safety.guards.apply]
    G -->|NaN| RJ[거부]
    G -->|현재 위치 모름| RJ
    G --> CL["한계 클리핑  cal 공간"]
    CL --> JP[점프 클리핑]
    JP --> R2R[cal_to_raw]
    R2R --> MC[MitCommand]

    G -.기록.-> CNT[counters]
    JP -.기록.-> CNT

    style G fill:#fff0e0
    style R2R fill:#e0ffe0
```

**검사가 변환보다 먼저임.** 한계가 cal 공간에 있으므로 raw 로 내린 뒤 검사하면
`sign` 이 -1 인 관절에서 부호가 뒤집혀 한계가 반대로 걸림.

**발목의 두 실패가 다름.** IK 가 안 풀리면 통째로 버리고, 한계에 잘리는 것은
개별로 처리함 — 잘린 각도 쌍도 대응하는 발 자세가 있음.

---

## 5. 브링업 메뉴 경로

```mermaid
graph LR
    U[사람] --> M[메뉴 항목]
    M --> MO[Motion 을 만듦]
    MO --> RUN["_run(loop, motion, 초)"]
    RUN --> LP[ControlLoop.run]
    LP --> LEG[Leg]
    LP --> TEL[Telemetry]
    LEG --> BUS[bus]

    style RUN fill:#e0ffe0
```

**메뉴가 로봇을 직접 부르지 않음.** 동작만 정하고 루프에 넘김.

직접 부르면 그 경로에서만 텔레메트리·주기 측정·정지 순서가 빠짐. 그러면 그래프가
안 나오는데 텔레메트리가 고장난 줄 알게 됨 ([이슈 #4](issues.md)).

테스트가 `ControlLoop.run` 을 감시해 이것을 고정함.

### 커미셔닝은 루프를 타지 않음

```mermaid
graph LR
    U[사람] --> C[commission 명령]
    C --> CM[commissioning 함수]
    CM --> BUS[bus]

    style C fill:#ffe0e0
```

영점·CAN id·프로토콜은 **반복하지 않는 조작이라 주기가 없음.** 되돌리기 어려워서
`--yes` 를 요구하고, 승인 확인이 버스를 열기 전에 일어남.

---

## 6. 설정과 실측값

```mermaid
graph TD
    Y[config/robot.yaml] -->|사람이 적음| L[load_robot]
    J[config/calibration/*.json] -->|기계가 잼| CL[calibration.load]

    L --> RC[RobotConfig]
    RC --> S[SafetyConfig]
    RC --> T[TelemetryConfig]
    RC --> LC[LimbConfig]
    LC --> MO["Motor  id, model, gains"]

    CL --> MC["MotorCalibration  sign, offset, zero_reference"]

    MO --> LEG[Leg]
    MC --> LEG
    S --> LEG
    T --> TEL[Telemetry]

    LEG -.->|쓰기 없음| X1[ ]
    CM[commissioning zero] -->|쓰기| J

    style Y fill:#e0f0ff
    style J fill:#fff0e0
```

**제어 경로는 읽기만 함.** 쓰기는 커미셔닝에서만 일어남 — 제어 중에 실측값이
바뀌면 좌표계가 도중에 옮겨감.

두 파일을 나눈 이유: `robot.yaml` 은 주석이 많아 **프로그램이 다시 쓰면 주석이
날아감.** 그래서 프로그램은 JSON 만 씀. 사람이 손으로 적는 값(id, model, 게인)만
`robot.yaml` 에 있음.

---

## 7. 텔레메트리 경로

```mermaid
graph TD
    LP[ControlLoop] -->|record| TM[Telemetry]
    TM --> BF[build_fast]
    TM --> BD[build_diag]

    BF --> R1[robot.get_observation]
    BF --> R2[robot.last_sent]
    BD --> R3[robot.link_status]
    BD --> R4[robot.counters]
    BD --> R5[bus.counters]

    BF -->|매 주기| U1[UDP 850B]
    BD -->|10주기마다| U2[UDP 950B]
    BF --> CSV[CSV 한 줄]
    BD --> CSV

    style U1 fill:#e0ffe0
    style U2 fill:#fff0e0
```

**필드 이름을 정하는 곳이 하나임.** UDP 와 CSV 가 같은 사전을 소비함.

**패킷을 둘로 나눔.** 합치면 66필드 약 1.8 KB 로 MTU(1500)를 넘어 조각나고, 조각
하나만 잃어도 패킷 전체가 버려짐.

CSV 는 안 나눔 — 크기 제약이 없고 한 줄에 다 있어야 대조하기 쉬움.

### 양다리

```
Telemetry(left_leg)   -> 패킷 둘
Telemetry(right_leg)  -> 패킷 둘
```

합쳐 보내지 않음. `merge()` 는 CSV 전용임.

---

## 8. CAN 한 왕복

```mermaid
sequenceDiagram
    participant L as Leg
    participant B as RobStrideBus
    participant D as codec/mit
    participant C as CanBus
    participant M as 모터

    L->>B: send_mit({10: MitCommand})
    B->>D: pack_command(pos, vel, kp, kd, tau, enc)
    Note over D: deg -> rad -> 양자화 -> 8바이트
    D-->>B: bytes
    B->>C: send_many([CanFrame])
    C->>M: 11-bit 표준 프레임

    Note over M: tau = kp*(목표-현재) + kd*(0-속도) + tau_ff

    M-->>C: 상태 프레임
    C->>C: drain(expect=n)
    C-->>B: [CanFrame]
    B->>D: decode_state(data, enc)
    Note over D: 8바이트 -> 역양자화 -> rad -> deg
    D-->>B: (id, pos, vel, tau, temp)
    B->>B: 캐시 갱신
    B-->>L: 응답 없는 id
```

**응답이 곧 ack 임.** MIT 모드는 명령을 받으면 반드시 답하므로, 안 오면 그 모터가
명령을 처리하지 않은 것임.

CAN 하드웨어 ACK 는 버스의 아무 노드나 찍어주므로 "누군가 들었다" 일 뿐임 —
그건 `tx_errors` 가 봄.

### 프레임 배치

```
명령 (Command 3)                    응답 (Response Command 1)
Byte0~1              목표각 16bit   Byte0                모터 CAN ID
Byte2 + Byte3[7:4]   목표속도 12bit  Byte1~2              현재각 16bit
Byte3[3:0] + Byte4   Kp     12bit   Byte3 + Byte4[7:4]   현재속도 12bit
Byte5 + Byte6[7:4]   Kd     12bit   Byte4[3:0] + Byte5   현재토크 12bit
Byte6[3:0] + Byte7   목표토크 12bit  Byte6~7              온도
```

**배치가 다름** — 응답은 앞에 모터 ID 가 붙어 한 칸씩 밀림.

---

## 9. 한눈에

| 계층 | 무엇을 하나 | 무엇을 모르나 |
|---|---|---|
| `scripts/` | 사람의 입력을 동작으로 | 프레임, 바이트 |
| `control/` | 주기, 모드, 시작·종료 순서 | 관절 이름, 모터 |
| `robots/` | 관절 ↔ 모터, cal ↔ raw, 안전 | 바이트 |
| `kinematics/` | 발목 pitch/roll ↔ a1/a2 | 모터, CAN |
| `safety/` | 한계·점프·NaN 검사 | 좌표계가 무엇인지 |
| `config/` | 설정 읽기 | 모터, CAN |
| `calibration/` | 실측값 읽기·쓰기 | 모터, CAN |
| `motors/base` | 벤더 중립 자료형 | 프레임 배치 |
| `robstride/` | 벤더 사양, 코덱, 버스 | 관절 이름 |
| `canbus/` | 8바이트 송수신 | 바이트의 뜻 |
| `telemetry/` | 관찰 | 제어에 끼어들지 않음 |
