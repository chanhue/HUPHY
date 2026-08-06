# `robstride/` — RobStride 드라이버

```
robstride/
├── tables.py   벤더 사양 (데이터시트에서 오는 값)
└── codec/
    └── mit.py  MIT 표준 프레임 인코딩/디코딩
```

둘 다 순수 계산임. `python-can` 없이 import되고 테스트됨.

출처: RS02 User Manual (Seeed Studio 배포판 251112). 각 값의 페이지를 주석에 명시함.

---

## `tables.py`

실측이 아님. 모터를 바꾸지 않는 한 변하지 않음.
실측값(sign/offset/limits/gains)은 `calibration/` 으로 감.

### 프로토콜과 제어 모드는 다른 축임

이름이 겹쳐 헷갈리기 쉬움.

| | `Protocol` | `ControlMode` |
|---|---|---|
| 정하는 것 | **프레임 포맷** | **무엇을 명령할지** |
| 값 | 11-bit 표준 / 29-bit 확장 | 5개 파라미터 / 목표위치 / 목표속도 |
| 전환 | Command 8 → **전원 재투입** | Command 6 → 즉시 |

**서로 독립임.** MIT 프로토콜을 쓰면서 Position 모드를 쓸 수도 있음.

> `Protocol.MIT`은 "11-bit 표준 프레임을 쓴다",
> `ControlMode.MIT`은 "5개 파라미터를 보낸다" — 같은 이름이지만 뜻이 다름.
> 매뉴얼도 후자를 "MIT mode", "Motion Control Mode", "operation control mode"로
> 혼용함.

모드마다 보내는 명령이 다름.

| 모드 | 명령 | 보내는 것 |
|---|---|---|
| `MIT` (전원 투입 기본값) | Command 3 | 위치 · 속도 · kp · kd · 토크 **5개** |
| `POSITION` | Command 10 | 목표 위치 + 속도 제한 |
| `VELOCITY` | Command 11 | 목표 속도 + 전류 제한 |

**본 프로젝트는 MIT 모드를 사용함.** 궤적을 직접 만들고 게인을 매 프레임 실어
보내야 하기 때문임.

### 인코딩 범위는 `[프로토콜][모델]`

같은 RS02라도 프로토콜에 따라 속도 범위가 다름.

| 조합 | 위치 | 속도 | 토크 | 출처 |
|---|---|---|---|---|
| RS02 / **MIT** | ±12.57 rad | **±33 rad/s** | ±17 N·m | p.37~38 |
| RS02 / private | ±12.57 rad | **±44 rad/s** | ±17 N·m | p.20~21 |
| RS00 / MIT | ±12.57 rad | ±33 rad/s | ±14 N·m | p.26 (미확인) |

무부하 410rpm ≈ 43 rad/s이므로 private은 전 범위를, MIT은 12bit라 ±33으로 잘라
쓰는 것으로 보임.

**현재 MIT만 사용하지만 `PRIVATE_ENCODING`도 표에 남김.** 두 범위가 다르다는 사실
자체가 중요하며, 이 축이 없으면 private 값을 MIT에 가져다 쓰는 실수가 남.

```python
encoding_for(Model.RS02)                      # 기본 인자가 MIT
encoding_for(Model.RS02, Protocol.PRIVATE)
encoding_for(Model.RS00, Protocol.PRIVATE)    # KeyError
```

**없는 조합에 `KeyError`를 던짐.** 조용히 기본값으로 때우지 않음 — 범위가 틀리면
명령한 값과 실제가 배율만큼 어긋나는데, 실물에서 찾기 매우 어려움.

> `pmax = 12.57 rad`은 4π를 반올림한 값임 (= ±720°).

### 명령 바이트의 F_CMD 규약

```
data[0:6] = 0xFF    data[6] = F_CMD    data[7] = 명령 바이트
```

`F_CMD`가 `0xFF`면 기본 동작, 다른 값이면 변형 동작임.

| `data[7]` | `F_CMD = 0xFF` | `F_CMD = 그 외` |
|---|---|---|
| `0xFC` | Enable | 제어 모드 설정 (값 = 모드) |
| `0xFD` | Stop | 프로토콜 전환 (값 = 프로토콜) |
| `0xFB` | 고장 클리어 | 고장값 조회 |

**같은 바이트가 두 명령을 겸함.** 그래서 상수 이름을 나눠 뒀음.

```python
CMD_ENABLE = 0xFC        # F_CMD = 0xFF
CMD_SET_MODE = 0xFC      # F_CMD = ControlMode
```

> 주석의 "Command N"은 **매뉴얼 목차 번호**임. CAN에는 이 숫자가 들어가지 않음.
> 나중에 "이 `0xFC`가 뭐지?" 할 때 매뉴얼에서 바로 찾으라는 표시임.
> private 프로토콜은 같은 것을 "Communication Type N"이라 부름 — 번호 체계가 다름.

### 파라미터 인덱스

하드웨어 전제를 확인하는 데 쓰는 것만 둠. 나머지는 필요해질 때 매뉴얼에서 추가함.

| 상수 | 인덱스 | 용도 |
|---|---|---|
| `PARAM_PROTOCOL_FLAG` | `0x201F` | 현재 프로토콜 확인 |
| `PARAM_ZERO_STA` | `0x7029` | 위치 보고 범위 확인 (`0`=`[0,360)`, `1`=`[-180,180)`) |

---

## `codec/mit.py`

로봇을 전혀 모름. 관절 이름도 모터 배치도 없고 숫자와 바이트만 다룸.
인코딩 범위를 **인자로 받으므로** 모델·프로토콜에 묶이지 않음.

### 왜 양자화하나

CAN 2.0 프레임은 데이터가 **8바이트뿐**임. 실수 5개에 부동소수(4바이트씩 20바이트)를
쓸 수 없음.

```
uint = (값 + max) / (2*max) * (2**bits - 1)
해상도 = 2*max / (2**bits - 1)
```

위치에 16비트를 몰아주고 나머지를 12비트로 깎아 8바이트에 맞춤.

```
16 + 12 + 12 + 12 + 12 = 64bit = 8byte    ← 정확히 맞음
```

| 값 | 비트 | 해상도 (RS02) |
|---|---|---|
| 위치 | 16 | **0.0220°** |
| 속도 | 12 | 0.016 rad/s |
| 토크 | 12 | **0.0083 N·m** |
| Kp | 12 | 0.12 |
| Kd | 12 | 0.0012 |

### 프레임 배치

**명령** — Command 3 (p.38). 11-bit 표준 ID = 대상 모터 CAN ID

```
Byte0~1               목표각    16bit
Byte2 + Byte3[7:4]    목표속도  12bit
Byte3[3:0] + Byte4    Kp        12bit
Byte5 + Byte6[7:4]    Kd        12bit
Byte6[3:0] + Byte7    목표토크  12bit
```

**응답** — Response Command 1 (p.37)

```
Byte0                 모터 CAN ID
Byte1~2               현재각    16bit
Byte3 + Byte4[7:4]    현재속도  12bit
Byte4[3:0] + Byte5    현재토크  12bit
Byte6~7               권선 온도 (0.1° 단위)
```

**명령과 응답의 배치가 다름** — 응답은 앞에 모터 ID가 붙어 한 칸씩 밀림.

### 함수

| 함수 | 반환 |
|---|---|
| `float_to_uint(x, min, max, bits)` | 양자화된 정수 |
| `uint_to_float(x, min, max, bits)` | 역변환 |
| `pack_command(...)` | 명령 8바이트 |
| `decode_state(data, enc)` | `(id, pos_deg, vel_deg_s, tau_nm, temp_c)` |
| `decode_fault(data)` | `(id, fault_word)`. `0`이면 정상 |

### 클램프 특성 ⚠️

`float_to_uint`는 범위를 벗어나면 **잘림** (감싸지 않음).

```python
float_to_uint(999.0, -10.0, 10.0, 12)   # 4095 = 최대값
```

따라서 **전송 전에 범위 확인이 필요함** — 넘으면 조용히 최대/최소값이 나감.

그리고 **NaN은 이 클램프를 통과함.**

```python
min(10, nan)                             # 10      ← 비교가 False라 통과
float_to_uint(nan, -12.57, 12.57, 16)    # 65535 = 720°
```

**NaN 하나가 720° 목표 명령이 됨.** `safety.guards`가 미리 걸러야 함.

### 단위

내부는 rad, 외부는 deg. 변환이 **이 파일에서만** 일어나도록 경계에 가둠.
나머지 코드는 라디안을 신경 쓰지 않아도 됨.

### `decode_fault`의 한계

고장 응답은 일반 상태 프레임과 **CAN ID가 같아** 겉으로 구분되지 않음.
조회 명령을 보낸 직후의 첫 응답으로 간주해야 함.

---

## 미구현

| 파일 | 용도 | 필요해지는 시점 |
|---|---|---|
| `bus.py` | 런타임 통신 | 3단계 |
| `commissioning.py` | 영점·CAN ID·프로토콜 전환 | 4단계 |
| `codec/private.py` | 29-bit 확장 프레임 | 프로토콜이 private으로 확인되면 |
