# `sensors/` — IMU

```
sensors/
├── base.py         ImuState, Imu. 벤더를 모르는 자료형과 계약
├── registry.py     설정의 model 문자열 -> 구현체
└── xsens/
    ├── imu.py      XsensImu. 위쪽이 쓰는 것
    └── xbus/       받아 온 시리얼 읽기 코드
```

`motors/` 와 같은 나눔임. 중립 계층이 자료형과 계약을 정하고 벤더 폴더가 구현함.
센서를 갈아 끼워도 위쪽 코드는 바뀌지 않음.

---

## 받아오는 값

`Imu.read()` 가 `ImuState` 하나를 냄. 새로 통신하지 않고 **가장 최근 값만** 꺼냄.

| 필드 | 단위 | 무엇 |
|---|---|---|
| `roll_deg` `pitch_deg` `yaw_deg` | 도 | 자세 |
| `accel_mps2` | m/s² | 몸체 좌표계 가속도. **중력 포함** |
| `gyro_dps` | 도/초 | 각속도 |
| `temp_c` | ℃ | 센서 온도 |
| `stamp` | 초 | 이 값을 받은 시각. `time.monotonic()` 기준 |
| `is_valid` | | 한 번이라도 패킷을 받았는지 |

`age_ms()` 로 마지막 값 이후 경과를 냄. 한 번도 못 받았으면 `-1`.

못 받았어도 `ImuState` 객체는 나옴. `is_valid` 가 거짓이고 값은 전부 0 임 — 0도라는
뜻이 아니라 모른다는 뜻임.

각도와 각속도는 도임. 센서가 라디안으로 주면 벤더 모듈이 바꿔서 올림.

---

## 설정

```yaml
imus:
  main:
    model: xsens_mti
    port: /dev/xsens_mti
    baudrate: 921600
    mount: right_leg
```

| 키 | 기본값 | 무엇 |
|---|---|---|
| `model` | 필수 | 어느 센서인지. `registry.MODELS` 의 키 |
| `port` | 필수 | 시리얼 장치 경로 |
| `baudrate` | `921600` | 센서에 저장된 값과 같아야 함 |
| `mount` | 없음 | 어디 붙었는지. 팔다리 이름이거나 `torso` `pelvis` `head` |

`imus` 는 `limbs` 와 나란히 있음. 팔다리 안에 두지 않는 이유는 같은 센서가 다리에
붙었다가 몸통으로 옮겨가기 때문임 — 옮길 때 `mount` 한 줄만 바뀜.

`port` 는 udev 로 고정한 심볼릭 링크를 쓸 것. USB 는 꽂는 순서대로 `ttyUSB0`,
`ttyUSB1` 이 붙어 재부팅마다 달라짐.

```
# /etc/udev/rules.d/99-xsens.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ATTRS{serial}=="<시리얼>", SYMLINK+="xsens_mti"
```

한 포트를 두 IMU 가 쓰면 설정을 읽을 때 멈춤. `mount` 가 팔다리 이름인데 그런
팔다리가 없어도 멈춤.

---

## 갈아 끼우기

`registry.py` 의 표가 `model` 문자열을 구현체로 품.

```python
MODELS = {"xsens_mti": _xsens}
```

다른 센서를 붙이면 `sensors/<벤더>/` 를 만들고 표에 한 줄 더함. 벤더 모듈은
`make_imu` 안에서 import 함 — 안 쓰는 센서의 의존성까지 깔려 있어야 설정을 읽을 수
있으면 곤란함.

구현체가 갖출 것은 `Imu` 프로토콜임. 상속하지 않아도 됨.

```python
name: str
is_connected -> bool
connect()  disconnect()
read() -> ImuState
```

`name` 은 설정의 개체 이름임. **텔레메트리 필드 앞에 붙음.**

---

## 붙는 자리

```python
robot.imus_on("right_leg")     # 그 팔다리에 붙은 것만
Leg(limb, bus, ..., imus=[...])
```

`build_leg()` 가 `mount` 를 보고 골라서 넣음. `Leg.imus` 는 비어 있어도 됨.

`leg.connect()` 가 IMU 도 같이 엶. **여는 데 실패해도 다리는 그대로 씀** — IMU 는
관측이지 제어가 아님. 로그에 경고만 남고 `read()` 는 `is_valid=False` 를 냄.

`leg.imu_states()` 가 개체 이름 -> `ImuState` 를 냄.

---

## 텔레메트리

```
imu/main/roll    imu/main/ax    imu/main/gx    imu/main/age
imu/main/pitch   imu/main/ay    imu/main/gy
imu/main/yaw     imu/main/az    imu/main/gz
```

앞에 붙는 것이 **팔다리 이름이 아니라 IMU 개체 이름**임. 다리에서 몸통으로 옮겨도
필드 이름이 그대로라 예전 로그와 그래프 레이아웃이 맞음.

UDP 는 다리 패킷과 **따로** 나감. 다리 하나가 이미 MTU 에 가깝고, 붙는 자리도
다리와 무관함. IMU 가 없으면 이 패킷은 안 보냄.

CSV 는 한 줄에 다 들어감. 값이 없어도 키는 나가고 `age` 가 `-1` 임.

---

## 의존성

```
pyserial    시리얼 포트
numpy       패킷 해석
```

`connect()` 안에서 import 함. 센서를 안 쓰는 실행에서는 없어도 됨.

---

## `xsens/xbus/` 는 받아 온 코드임

출처: `maido-39/Huphychan-RIP_Sim2Real` 의 `utils_imu/`. 그쪽은 다시
`jiminghe/Xsens_MTi_Serial_Reader` 를 감싼 것임.

파일 이름과 내용을 바꾸지 않음 — 원본과 대조할 수 있어야 함. **고친 것은 import
하나뿐**임(평평한 import 를 상대 import 로).

```
SerialHandler.py      시리얼 포트
XbusPacket.py         프레임 조립·체크섬
DataPacketParser.py   패킷 해석
SetOutput.py          센서의 출력 항목 설정
imu_reader.py         위 셋을 감싼 리더. 백그라운드 스레드로 받음
example_usage.py      원본 사용 예
```

여기 것을 직접 쓰지 말 것. 위쪽은 `XsensImu` 를 씀 — 그것이 리더의 dict 를
`ImuState` 로 바꿔 올림.
