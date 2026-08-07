# `calibration/` — 실측값 읽기·쓰기

```
calibration/
└── store.py   config/calibration/*.json <-> dict[관절이름, MotorCalibration]
```

값 자체는 여기 없음. 저장소 루트의
[`config/calibration/`](../../../config/calibration/) 에 있음.

---

## 무엇을 담나

**조립을 재서 얻는 값**만 담음.

```
sign             모터 회전 방향이 관절 양의 방향과 반대면 -1. 설계가 정함
offset_deg       기계 영점에서 관절 0도까지의 차이. commission sweep 이 잼
zero_reference   영점을 어느 자세에서 잡았는지 사람이 남기는 메모
```

```
cal = sign * raw + offset
```

한계와 게인은 **적는 값**이라 `robot.yaml` 에 있음 (이슈 #2). 무효화 시점이 다름 —
모터를 다시 달면 여기 값만 무효가 되고, 기구 설계가 바뀌면 `robot.yaml` 만 바뀜.

---

## 자료형은 여기 없음

```
motors/base.py         MotorCalibration 이 무엇인지     <- 모양
calibration/store.py   그걸 파일에서 읽고 쓰는 법        <- 입출력
```

`base.py` 가 파일 형식을 몰라야 함. 형식을 바꿔도 그쪽은 안 건드림.

`config/` 의 `schema.py` / `loader.py` 와 같은 나눔임.

---

## 관절 이름으로 키를 맞춤

```json
"knee": {"sign": 1.0, "offset_deg": 0.0, "zero_reference": ""}
```

CAN id 는 바뀔 수 있음 (`commissioning.set_can_id`). **관절 자리는 안 바뀜.**

`robot.yaml` 도 관절 이름을 키로 쓰므로 두 파일을 나란히 놓고 대조할 수 있음.

`motor_id` 는 저장하지 않음. `robot.yaml` 이 가진 값이고, 두 군데 있으면 어긋날 수
있음. 읽을 때는 `-1` 로 남고 `attach()` 가 채움.

---

## 함수

| | |
|---|---|
| `load(path)` | 파일 -> 관절 이름 키의 사전 |
| `save(path, ...)` | 사전 -> 파일. 덮어씀 |
| `attach(cal, motors)` | `robot.yaml` 의 모터 목록과 맞춰 모터 id 로 다시 키를 잡음 |
| `identity(joints)` | 전부 항등변환. 실측 전 상태 |
| `unmeasured(cal)` | 아직 실측되지 않은 관절 이름들 |

### `attach` 가 경계임

```python
robot = load_robot("config/robot.yaml")
leg = robot.limb("right_leg")

cal_by_id = attach(load(leg.calibration_path), leg.motors)
# {10: MotorCalibration(motor_id=10, sign=1.0, ...)}
```

버스 계층은 관절 이름을 모름. 여기서 이름을 버리고 모터 id 로 넘김.

**양쪽 관절 이름이 정확히 같아야 함.** 한쪽에만 있으면 에러임 — 관절 하나가 조용히
항등변환으로 도는 것이 가장 나쁨. `sign` 이 반대인 관절이 그렇게 되면 목표에서
**멀어지는 방향**으로 토크가 걸림.

### `unmeasured` 가 메모로 판정함

`sign`/`offset` 으로 판정하지 않음. **실측 결과가 우연히 `1.0`/`0.0` 일 수 있음.**

메모는 사람이 적는 것이라 우연히 채워지지 않음.

---

## 저장이 원본을 지킴

```
임시 파일에 씀  ->  fsync  ->  os.replace 로 바꿔치기
```

도중에 죽으면 원본이 그대로 남음. **실측값을 잃으면 다시 재는 수밖에 없는데, 그건
로봇을 분해해야 하는 작업일 수 있음.**

임시 파일도 남기지 않음.

---

## 제어 경로는 읽기만 함

쓰기는 캘리브레이션 절차에서만 일어남. 제어 중에 실측값이 바뀌면 좌표계가 도중에
옮겨감 — 같은 목표각이 갑자기 다른 물리적 위치를 가리키게 됨.

---

## 파일 형식 번호

```json
{"schema_version": 1, ...}
```

형식이 바뀌면 올림. 읽을 때 대조해서, 코드가 기대하는 것과 다른 파일을 조용히
읽어 들이지 않게 함 — 항목 하나가 무시되면 그 관절만 항등변환으로 돎.

---

## 거부 조건

| | 왜 |
|---|---|
| 모르는 키 | 한계·게인이 여기 남아 있으면 값이 두 군데가 됨 |
| `sign = 0` | 모든 raw 가 같은 cal 로 뭉개져 역변환이 불가능함 |
| 형식 번호 불일치 | 항목이 조용히 무시됨 |
| 관절 이름 불일치 (`attach`) | 그 관절만 항등변환으로 돎 |

빠진 항목은 항등변환으로 봄. 파일을 새로 만들 때 편함.

---

## 쓰는 법

```python
from huphy import calibration as cal
from huphy.config import load_robot

leg = load_robot("config/robot.yaml").limb("right_leg")

c = cal.load(leg.calibration_path)
cal.unmeasured(c)              # ('hipz', 'hipx', ...) -- 지금은 전부
by_id = cal.attach(c, leg.motors)
by_id[10].raw_to_cal(45.0)     # 45.0 (아직 항등)

c["knee"] = MotorCalibration(motor_id=-1, sign=-1.0, offset_deg=12.0,
                             zero_reference="다리 편 상태, 발바닥 평면 접촉")
cal.save(leg.calibration_path, c, limb="right_leg")
```

---

## 현재 상태

전부 미실측임. `sign=1, offset=0` 이라 **`cal` 과 `raw` 가 같은 숫자**이고, 어느
쪽으로 해석해도 동작이 같아서 두 공간을 섞어 써도 드러나지 않음 (이슈 #2).

`zero_reference` 가 전부 비어 있음 (이슈 #9).

---

## 테스트

```bash
PYTHONPATH=src python3 -m pytest tests/test_calibration.py -q
```

35개. 실제 `config/calibration/*.json` 두 개를 읽고 `robot.yaml` 과 관절 이름이
맞는지 확인함.

저장 중 죽어도 원본이 남는지는 `os.replace` 를 실패시켜 확인함.
