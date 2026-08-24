# RobStride 모터 초기 설정 — CAN id 와 통신 프로토콜

조립할 때 한 번 하는 조작임. 둘 다 **모터에 저장되고 전원을 껐다 켜도 남음.**

    CAN id      한 버스에 여러 모터를 물리려면 서로 달라야 함. 출하 상태는 전부 같음
    프로토콜    이 저장소는 MIT 를 씀. 출하 기본값은 private 임

출처는 각 모델의 RobStride 사용자 매뉴얼임.

    RS02   RS02 User Manual 251112 (Seeed Studio 배포판)
    RS03   RobStride Motor User Manual (RobStride Dynamics)
    RS04   RobStride Motor User Manual (RobStride Dynamics)

세 매뉴얼의 명령 표가 글자까지 같음.

구현은 [`motors/robstride/commissioning.py`](../src/huphy/motors/robstride/commissioning.py)
이고, 터미널에서는 `huphy-commission can-id` / `protocol` 로 부름.

---

## 1. 순서

프로토콜을 **먼저** 바꿈. private 상태에서는 이 저장소의 MIT 프레임이 안 먹으므로
id 를 바꿀 방법도 없음.

```
1  모터 한 대만 버스에 물림
2  프로토콜을 MIT 로                    전원 재투입 필요
3  CAN id 를 바꿈                       즉시 적용
4  선을 빼고 다음 모터를 물림
5  반복
6  config/robot.yaml 의 id 를 고침
```

### 왜 한 대씩 하나

**같은 id 가 둘이 되면 되돌릴 방법이 없음.**

두 모터가 같은 id 를 쓰면 명령이 양쪽에 다 가고 응답도 둘이 동시에 나옴. 어느 쪽이
답한 것인지 구분이 안 되므로 "id 를 바꿀 대상" 을 지목하는 것 자체가 안 됨. 선을
뽑아 한 대만 남기고 다시 해야 함.

### 6번을 빼먹으면 안 됨

설정 파일과 실물이 어긋나면 명령이 안 나가는데, 증상이 "모터가 안 움직인다" 라서
원인이 배선인지 id 인지 구분되지 않음.

---

## 2. 통신 프로토콜

세 가지가 있고 프레임 형식 자체가 다름.

| | 프레임 | 이 저장소 |
|---|---|---|
| private | 29-bit 확장 | 출하 기본값. 안 씀 |
| CANopen | CiA 402 | 안 씀 |
| **MIT** | **11-bit 표준** | **이것을 씀** |

**어긋나면 연결도 되고 에러도 없는데 모터만 안 움직임.** 프레임이 나가긴 하는데
모터가 자기 형식이 아니라 무시함. 증상이 배선·전원·id 를 먼저 의심하게 만들어서
프로토콜은 마지막에 떠오름.

### MIT 로 바꾸기 — Command 8

지금 private 이면 29-bit 확장 프레임으로 보내야 함. MIT 로 이미 바뀐 상태에서
다시 확인하거나 되돌릴 때는 11-bit 표준 프레임을 씀.

```
11-bit ID   현재 모터 CAN id
데이터      FF FF FF FF FF FF <F_CMD> FD
            └── byte0~5 ──┘   byte6   byte7

  byte7   FD   명령 코드 (프로토콜 전환)
  byte6        F_CMD = 0 private / 1 CANopen / 2 MIT
```

id 7 인 모터를 MIT 로:

```
can_id = 0x07
data   = FF FF FF FF FF FF 02 FD
```

응답은 Response Command 2 임 — 11-bit ID 에 모터 id, 데이터는 64-bit MCU 고유번호.

**전원을 재투입해야 적용됨.** 재투입 전까지는 옛 형식으로 계속 통신해야 함.

### private 쪽 — Type 25

출하 상태(private)에서 보낼 때 쓰는 형식임.

```
29-bit ID
  bit28~24   0x19          통신 타입 25
  bit15~8    호스트 id     보통 0xFD
  bit7~0     대상 모터 id

데이터       01 02 03 04 05 06 <F_CMD>
                              byte6 = 0 private / 1 CANopen / 2 MIT
```

응답은 Type 0 프레임임. **전원 재투입 후 적용됨.**

### 이 저장소의 구현

`set_protocol` 이 위의 MIT Command 8 을 보냄. **바뀌었는지는 코드로 확인 못 함** —
현재 프로토콜은 파라미터 `0x201F` 에 있는데 그 읽기가 private 확장 프레임을
필요로 함. 이 코드는 11-bit 표준 프레임만 보냄.

```bash
huphy-commission --limb right_leg protocol knee --to mit --yes
# 전원을 재투입해야 적용됨
```

출하 상태에서 처음 바꿀 때는 MotorStudio 같은 외부 도구를 쓰는 편이 확실함.

---

## 3. CAN id — MIT Command 7

이 저장소가 쓰는 경로임.

```
11-bit ID   현재 모터 CAN id
데이터      FF FF FF FF FF FF <F_CMD> FA
            └── byte0~5 ──┘   byte6   byte7

  byte7   FA   명령 코드 (CAN id 변경)
  byte6        F_CMD = 새로 줄 CAN id
```

id 7 인 모터를 10 으로:

```
can_id = 0x07
data   = FF FF FF FF FF FF 0A FA
```

**즉시 적용됨.** 전원 재투입이 필요 없음.

### 응답이 다른 종류임

Command 7 의 응답은 **Response Command 2** 이고, 일반 상태 프레임이 아님.

```
Response Command 1 (일반 상태)   Byte0 이 모터 id
Response Command 2 (MCU 식별)    11-bit 중재 id 가 모터 id, 데이터는 MCU 고유번호
```

모터 id 가 **데이터가 아니라 중재 id 에** 실림. 일반 상태 프레임처럼 `Byte0` 으로
판정하면 MCU 고유번호의 첫 바이트를 id 로 읽게 됨.

그래서 `set_can_id` 는 이 응답으로 판정하지 않고, **새 id 로 정지 명령(Command 2)을
보내 응답이 오는지**로 확인함. 정지 명령의 응답은 Response Command 1 이라 `Byte0` 에
모터 id 가 실림.

```bash
huphy-commission --limb right_leg can-id knee --to 10 --yes
```

바꾸기 전에 세 가지를 봄 — 1~127 범위인가, 현재 id 와 다른가, 이 버스의 다른 모터가
쓰고 있지 않은가.

---

## 4. CAN id — private Type 7

출하 상태에서 바꿀 때 쓰는 형식임.

```
29-bit ID
  bit28~24   0x07          통신 타입 7
  bit23~16   새 CAN id     preset CAN_ID
  bit15~8    호스트 id     보통 0xFD
  bit7~0     대상 모터 id

데이터       비움
```

MIT 는 새 id 가 **데이터 byte6** 에 들어가는데 private 은 **중재 id 안(bit23~16)** 에
들어감. 같은 조작인데 값이 놓이는 자리가 다름.

응답은 Type 0 브로드캐스트 프레임임. **즉시 적용됨.**

---

## 5. 지금 id 를 모를 때

**MIT** — 스캔 명령이 따로 없음. 1~127 로 정지 명령(Command 2)을 하나씩 보내고
응답이 오는 id 를 찾음.

```bash
huphy-commission --limb right_leg scan
```

**private** — Type 0 "Get device ID" 로 물어보면 id 와 64-bit MCU 고유번호를 답함.
고유번호가 같이 오므로 **같은 id 를 쓰는 모터가 둘인지도 드러남.** MIT 에는 이것에
해당하는 명령이 없음.

---

## 6. 모델별 인코딩 범위

id·프로토콜과는 별개지만 **모델을 섞어 쓸 때 같이 봐야 하는 값**임. CAN 프레임에는
N·m 이 아니라 범위 안의 눈금만 실리므로, 설정의 `model` 이 실물과 다르면 그 비율만큼
토크가 어긋나는데 프레임도 응답도 정상임.

각 매뉴얼의 Command 3 "MIT Dynamic Parameters" 표에서 옮김.

| 모델 | 각도 16bit | 속도 12bit | Kp 12bit | Kd 12bit | 토크 12bit |
|---|---|---|---|---|---|
| RS00 | ±12.57 rad | ±33 rad/s | 0~500 | 0~5 | ±14 N·m |
| RS02 | ±12.57 rad | ±33 rad/s | 0~500 | 0~5 | ±17 N·m |
| RS03 | ±12.57 rad | ±33 rad/s | 0~500 | 0~5 | ±60 N·m |
| RS04 | ±15 rad | ±33 rad/s | 0~500 | 0~5 | ±120 N·m |

**RS04 는 각도 범위까지 다름.** 나머지 셋은 토크만 갈림. 같은 정수 34132 가 RS02
에서는 29.99도, RS04 에서는 35.79도로 풀림.

RS02 설정으로 RS04 를 돌리면 시킨 토크의 **7배**가 나감 (120/17).

### private 은 범위가 또 다름

같은 모터라도 프로토콜에 따라 다름.

| 모델 | 속도 | Kp | Kd | 토크 |
|---|---|---|---|---|
| RS02 private | ±44 rad/s | 0~500 | 0~5 | ±17 N·m |
| RS03 private | ±20 rad/s | 0~5000 | 0~100 | ±60 N·m |
| RS04 private | ±15 rad/s | 0~5000 | 0~100 | ±120 N·m |

**RS03·RS04 는 Kp·Kd 범위가 MIT 의 10배와 20배임.** private 값을 MIT 프레임에 넣으면
게인이 1/10 로 들어가고, 반대면 10배로 들어감.

[`tables.py`](../src/huphy/motors/robstride/tables.py) 가 인코딩 범위를 "모델별" 이
아니라 **"프로토콜 × 모델"** 로 잡아 둔 이유가 이것임.
