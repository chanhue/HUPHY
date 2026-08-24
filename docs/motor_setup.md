# RobStride 모터 초기 설정 — 출하 상태에서 쓸 수 있는 상태까지

**갓 뜯은 모터를 이 저장소가 다룰 수 있게 만드는 절차임.**

루트 README 는 이게 끝난 상태를 전제함 — 거기서부터는 `huphy-commission` 으로 관절
이름을 부르면 됨. 여기는 그 이전임.

    출하 상태            private 프로토콜, CAN id 0x7F, 1 Mbps
    이 저장소가 쓰는 것  MIT 프로토콜, CAN id 7~12(오른다리), zero_sta = 1

**이 단계에서는 `huphy-commission` 을 못 씀.** 그 도구는 `robot.yaml` 에 적힌 id 로만
프레임을 보내는데, 출하 상태 모터는 그 id 를 안 쓰고 프레임 형식도 다름. 그래서
여기서는 `cansend` 로 직접 보냄.

출처는 각 모델의 RobStride 사용자 매뉴얼임. RS02·RS03·RS04 의 명령 표가 글자까지
같음.

---

## 1. 전제

```bash
sudo apt install -y can-utils
sudo ip link set can1 up type can bitrate 1000000
ip -details link show can1                       # state UP 이 보여야 함
```

**모터를 한 대만 물릴 것.** 출하 상태는 전부 같은 id 라, 여러 대를 물리면 명령이
양쪽에 다 가고 응답도 같이 나와서 어느 쪽인지 구분이 안 됨. 그 상태가 되면 선을
뽑는 것 말고 되돌릴 방법이 없음.

아래 예시는 **오른다리 무릎(id 10)** 을 만드는 것임. 호스트 id 는 `0xFD` 를 씀.

---

## 2. 순서

```
1  지금 id 찾기          Type 0
2  id 를 10 으로          Type 7        즉시 적용
3  zero_sta 를 1 로       Type 18 + 22  저장 필요
4  프로토콜을 MIT 로       Type 25       전원 재투입
5  전원 재투입
6  확인                   MIT 표준 프레임에 답하는지
7  선을 빼고 다음 모터
```

**id 를 먼저 바꿈.** 프로토콜을 먼저 바꾸면 재투입 후 MIT 인데 id 가 아직 0x7F 라,
그 상태를 다룰 수 있는 도구가 없어서 다시 손으로 프레임을 만들어야 함. id 를 먼저
주면 재투입하는 순간 `huphy-commission` 이 바로 붙음.

---

## 3. 지금 id 찾기 — Type 0

```bash
candump can1 &                                   # 다른 터미널이어도 됨
cansend can1 0000FD7F#0000000000000000
```

```
29-bit ID
  bit28~24   0x00          통신 타입 0 (Get device ID)
  bit15~8    0xFD          호스트 id
  bit7~0     0x7F          대상 모터 id
데이터       0
```

응답이 오면 그 id 가 맞음. 데이터에 64-bit MCU 고유번호가 실려 오므로 **같은 id 를
쓰는 모터가 둘이면 응답이 두 줄 나옴** — 그때는 한 대만 남기고 다시 함.

```
can1  0007FDFE  [8]  41 54 90 07 E8 0C 08 05     <- 응답
```

응답이 없으면 id 가 0x7F 가 아닌 것임. 0부터 127까지 훑음.

```bash
for i in $(seq 0 127); do
  printf -v id "0000FD%02X" $i
  cansend can1 "$id#0000000000000000"
  sleep 0.02
done
```

`candump` 에 뜬 응답의 `bit23~8` 이 그 모터의 id 임.

---

## 4. id 를 바꿈 — Type 7

무릎을 10(`0x0A`)으로:

```bash
cansend can1 070AFD7F#0000000000000000
```

```
29-bit ID
  bit28~24   0x07          통신 타입 7 (Set motor CAN_ID)
  bit23~16   0x0A          새 CAN id      <- 여기 들어감
  bit15~8    0xFD          호스트 id
  bit7~0     0x7F          지금 id
데이터       비움
```

**즉시 적용됨.** 재투입이 필요 없음. 바로 확인:

```bash
cansend can1 0000FD0A#0000000000000000        # 새 id 로 Type 0
```

응답이 오면 됨. 오른다리는 이렇게 배정함.

| 관절 | id | 두 번째 바이트 |
|---|---|---|
| hip_pitch | 7 | `0707FD7F` |
| hip_roll | 8 | `0708FD7F` |
| hip_yaw | 9 | `0709FD7F` |
| knee | 10 | `070AFD7F` |
| ankle_a | 11 | `070BFD7F` |
| ankle_b | 12 | `070CFD7F` |

왼다리는 1~6 이므로 `0701FD7F` ~ `0706FD7F`.

---

## 5. `zero_sta` 를 1 로 — Type 18 + Type 22

기본값이 0 이면 위치를 `[0, 2π)` 로 보고함. 이 저장소는 `[-π, π)` 를 전제하므로
1 로 바꿔야 함 — 안 바꾸면 음수 각도가 340도 같은 값으로 나옴.

```bash
cansend can1 1200FD0A#2970000001000000        # 쓰기
cansend can1 1600FD0A#0102030405060708        # 저장
```

```
쓰기 (Type 18)
  bit28~24   0x12
  bit15~8    0xFD          호스트 id
  bit7~0     0x0A          대상 모터 id
  Byte0~1    29 70         파라미터 0x7029. **하위 바이트가 앞임**
  Byte2~3    00 00
  Byte4~7    01 00 00 00   값 1

저장 (Type 22)
  bit28~24   0x16
  데이터      01 02 03 04 05 06 07 08   고정값
```

**Type 18 은 전원이 나가면 사라짐.** Type 22 를 안 보내면 재투입 후 0 으로 돌아감.

읽어서 확인 (Type 17):

```bash
cansend can1 1100FD0A#2970000000000000
```

응답 `Byte4` 가 `01` 이면 됨.

---

## 6. 프로토콜을 MIT 로 — Type 25

```bash
cansend can1 1900FD0A#0102030405060200
```

```
29-bit ID
  bit28~24   0x19          통신 타입 25
  bit15~8    0xFD          호스트 id
  bit7~0     0x0A          대상 모터 id
데이터        01 02 03 04 05 06 <F_CMD> 00
                                Byte6 = 0 private / 1 CANopen / 2 MIT
```

**전원을 재투입해야 적용됨.** 재투입 전까지는 확장 프레임으로 계속 통신해야 함.

---

## 7. 전원 재투입 후 확인

MIT 는 11-bit **표준** 프레임임. 정지 명령(Command 2)을 보내 답하는지 봄.

```bash
cansend can1 00A#FFFFFFFFFFFFFFFD
```

```
11-bit ID   0x0A          대상 모터 id
데이터       FF FF FF FF FF FF FF FD
                                  FD = 정지 명령
```

`candump` 에 **3자리 id** 로 응답이 오면 MIT 임.

```
can1  00A   [8]  0A 85 53 80 08 5F 01 3A     <- 표준 프레임. MIT
can1  1200FD0A [8] ...                        <- 확장 프레임. 아직 private
```

`cansend` 는 id 를 3자리로 쓰면 표준, 8자리로 쓰면 확장 프레임으로 보냄. `candump`
출력도 같은 폭이라 **눈으로 갈림.**

응답이 아예 없으면 재투입을 안 했거나, id·배선 문제임. 3절의 Type 0 로 되짚음.

---

## 8. 여기까지 하면

```bash
huphy-commission --limb right_leg scan
```

설정에 적힌 id 로 응답하므로 이제 이 도구가 붙음. 나머지 모터도 3~7절을 반복함.

여섯 개가 다 끝나면 `config/robot.yaml` 이 가리키는 파일의 `id` 가 실물과 맞는지
확인함. 다르면 명령이 안 나가는데 증상이 "모터가 안 움직인다" 라서 배선인지 id 인지
구분되지 않음.

---

## 9. id 가 부여된 뒤의 조작

**MIT 로 바뀐 뒤에는** 이 저장소 도구를 씀. 프로토콜을 되돌리거나, 나중에 모터를
교체했을 때 쓰는 자리임.

```bash
huphy-commission --limb right_leg can-id knee --to 10 --yes
huphy-commission --limb right_leg protocol knee --to mit --yes
```

관절 이름 대신 CAN id 로도 부를 수 있음. 관절 이름을 빼면 목록을 보여주고 물어봄.

같은 조작의 MIT 프레임은 이 모양임.

```
CAN id 변경 (Command 7)
  11-bit ID   현재 모터 id
  데이터       FF FF FF FF FF FF <새 id> FA

프로토콜 전환 (Command 8)
  11-bit ID   현재 모터 id
  데이터       FF FF FF FF FF FF <F_CMD> FD
                                 0 private / 1 CANopen / 2 MIT
```

private 은 새 값이 **중재 id 안**에 들어가는데 MIT 는 **데이터 byte6** 에 들어감.
같은 조작인데 값이 놓이는 자리가 다름.

### 응답 형식이 다름

Command 7·8 의 응답은 **Response Command 2** 이고 일반 상태 프레임이 아님.

```
Response Command 1 (일반 상태)   Byte0 이 모터 id
Response Command 2 (MCU 식별)    11-bit 중재 id 가 모터 id, 데이터는 MCU 고유번호
```

모터 id 가 데이터가 아니라 중재 id 에 실림. 일반 상태 프레임처럼 `Byte0` 으로
판정하면 MCU 고유번호의 첫 바이트를 id 로 읽게 됨.

그래서 `set_can_id` 는 이 응답으로 판정하지 않고, **새 id 로 정지 명령을 보내 응답이
오는지**로 확인함.

`set_protocol` 은 확인을 못 함 — 현재 프로토콜은 파라미터 `0x201F` 에 있는데 그
읽기가 확장 프레임을 필요로 함. 7절의 방법으로 사람이 확인해야 함.

---

## 10. 모델별 인코딩 범위

CAN 프레임에는 N·m 이 아니라 범위 안의 눈금만 실림. 설정의 `model` 이 실물과 다르면
그 비율만큼 토크가 어긋나는데 프레임도 응답도 정상임.

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

| 모델 | 속도 | Kp | Kd | 토크 |
|---|---|---|---|---|
| RS02 private | ±44 rad/s | 0~500 | 0~5 | ±17 N·m |
| RS03 private | ±20 rad/s | 0~5000 | 0~100 | ±60 N·m |
| RS04 private | ±15 rad/s | 0~5000 | 0~100 | ±120 N·m |

**RS03·RS04 는 Kp·Kd 범위가 MIT 의 10배와 20배임.** private 값을 MIT 프레임에 넣으면
게인이 1/10 로 들어감.

[`tables.py`](../src/huphy/motors/robstride/tables.py) 가 인코딩 범위를 "모델별" 이
아니라 **"프로토콜 × 모델"** 로 잡아 둔 이유가 이것임.

---

## 11. 명령 한눈에

무릎(id 10) 기준. `0A` 자리를 바꾸면 다른 모터임.

```bash
# 채널
sudo ip link set can1 up type can bitrate 1000000
candump can1 &

# 출하 상태 (private, 확장 프레임)
cansend can1 0000FD7F#0000000000000000     # 지금 id 확인
cansend can1 070AFD7F#0000000000000000     # id -> 10           즉시
cansend can1 1200FD0A#2970000001000000     # zero_sta -> 1
cansend can1 1600FD0A#0102030405060708     # 저장
cansend can1 1900FD0A#0102030405060200     # 프로토콜 -> MIT     재투입 필요

# 전원 재투입

# MIT (표준 프레임)
cansend can1 00A#FFFFFFFFFFFFFFFD          # 정지. 응답 오면 MIT
huphy-commission --limb right_leg scan     # 여기서부터 도구가 붙음
```
