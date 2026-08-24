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
```

### 채널 올리기

**어댑터 종류에 따라 다름.** 속도를 어디서 정하느냐가 갈림길임.

```bash
# USB 어댑터 (CANable 등) -- 속도를 어댑터가 들고 있음
sudo slcand -o -c -s8 /dev/canable0 can1
sudo ip link set can1 up

# 네이티브 CAN (HAT, MCP2515) -- 속도를 커널이 들고 있음
sudo ip link set can1 up type can bitrate 1000000
```

`-s8` 이 **1 Mbps** 임. 모터 출하 기본값이 1 Mbps 라 이대로 맞음.

USB 쪽은 `can1` 이 **찾는 이름이 아니라 짓는 이름**임 -- `slcand` 마지막 인자가
만들어낼 장치 이름이고, `config/robot.yaml` 의 `channel` 과 같아야 함. 네이티브
쪽은 부팅할 때 드라이버가 이미 만들어 두므로 골라 쓸 수 없음.

```bash
ip -br link show type can        # 지금 뭐가 있나
ip -details link show can1       # state UP 이 보여야 함
```

`/dev/canable0` 은 udev 로 고정한 심볼릭 링크임. USB 는 꽂는 순서대로 `ttyACM0`,
`ttyACM1` 이 붙어 재부팅마다 달라짐. 자세한 것은 루트
[README](../README.md) 4번.

### 한 대씩

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
candump can1 &                                  # 응답을 계속 지켜봄. 다른 터미널이어도 됨
cansend can1 0000FD7F#0000000000000000          # Type 0  출하 기본 id(0x7F)에게 "너 누구냐"
```

```
29-bit ID
  bit28~24   0x00          통신 타입 0 (Get device ID)
  bit15~8    0xFD          호스트 id
  bit7~0     0x7F          대상 모터 id
데이터       0
```

```
can1  00007FFE  [8]  41 54 90 07 E8 0C 08 05     <- 응답
      ────┬───       ────────┬────────
          │                  └── 64-bit MCU 고유번호
          00 00 7F FE
           │  │  │  └── FE   Type 0 응답 표시 (고정)
           │  └──┴───── 7F   **모터 CAN id. 여기를 읽음**
           └─────────── 00   통신 타입 0
```

**중재 id 안에 모터 id 가 들어 있음.** `bit23~8` 을 읽으면 지금 그 모터의 id 임.

데이터는 MCU 고유번호라 id 확인에는 안 씀. 다만 **같은 값이 두 줄 나오면 모터가 두
대 물려 있는 것**이므로 그건 봐 둘 만함 — 한 대만 남기고 다시 함.

응답이 없으면 id 가 0x7F 가 아닌 것임. 0부터 127까지 훑음.

```bash
for i in $(seq 0 127); do
  printf -v id "0000FD%02X" $i                  # 마지막 두 자리가 대상 id
  cansend can1 "$id#0000000000000000"           # Type 0  id 를 0부터 127까지 하나씩 물어봄
  sleep 0.02
done
```

`candump` 에 뜬 응답의 `bit23~8` 이 그 모터의 id 임 — `0000<id>FE` 모양으로 나옴.

---

## 4. id 를 바꿈 — Type 7

무릎을 10(`0x0A`)으로:

```bash
cansend can1 070AFD7F#0000000000000000          # Type 7  id 0x7F 인 모터의 id 를 0x0A 로
```

```
29-bit ID
  bit28~24   0x07          통신 타입 7 (Set motor CAN_ID)
  bit23~16   0x0A          새 CAN id      <- 여기 들어감
  bit15~8    0xFD          호스트 id
  bit7~0     0x7F          지금 id
데이터       비움
```

응답은 **Type 0 형식**으로 옴 (매뉴얼: Answer motor broadcast frame).

```
can1  00000AFE  [8]  41 54 90 07 E8 0C 08 05     <- 새 id 0x0A 로 바뀜
can1  00007FFE  [8]  ...                          <- 아직 옛 id. 안 바뀜
```

**즉시 적용됨.** 재투입이 필요 없음. 응답을 놓쳤으면 새 id 로 한 번 더 물어봄.

```bash
cansend can1 0000FD0A#0000000000000000          # Type 0  새 id(0x0A)로 다시 물어봄
```

`00000AFE` 가 오면 확정임. 안 오면 옛 id(`0000FD7F`)로 물어봄 — 그쪽이 답하면
변경이 안 된 것임.

오른다리는 이렇게 배정함.

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
cansend can1 1200FD0A#2970000001000000          # Type 18  zero_sta(0x7029) 에 1 을 씀. RAM 에만
cansend can1 1600FD0A#0102030405060708          # Type 22  플래시에 커밋. 안 하면 재투입 때 사라짐
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
cansend can1 1100FD0A#2970000000000000          # Type 17  zero_sta 를 읽어 봄
```

응답 `Byte4` 가 `01` 이면 됨.

---

## 6. 프로토콜을 MIT 로 — Type 25

```bash
cansend can1 1900FD0A#0102030405060200          # Type 25  프로토콜을 MIT(2) 로. 재투입해야 적용
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
cansend can1 00A#FFFFFFFFFFFFFFFD               # Command 2  MIT 정지 명령. 표준 프레임(3자리 id)
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
# 채널 -- 어댑터에 맞는 쪽 하나만
sudo slcand -o -c -s8 /dev/canable0 can1 && sudo ip link set can1 up  # USB 어댑터
sudo ip link set can1 up type can bitrate 1000000                     # 네이티브 CAN
candump can1 &                                  # 응답을 계속 지켜봄

# 출하 상태 -- private 프로토콜, 29-bit 확장 프레임 (id 8자리)
cansend can1 0000FD7F#0000000000000000          # Type 0   지금 id 확인      -> 00007FFE
cansend can1 070AFD7F#0000000000000000          # Type 7   id 를 0x0A 로      즉시 적용
cansend can1 0000FD0A#0000000000000000          # Type 0   새 id 로 확인      -> 00000AFE
cansend can1 1200FD0A#2970000001000000          # Type 18  zero_sta -> 1      RAM 에만
cansend can1 1600FD0A#0102030405060708          # Type 22  플래시에 커밋
cansend can1 1100FD0A#2970000000000000          # Type 17  zero_sta 읽기      Byte4 = 01 이면 됨
cansend can1 1900FD0A#0102030405060200          # Type 25  프로토콜 -> MIT     재투입해야 적용

# 전원 재투입

# MIT -- 11-bit 표준 프레임 (id 3자리)
cansend can1 00A#FFFFFFFFFFFFFFFD               # Command 2  정지. 응답 오면 MIT 임
huphy-commission --limb right_leg scan          # 여기서부터 저장소 도구가 붙음
```

`0A` 자리가 대상 모터 id 임. hip_pitch(id 7)면 `1200FD07`, `1600FD07` 처럼 바뀜.
`070AFD7F` 만 두 자리를 씀 -- 앞쪽 `0A` 가 새 id, 뒤쪽 `7F` 가 지금 id.
