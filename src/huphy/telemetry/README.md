# `telemetry/` — 무슨 일이 일어났는지 내보냄

```
telemetry/
├── snapshot.py   한 시점의 사전. 필드 이름을 정하는 유일한 곳
├── udp.py        UDP JSON 송신. 실시간으로 보는 것
└── csv_log.py    CSV 기록. 끝나고 되돌아보는 것
```

---

## 왜 제어보다 먼저인가

게인을 튜닝하려면 **목표와 실측을 겹쳐 봐야 함.**

```
목표를 지나쳤다 돌아옴    ->  kd 를 올림
목표까지 못 감            ->  kp 를 올림
부르르 떨림               ->  kp 를 내림
```

눈으로만 보면 "지나쳤다 돌아온다" 를 구분할 수 없음. 그래프가 없으면 게인을
찾을 수 없고, 게인이 없으면 다리가 안 움직임.

---

## 필드 이름은 한 곳에서만 정함

`snapshot.py` 가 사전 하나를 만들고 **UDP 와 CSV 가 둘 다 그것을 소비함.**

두 군데에서 필드를 만들면 반드시 어긋남 — CSV 헤더에는 있는데 UDP 에는 없는 값이
생기고, 어느 쪽이 맞는지 알 수 없어짐.

```python
snapshot.field_names(robot)   # 실행 전에 알 수 있음
snapshot.build(robot, t=1.5)  # 그 목록과 정확히 같은 키
```

테스트가 둘이 같은지 고정함.

### 실행 전에 알 수 있어야 함

CSV 헤더를 첫 줄에 써야 하고, PlotJuggler 레이아웃도 미리 만들어 둬야 함.

**필드가 나타났다 사라지면 안 됨.** 값이 없어도 키는 내보내고 `0` 을 채움 — 중간에
사라지면 그래프가 끊기고 CSV 열이 밀림.

---

## 이름 규약

```
t                            시작부터 흐른 초
loop_dt                      직전 루프 실제 주기 (ms)
missing                      이번 주기 무응답 모터 수

right_leg/knee/pos           실측 위치 (cal 공간)
right_leg/knee/tgt           목표 위치
right_leg/knee/err           tgt - pos
right_leg/knee/vel           실측 속도
right_leg/knee/tau           실측 토크
right_leg/knee/temp          권선 온도

right_leg/guard/clip_limit   한계에 잘린 횟수 (누적)
right_leg/guard/clip_jump    점프에 잘린 횟수
right_leg/guard/reject_nan   NaN 으로 버린 횟수
right_leg/guard/reject_nostate  현재 위치를 몰라 버린 횟수

right_leg/can/tx_errors      송신 실패
right_leg/can/rx_errors      수신 실패
right_leg/can/drain_timeouts 기대한 응답을 못 채운 횟수
```

`/` 로 나눔. PlotJuggler 가 트리로 묶어 보여줌 — 모터가 20개를 넘어가면 평평한
목록에서는 찾을 수 없음.

**팔다리 이름이 앞에 붙음.** 양다리를 같이 기록할 때 `knee` 가 둘이 되기 때문임.

### `tgt` 는 실제로 나간 명령임

명령한 값이 아니라 **잘리고 남은 값**임 (`robot.last_sent`).

그래야 오차를 보고 "왜 모터가 저기까지만 갔지" 가 설명됨. 명령한 값을 기록하면
한계에 걸린 것과 게인이 낮은 것이 그래프에서 구분되지 않음.

발목은 명령이 관절(pitch/roll)로 오므로 모터별 목표가 없음. 그때는 실측을 목표로
둬서 오차를 0으로 냄 — 가짜 오차가 그래프에 남는 것보다 나음.

---

## 팔다리마다 패킷 하나

**다리 하나가 필드 46개에 약 1.3 KB 임.**

둘을 한 패킷에 합치면 이더넷 MTU(1500)를 넘어 조각남. **조각 하나만 잃어도 패킷
전체가 버려져** 손실률이 확 올라감.

```python
Telemetry(left_leg,  host=...)    # 패킷 하나
Telemetry(right_leg, host=...)    # 패킷 하나
```

PlotJuggler 는 여러 출처를 같은 타임라인에 올림. 팔·상체까지 붙으면 이 구성이
그대로 늘어남.

CSV 는 크기 제약이 없으므로 `merge()` 로 한 파일에 모아도 됨.

---

## `udp.py`

**보내고 잊음.** 받는 쪽이 없어도, 느려도, 꺼져 있어도 제어 루프가 멈추지 않음.

### 왜 TCP 가 아닌가

TCP 는 상대가 안 받으면 **송신이 막힘.** 제어 루프 한가운데서 막히면 주기가 통째로
밀림. UDP 는 커널 버퍼에 넣고 바로 돌아옴.

패킷이 몇 개 빠져도 상관없음 — 그래프에 점 하나가 비는 것뿐이고, 100Hz 로 보내므로
눈에 띄지도 않음.

### 예외를 던지지 않음

네트워크가 끊기거나 상대가 꺼져 있는 것은 **로봇 입장에서 정상 상황임.** 실패는
세기만 하고, 처음 한 번만 로그를 남김.

관측이 제어를 멈추면 관측할 대상이 없어짐.

### 반올림

소수점 둘째 자리까지 보냄. 패킷 크기를 반으로 줄이는 데 이게 제일 큼.

0.01도는 모터 해상도(0.022도)보다 촘촘해서 **정보를 잃지 않음.**

### 시간축

`t` 를 PlotJuggler 의 timestamp 필드로 지정할 것. 지정하지 않으면 **수신 시각**을
쓰게 되어 네트워크 지터가 데이터에 섞임.

시각은 첫 호출을 0으로 하는 상대 시간임. 벽시계를 쓰면 x 축이 `1.7e9` 같은 값에서
시작해 읽을 수 없음.

### 자체 카운터는 UDP 로 안 나감

```python
tm.as_fields()   # {'udp.sent': ..., 'udp.errors': ..., 'csv.rows': ...}
```

나가는 경로가 고장났는데 **그 사실을 같은 경로로 알릴 수는 없음.** 로그와 CSV 로 봄.

---

## `csv_log.py`

UDP 는 실시간으로 보는 것이고, 이쪽은 **끝나고 나서 되돌아보는 것**임. 사고가 났을
때 그 직전 몇 초를 프레임 단위로 다시 볼 수 있어야 함.

### 열이 실행 전에 정해짐

첫 줄에 헤더를 쓰고 나면 열이 고정됨. 헤더에 없는 키는 버리고 셈.

**열이 밀리면 기록 전체가 못 쓰게 됨.** 나중에 열어 보면 어느 열이 무엇인지 알 수
없고, 그걸 알아채는 것은 보통 사고 조사 중임.

### 이어 쓰지 않음

열 구성이 실행마다 다를 수 있는데(모터를 추가하거나 관절 이름을 바꾸면) 이어 쓰면
한 파일 안에 두 형식이 섞임.

### 디스크 쓰기가 주기를 흔듦

`flush_every` 주기마다 한 번만 밀어 넣음. 기본 50이면 100Hz 에서 0.5초마다임.

**멈출 때는 반드시 밀어 넣음.** 버퍼에 남은 것이 사라지면 사고 직전 몇 줄을 잃는데,
하필 그게 제일 보고 싶은 부분임.

소수점은 셋째 자리까지 — UDP 보다 한 자리 더 남김. 크기 제약이 없고, 나중에
미분해서 볼 때 반올림 오차가 커짐.

---

## 쓰는 법

```python
from huphy import telemetry

tm = telemetry.Telemetry.from_config(leg, robot_config.telemetry)

with tm:
    while running:
        leg.send_action(action)
        tm.record(loop_dt_ms=dt * 1000, missing=len(missing))
```

설정에서 둘 다 꺼 두면 아무것도 하지 않음 — **호출부가 분기하지 않아도 되게 함.**

```yaml
telemetry:
  host: null            # 비우면 UDP 안 보냄
  port: 9870
  csv_path: null        # 비우면 파일 안 만듦
  csv_flush_every: 50
```

---

## 받는 쪽

```bash
sudo ufw allow 9870/udp
ping <파이 IP>
echo '{"t":1,"x":42}' | nc -u -w1 <우분투 IP> 9870
```

PlotJuggler 에서 UDP Server 를 열고 포트와 `t` 를 지정하면 됨. 자세한 것은
[docs/monitoring.md](../../../docs/monitoring.md) 참조.

---

## 테스트

```bash
PYTHONPATH=src python3 -m pytest tests/test_telemetry.py -q
```

36개. UDP 는 **진짜 소켓**으로 자기 자신에게 보내 받아 봄 — 직렬화와 반올림이
실제로 왕복하는지 확인함. CSV 는 임시 폴더에 씀.

고정하는 것:

```
field_names() 와 build() 의 키가 같음
다리 하나가 MTU 안에 들어감
송신·기록이 실패해도 예외를 던지지 않음
close() 가 버퍼를 밀어 넣음
헤더에 없는 필드를 버리고 셈
```
