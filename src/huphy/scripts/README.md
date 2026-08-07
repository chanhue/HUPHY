# `scripts/` — 터미널 진입점

```
scripts/
├── commission.py   조립할 때 한 번 하는 조작
└── bringup.py      다리를 실제로 움직여 보는 대화형 메뉴
```

설정 파일에서 모터 목록을 읽으므로 **모터 id 를 손으로 적지 않음.**

---

## `commission.py`

```bash
python -m huphy.scripts.commission --limb right_leg scan
python -m huphy.scripts.commission --limb right_leg state
python -m huphy.scripts.commission --limb right_leg nudge knee --delta 5
python -m huphy.scripts.commission --limb right_leg zero knee --note "다리 편 상태" --yes
```

설치하면 `huphy-commission` 으로도 부를 수 있음.

### 명령

| | 무엇 |
|---|---|
| `scan` | 어느 모터가 응답하는지 |
| `state` | raw 와 cal 을 나란히. 속도·토크·온도 |
| `fault` | 고장 상태 조회 |
| `clear-fault` | 고장 상태 지우기 |
| `sweep` | 토크를 끄고 손으로 밀어 가동 범위 측정 |
| `nudge` | 조금 움직였다 되돌림 |
| `zero` | **[영구]** 지금 자세를 기계 영점으로 |
| `mode` | 제어 모드 변경 |
| `can-id` | **[영구]** CAN id 변경 |
| `protocol` | **[영구]** 프로토콜 전환 |

---

## 관절 이름으로 말함

```
nudge knee      O
nudge 10        X
```

사람은 관절로 생각하고, **모터 id 는 배선이 바뀌면 달라짐.** 설정에 적힌 이름을
그대로 씀.

없는 이름을 주면 가용 목록을 알려줌.

```
right_leg 에 'elbow' 관절이 없음
(가용: ['ankle_a1', 'ankle_a2', 'hipx', 'hipy', 'hipz', 'knee'])
```

---

## `--limb` 을 요구함

다리가 둘이고 **각각 다른 CAN 채널에 있음.** 잘못 고르면 엉뚱한 다리가 움직임.

```
--limb 을 지정할 것 (가용: ['left_leg', 'right_leg']).
팔다리마다 CAN 채널이 달라 잘못 고르면 엉뚱한 쪽이 움직임
```

팔다리가 하나뿐이면 생략 가능함.

---

## 되돌리기 어려운 것은 `--yes` 를 요구함

```
zero        기계 영점. 모터에 저장되고 좌표계가 통째로 옮겨감
can-id      CAN id 변경. 바꾼 뒤 robot.yaml 도 고쳐야 함
protocol    프로토콜 전환. 전원 재투입 필요
```

**승인 확인이 버스를 열기 전에 일어남.** 모터에 아무것도 보내지 않은 상태에서 멈춤.

```
$ ... zero knee --note "편 상태"
right_leg.knee: 기계 영점을 지금 자세로 잡음. 모터에 저장되고 좌표계가 통째로 옮겨감.
되돌리기 어려운 조작임. 확인했으면 --yes 를 붙일 것.
```

`--yes` 는 최상위가 아니라 이 세 명령에만 붙음. argparse 가 최상위 플래그를 서브명령
뒤에 받지 않아서, 최상위에 두면 `--yes zero knee` 로 써야 함.

---

## 설정 파일

`--config` 를 주지 않으면 **현재 폴더부터 위로 올라가며** `config/robot.yaml` 을
찾음. 저장소 어디서 실행하든 같은 파일을 씀.

설정에 오류가 있으면 **채널을 열기 전에** 멈춤.

---

## 명령별 동작

### `scan` — 원인 후보를 같이 냄

```
right_leg  can1  모터 6개

  hipz       id=7   RS02   응답
  ...
  ankle_a2   id=12  RS00   ----

응답 없음: ['ankle_a2']
  배선, 전원, CAN id, 프로토콜 모드가 후보임.
  이 넷은 여기서 구분되지 않음 -- 전부 조용히 빠짐 (이슈 #11).
```

넷 다 조용히 빠지므로 **어디부터 봐야 할지 알려줘야 함.**

### `state` — raw 와 cal 을 나란히

```
  관절               raw       cal        속도       토크     온도
  knee           29.99     29.99     -0.46     0.39   32.5

  미실측 상태라 cal 이 raw 와 같음.
```

같은 값이 두 번 나오면 버그처럼 보이므로 **이유를 밝혀 둠.**

### `sweep` — 가동 범위 측정

```bash
huphy-commission --limb right_leg sweep          # 전 관절
huphy-commission --limb right_leg sweep knee     # 하나만
```

토크를 끄고 **사람이 관절을 양쪽 끝까지 미는 동안** 최대·최소를 기록함. Enter 로
끝냄.

```
  관절                최소        지금        최대        범위
  hipz         -116.55    -21.35    -21.18     95.37
  knee          -20.61     74.76     74.76     95.37
```

끝나면 `robot.yaml` 에 붙일 수 있는 형태로 냄.

```
      knee:      {id: 10, model: RS02, limits_deg: [-20.61, 74.76], kp: 0.0, kd: 0.0}
```

**파일을 고치지는 않음.** `robot.yaml` 은 사람이 적는 파일이고 주석이 많음 —
프로그램이 다시 쓰면 주석이 날아감.

발목도 같이 잼. 발을 잡고 움직이면 두 모터가 같이 따라옴.

범위가 5도도 안 되는 관절이 있으면 알려주고 종료 코드 `1` 을 냄 — 끝까지 안 민
것임.

### `nudge` — 어느 관절인지 눈으로 확인

이슈 #8(모터 id ↔ 관절 매핑 실물 미확인)을 해소하는 절차임. 설정에는
`7=hipz 8=hipx 9=hipy 10=knee 11=ankle_a1 12=ankle_a2` 로 되어 있지만 실물로
확인된 적이 없음.

```
right_leg.knee (id=10) 를 +5.0도 움직였다 되돌림.
  게인 kp=5.0 kd=0.5
  다리를 받쳐 두고, 실제로 어느 관절이 움직이는지 볼 것.

  시작    29.99
  최대    34.72   (움직인 양 +4.73)
  끝      30.23
```

명령한 양의 30% 미만이면 알림 — 게인이 낮거나, 중력을 못 이기거나, 걸린 것임.

응답 없는 모터에는 **토크를 넣지 않음.** 배선부터 확인할 일임.

### `zero` — 메모를 파일에 적음

```bash
... zero knee --note "다리 편 상태, 발바닥 평면 접촉" --yes
```

```json
"knee": {"sign": 1.0, "offset_deg": 0.0, "zero_reference": "다리 편 상태, 발바닥 평면 접촉"}
```

모터는 영점 값을 저장하지만 **"그때 어떤 자세였는지" 는 어디에도 안 남음.** 명령이
끝나면 캘리브레이션 파일에 바로 적음. 다른 관절 값은 건드리지 않음.

토크를 먼저 끊고 영점을 잡음 — 순서가 반대면 직전 목표각이 옛 좌표계 값이라 그
차이만큼 관절이 튐.

---

## 채널이 안 열리면

```
can1 (socketcan) 를 열 수 없음: ...
채널이 올라와 있는지 확인할 것:
  sudo ip link set can1 up type can bitrate 1000000
  ip -details link show can1
```

---

## 종료 코드

```
0   정상
1   응답 없는 모터가 있거나 고장이 있음
```

`scan`, `state`, `fault` 는 문제가 있으면 `1` 을 냄. 스크립트로 엮을 때 씀.

---

## 실물에서의 순서

```
1  sudo ip link set can1 up type can bitrate 1000000
2  commission scan          6개 다 응답하나
3  commission fault         고장 없나
4  commission state         지금 어디 있나
5  commission nudge <관절>   설정대로 움직이나 (관절마다)
6  commission zero <관절>    영점 (자세를 잡아 놓고 하나씩)
7  commission sweep         가동 범위. 영점 뒤에 재야 함
8  bringup                  게인 튜닝. 그래프를 보며
```

**7 이 6 뒤인 이유**: `sweep` 이 raw 공간으로 재는데 raw 는 영점에 매달려 있음.
영점을 다시 잡으면 범위도 다시 재야 함.

---

## `bringup.py`

```bash
python -m huphy.scripts.bringup --limb right_leg
python -m huphy.scripts.bringup --limb right_leg --gain-scale 0.1 --allow-uncalibrated
```

설정·실측값·버스·기구학·안전·텔레메트리·제어 루프를 한데 묶어 씀. **모든 계층이
여기서 만나는 유일한 곳임.**

### 메뉴

```
1. 상태 보기
2. 카운터 보기
3. 자세 유지 [토크]
4. 한 관절 옮기기 [토크]
5. 계단 응답 [토크]
6. 사인파 왕복 [토크]
```

`[토크]` 가 붙은 항목은 힘이 나감. **미실측이면 거부함** — `--allow-uncalibrated`
가 있어야 진행됨.

### 움직이는 것은 전부 제어 루프를 탐

```
menu -> motion -> ControlLoop -> Leg -> bus
```

메뉴가 로봇을 직접 부르지 않음. **동작만 정하고 루프에 넘김.**

직접 부르면 그 경로에서만 이것들이 빠짐 (이슈 #4).

```
텔레메트리 (UDP·CSV)
주기 측정 (loop_dt, overruns, kept_up)
정지 순서 (hold 후 토크 차단)
```

그러면 그래프가 안 나오는데 **텔레메트리가 고장난 줄 알게 되고**, 같은 일을 하는
코드가 두 벌이 되어 한쪽만 고쳐짐.

테스트가 `ControlLoop.run` 을 감시해 이것을 고정함.

### 상태 보기

```
  관절               raw       cal        속도       토크     온도  ack   age(ms)
  hipz          -60.01    -60.01     -0.46     0.32   30.0    1      0.02
  ...

  발목  pitch   -0.00   roll    0.01
```

`ack` 와 `age` 로 **명령이 씹혔는지** 바로 봄. 발목은 모터 각도가 아니라
pitch/roll 로 보여줌 — 사람이 보고 싶은 것이 그것임.

### 한계 밖에 있으면 경고함

```
  ** 한계 밖에 있는 관절 **
  토크를 넣으면 가드가 한계 안으로 끌어당김 -- 그 방향으로 움직임.
    knee         200.00  한계 -20.65 ~ 74.79
```

다리를 손으로 옮겨 놓았거나, 한계값이 실물과 다른 것임. **모르고 토크를 넣으면
관절이 그 방향으로 감.**

### 움직이기 전에 `freeze`

모든 동작이 `freeze` 로 시작함 — 지금 자세를 잡고 거기서 출발함. 목표를 0으로 잡고
시작하면 **토크를 넣는 순간 다리가 0을 향해 한 번에 움직임.**

흔들지 않는 관절은 지금 자리에 붙잡아 둠. 여럿을 같이 흔들면 어느 관절이 원인인지
섞임.

### 게인을 낮춰 시작

```bash
--gain-scale 0.1     # 설정값의 10%
```

튜닝값을 찾은 뒤에도 처음엔 낮게 시작하는 것이 안전함.

### 안전

```
시작할 때    관찰 모드. 토크가 꺼져 있음
움직이기 전  freeze 로 지금 자세를 잡음
끝날 때      hold 후 토크 차단
Ctrl-C       루프의 finally 를 지나므로 같은 순서
```

---

## 둘을 나눈 이유

| | `commission.py` | `bringup.py` |
|---|---|---|
| 무엇 | 조립할 때 한 번 | 반복해서 움직여 봄 |
| 주기 | 없음 | 제어 루프 |
| 되돌리기 | 어려움 | 쉬움 |
| 텔레메트리 | 없음 | 있음 |

영점이나 CAN id 는 **반복하지 않는 조작이라 주기가 없음.** 제어 경로에서 손 닿는
곳에 두지 않으려는 것이기도 함.

---

## 테스트

```bash
PYTHONPATH=src python3 -m pytest tests/test_commission_cli.py tests/test_bringup.py -q
```

`commission` 30개, `bringup` 26개. 가짜 모터가 붙은 가짜 CAN 버스에 대고 명령줄과
메뉴를 그대로 돌림.

`bringup` 은 루프가 실제로 도는 항목이 있어 시간이 걸림(약 7초).

설정 파일은 임시 폴더에 만들어 씀 — 테스트가 저장소의 실제 파일을 건드리면 안 됨.
