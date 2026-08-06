# `scripts/` — 터미널 진입점

```
scripts/
└── commission.py   조립할 때 한 번 하는 조작
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
2  scan          6개 다 응답하나
3  fault         고장 없나
4  state         지금 어디 있나
5  nudge <관절>   설정대로 움직이나 (관절마다)
6  zero <관절>    영점 (자세를 잡아 놓고 하나씩)
```

---

## 미구현

| 파일 | 용도 | 필요해지는 시점 |
|---|---|---|
| `bringup.py` | 대화형 메뉴. 제어 루프를 타고 텔레메트리가 흐름 | 9단계 |

---

## 테스트

```bash
PYTHONPATH=src python3 -m pytest tests/test_commission_cli.py -q
```

30개. 가짜 모터가 붙은 가짜 CAN 버스에 대고 명령줄을 그대로 돌림. 확인하는 것은
사람이 보는 출력과 종료 코드, 그리고 거부 조건임.

설정 파일은 임시 폴더에 만들어 씀 — 테스트가 저장소의 실제 파일을 건드리면 안 됨.
