# 실제 로봇 결합 — 다리 하나에서 양다리로

지금 코드가 **다리 하나를 어떻게 돌리고 있는지**, 양다리로 가려면 무엇이 더
필요한지 정리함.

계층을 나눈 이유는 [architecture.md](architecture.md), 한 주기에 값이 어떻게
흐르는지는 [cycle.md](cycle.md).

---

## 0. 한눈에

| | 상태 |
|---|---|
| 설정 (`robot.yaml`) | 양다리가 이미 다 적혀 있음 |
| 실행 (제어 객체) | 다리 하나만 만들어짐 |
| 두 다리를 묶는 계층 | **없음.** 자리만 비어 있음 |
| 캘리브레이션 | 1.0 은 양쪽 다 비어 있음 — 실측 전임 |
| 정책 | 관절 6개짜리. 양다리 모델이 아님 |

핵심은 **설정과 실행이 어긋나 있다**는 것임. 설정에는 로봇 전체를 뜻하는 마디가
있는데, 실행에는 그 마디가 없어서 다리가 그 자리에 그대로 앉아 있음.

---

## 1. 지금 다리 하나를 쓰는 방법

### 1.1 두 계층에 "로봇" 이 따로 있음

```python
robot = load_robot(path)          # RobotConfig — 설정. 양다리 다 있음
limb  = _pick_limb(robot, name)   # LimbConfig  — 그중 하나를 고름
leg   = build_leg(robot, limb)    # Leg         — 실행 객체
loop  = ControlLoop(leg, ...)     # 루프에 들어가는 것은 leg
```

| | [`RobotConfig`](../src/huphy/config/schema.py#L263) | [`Robot`](../src/huphy/robots/base.py#L54) |
|---|---|---|
| 무엇 | 야믈을 담은 읽기 전용 자료형 | 관절 이름으로 말하는 실행 객체 |
| 양다리 | 있음 (`limbs` 에 둘 다) | 구현체가 [`Leg`](../src/huphy/robots/leg.py#L101) 하나뿐 |
| 하는 일 | 없음. 데이터임 | 명령 계산·전송·수거 |

`robot` 이라는 변수는 `build_leg` 에 재료로 들어가고 끝임. 제어 루프 근처에
가지 않음.

### 1.2 조립 순서

`huphy-bringup`, `huphy-run`, `huphy-test`, `huphy-commission` 이 전부 같은
순서를 지남 ([bringup.py:584](../src/huphy/scripts/bringup.py#L584)).

```
1  _find_config()          config/robot.yaml 을 찾음
2  load_robot(path)        YAML -> RobotConfig                하드웨어 안 만짐
3  _pick_limb(robot, ...)  팔다리 하나를 고름                  <- 여기서 좁아짐
4  build_leg(robot, limb)  CanBus -> RobStrideBus -> Leg      객체만 생김
5  leg.connect()           CAN 채널을 엶                      <- 처음 하드웨어
6  ControlLoop(leg, ...)   주기·통계·기록을 맡김
```

2~4 는 통신을 하지 않음. 설정 오류(모르는 모델, 관절 이름 불일치, id 중복)는
전부 4 에서 걸리고, 5 에는 배선 문제만 남음.

캘리브레이션 파일은 4 에서 `Leg` 생성자가 읽음. 야믈에서 온 모터 정보와
JSON 의 `limits_deg` 가 합쳐지는 지점이 거기 한 군데임
([leg.py:206](../src/huphy/robots/leg.py#L206)).

### 1.3 각 계층이 맡은 것

```
scripts/     설정을 읽고 객체를 조립함
control/     주기를 지킴. 끝날 때 안전하게 정리함
robots/      관절 이름 <-> 모터 id, 관절각 <-> 모터각          <- 경계
motors/      모터 id 와 raw 각도. 각도 <-> 바이트
canbus.py    프레임 송수신
```

| 계층 | 아는 것 | 모르는 것 |
|---|---|---|
| `control/` | 시간, 주기 | 관절도 모터도 모름 |
| `robots/` | 관절 이름, cal 각도 | 프레임, 바이트 |
| `motors/` | 모터 id, raw 각도 | 관절 이름 |
| `canbus.py` | CAN id 와 8바이트 | 바이트의 뜻 |

제어 루프가 관절을 모르기 때문에, 루프를 고치지 않고도 다른 종류의 로봇을
넣을 수 있음. 이것이 아래 3장의 전제임.

### 1.4 `Leg` 가 하는 일

위에서 관절 목표를 받아 아래로 모터 명령을 냄. 네 가지가 **여기서만** 일어남.

```
입력   {"knee": 30.0, "ankle_pitch": 10.0}      관절 이름, cal 공간
  1  한계·점프 검사        cal 공간에서 먼저 함
  2  발목 pitch/roll -> a/b 각도    기구학
  3  관절 이름 -> 모터 id   robot.yaml 의 매핑
  4  cal -> raw            캘리브레이션의 sign/offset
출력   {10: MitCommand(62.79도, kp, kd)}         모터 id, raw 공간
```

**검사가 변환보다 먼저임.** 한계는 관절 좌표계에서 잰 값이라, 모터 각도로 바꾼
뒤 검사하면 부호가 뒤집힌 관절에서 한계가 반대로 걸림.

### 1.5 한 주기

`Leg` 는 이 일을 세 동작으로 나눠 가지고 있음
([base.py:140](../src/huphy/robots/base.py#L140)).

| | 하는 일 | CAN |
|---|---|---|
| `build_commands` | 위 네 가지 계산 | 안 씀 |
| `send` | 계산해 둔 명령 전송 | 씀 |
| `collect` | 응답 수거, 상태 갱신 | 씀 |

다리가 하나면 `send_action()` 이 셋을 순서대로 부르면 됨. 나눠 둔 이유는 2.2 에
있음.

---

## 2. 왜 이렇게 되어 있나

### 2.1 다리 하나로 시작했음

0.5 하드웨어가 다리 하나였음. 그 시점에 양다리 관리 계층을 만들었다면 실물로
검증한 적 없는 추상이 하나 늘 뿐임. 대신 **끼울 자리를 인터페이스로 남겨** 둠.

### 2.2 이미 깔려 있는 것

양다리를 전제하고 만들었으나 아직 쓰이지 않는 것들임.

| 무엇 | 어디 | 지금 |
|---|---|---|
| 계산·전송·수거 분리 | [base.py:140](../src/huphy/robots/base.py#L140) | `send_action` 이 셋을 묶어 씀 |
| 여러 버스 수거 (`drain_all`) | [canbus.py:303](../src/huphy/motors/canbus.py#L303) | 테스트만 씀 |
| 기록 필드에 팔다리 이름 | [snapshot.py:151](../src/huphy/telemetry/snapshot.py#L151) | `right_leg/knee/pos` 로 이미 나감 |
| 왼다리 발목 거울상 | [bringup.py:115](../src/huphy/scripts/bringup.py#L115) | `side` 로 갈림 |
| 채널 공유 시 id 충돌 검사 | [schema.py:293](../src/huphy/config/schema.py#L293) | 채널이 달라 안 걸림 |
| 팔다리 선택 도구 | [schema.py:319](../src/huphy/config/schema.py#L319) | 부르는 곳 없음 |

계산·전송·수거를 나눈 이유가 핵심임 ([이슈 #10](issues.md)). 다리마다
"계산 → 전송 → 수거" 를 돌리면 이렇게 됨.

```
왼다리   [계산][전송][---- 응답 대기 ----]
오른다리                                    [계산][전송][대기]
```

두 CAN 채널은 물리적으로 독립이라 실제로 겹쳐 보낼 수 있는데 그 병렬성을 못 씀.
두 다리의 명령 시각도 벌어짐. 나눠 두면 이 순서를 짤 수 있음.

```
왼다리 계산 -> 오른다리 계산 -> 왼다리 전송 -> 오른다리 전송 -> 수거
```

### 2.3 비어 있는 자리

```
설정 계층                   실행 계층
RobotConfig        ->       (없음)                <- 비어 있는 자리
 ├ limbs["right"]  ->         Leg("right_leg")    <- 루프의 뿌리가 여기
 ├ limbs["left"]   ->         (안 만들어짐)
 ├ safety          ->         Leg.safety
 ├ imus            ->         Imu (mount 로 골라 들어감)
 └ telemetry       ->         Telemetry
```

`Robot` 계약이 이 자리를 명시해 둠 —
"다리, 팔, 나중에는 **양다리를 묶은 것**도 이 계약을 채움"
([base.py:55](../src/huphy/robots/base.py#L55)).

---

## 3. `Biped` 를 넣으면

### 3.1 위치

`Leg` 를 대체하지 않음. 위에 얹혀 같은 `Robot` 계약을 채우고, 안에 `Leg` 둘을 둠.

```
ControlLoop
 └ robot -> Biped(Robot)                        <- 새로 씀
             ├ Leg("right_leg") -> RobStrideBus -> CanBus("can1")
             └ Leg("left_leg")  -> RobStrideBus -> CanBus("can0")
```

### 3.2 하는 일

순서 지휘가 전부임.

```python
def build_commands(self, action):
    return {leg.id: leg.build_commands(split(action, leg.id)) for leg in self.legs}

def send(self, commands):
    return sum(leg.send(commands[leg.id]) for leg in self.legs)      # 전송을 몰아서

def collect(self):
    return tuple(chain.from_iterable(leg.collect() for leg in self.legs))
```

### 3.3 바뀌는 것과 안 바뀌는 것

| | |
|---|---|
| `ControlLoop` | 안 고침. `Robot` 계약만 보므로 그대로 받음 |
| `Leg` | 안 고침. 자기가 묶였다는 것을 몰라도 됨 |
| `RobStrideBus`, `CanBus` | 안 고침. 애초에 팔다리를 모름 |
| 설정 읽기 | 안 고침. 이미 양다리를 읽음 |
| 텔레메트리 | 안 고침. 필드에 팔다리 이름이 이미 붙음 |
| `Biped` 클래스 | **새로 씀** |
| CLI 진입 경로 | **새로 씀.** `--limb` 하나 대신 다리 종류를 전부 잡음 |

---

## 4. 앞으로 해야 할 것

### 하드웨어

- **양다리 실측** — `huphy-commission sweep` 으로 `offset_deg` 와 `limits_deg` 를 잼.
  1.0 캘리브레이션은 지금 **양쪽 다 비어 있음.** 값이 없으면 `is_configured` 가
  거짓이라 토크가 나가는 경로가 막힘 (오른다리도 마찬가지임)
- **왼다리 배선·전원·CAN 채널 확인** — `huphy-commission --limb left_leg scan`
- **모터 매핑 확인** ([이슈 #8](issues.md)) — 명령한 관절이 실제로 움직이는지

### 코드

- **`Biped(Robot)` 작성** — `robots/biped.py`. 계산·전송·수거를 두 다리에 지휘
- **CLI 경로 추가** — 팔다리를 전부 잡는 분기. 설정 쪽 도구는 이미 있음
- **관절 이름 규칙 결정** — 다리가 둘이면 `knee` 가 둘임. `right_leg.knee` 처럼
  접두어를 붙이고, 정책 출력과 이름을 맞추는 규칙을 같이 정함
- **한쪽 다리 통신 두절 시 동작 결정** — 지금은 각 `Leg` 가 자기 모터만 셈.
  한쪽이 끊긴 채 다른 쪽만 명령하면 로봇이 넘어짐. "한쪽이 죽으면 둘 다 정지" 를
  `Biped` 가 판정해야 함
- **정지 절차 확장** — 지금 종료 경로는 현재 자세를 몇 주기 붙잡고 토크를 끊음.
  두 다리가 동시에 이 과정을 거치도록 `Biped` 가 지휘
- **주기 예산 확인** — 모터가 6개에서 12개가 됨. 수거가 가장 비쌈.
  `drain_all` 이 순차라 대기가 버스 수만큼 곱해짐 ([이슈 #10](issues.md) 의 남은 것)
- **수신 스레드** — 위 예산이 넘칠 때만. 버스마다 스레드를 두어 제어 루프에서
  수거를 없앰. `canbus.py` 의 락 구성을 다시 봐야 함

### 정책

- **양다리 시뮬 모델** — 지금 모델은 관절 6개, 관찰 24/26칸이고 시뮬 파일이
  `half_huphy.xml` 임 ([policy.py:51](../src/huphy/control/policy.py#L51))
- **재학습** — 코드 수정으로는 안 됨. 별개 작업임

---

## 5. 예상 순서

| 단계 | 무엇 | 끝났다는 기준 |
|---|---|---|
| 1 | 왼다리 배선·CAN 확인 | `scan` 에 모터 6개가 다 뜸 |
| 2 | 양다리 실측 (`zero` → `sweep`) | 두 JSON 의 `limits_deg` 가 채워짐 |
| 3 | 다리별 단독 브링업 | 양쪽 다 `huphy-bringup` 으로 사인파가 돎 |
| 4 | `Biped` + CLI 경로 | 한 프로세스가 두 다리를 같은 주기로 돌림 |
| 5 | 주기 예산 측정 | 12모터에서 목표 Hz 를 지킴. 못 지키면 수신 스레드 |
| 6 | 양다리 시뮬 모델과 재학습 | 시뮬에서 서고 걷는 것을 확인 |
| 7 | 정책 실물 적용 | — |

1~3 은 하드웨어 작업이고 코드를 안 고침. 4~5 가 이 문서의 본론임. 6~7 은 별개
프로젝트에 가까움.

**4 를 3 보다 먼저 하지 않을 것.** 다리 하나가 혼자 제대로 돌지 않는 상태에서
둘을 묶으면, 문제가 다리에 있는지 묶는 계층에 있는지 갈리지 않음.
