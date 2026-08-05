# K-Bot 제작 사례에서 얻은 것

같은 **RobStride 모터 + CAN + LeRobot** 조합으로 휴머노이드(K-Bot) 팔을 만든 사람의
제작기에서, 우리 프로젝트에 걸리는 내용을 정리한다.

우리와 조건이 거의 같다 — RobStride 모터 6개, 하나의 CAN 버스, 100 Hz 제어,
라즈베리파이, LeRobot 연동 검토. 그래서 그가 겪은 문제가 우리에게도 그대로 온다.

---

## 1. ⚠️ 프로토콜 불일치 — 가장 먼저 확인할 것

> "The motors on my K-Bot are set to Robstride's **default protocol, the private mode,
> which uses the 29-bit extended identifier.** ... with the bus on 29 bits and LeRobot
> sending on 11, **the motor simply wouldn't move, and without throwing any error**,
> which threw me off quite a bit at the start."

| | 프레임 | ID 폭 |
|---|---|---|
| RobStride 공장 기본값 | private | **29-bit 확장** |
| LeRobot의 robstride 지원 | MIT | 11-bit 표준 |
| **HUPHY 현재 코드** | MIT | **11-bit 표준** |

### 왜 위험한가

`bus.py`가 `is_extended_id=False`(11-bit)로 보내는데 모터가 공장 기본값이면
**명령이 무시되고 에러도 나지 않는다.** 연결도 되고 코드도 안 죽는데 모터만 안
움직인다. 진단이 가장 어려운 종류의 실패다.

우리 매뉴얼 조사와도 일치한다 — RS02 매뉴얼 Command 8 "Change Communication Protocol
(Takes Effect After Power Cycle)", `F_CMD`: 0=Private(기본), 1=CANopen, 2=MIT.

### 확인 방법

1. 제조사 GUI(motorstudio)로 파라미터 `0x201F protocol_1` 읽기
2. 11-bit 표준 프레임으로 `enable`(`0xFC`)을 보내 응답이 오는지 — 오면 MIT 모드

### 두 갈래

| 경로 | 방법 | 비용 |
|---|---|---|
| **A. 모터를 MIT로 전환** | `commissioning.set_protocol()` + 모터마다 전원 재투입 | 모터 6개 × 1회. 이후 현재 코드 그대로 |
| **B. private 모드로 대화** | `codec/private.py` 구현 (29-bit 확장 프레임) | 코덱 하나 추가. 모터는 안 건드림 |

글쓴이는 **B**를 골랐다 — "모터마다 하나씩 프로토콜 전환을 하기 싫어서". 그는 팔
하나(6개)였고 우리도 다리 하나(6개)라 **A도 충분히 현실적이다.** 다만 양다리로 가면
12개가 되고, 모터를 교체할 때마다 반복해야 한다.

> **우리는 구조적으로는 준비되어 있다.** `tables.py`에 `Protocol` enum과
> `PRIVATE_ENCODING`/`MIT_ENCODING`이 둘 다 있고 `commissioning.set_protocol()`도
> 있다. 빠진 것은 `codec/private.py` 하나뿐이다. (→ [architecture.md](architecture.md) §2.1)

---

## 2. 참고 프로젝트 · 라이브러리

| 이름 | 무엇 | 우리 관련성 |
|---|---|---|
| [motorbridge](https://github.com/motorbridge/motorbridge) | Rust 코어 + C ABI + Python/C++ 바인딩. **private 29-bit 프로토콜**로 RobStride를 제어. aarch64 휠이 있어 라즈베리파이에서 바로 동작 | ★ 경로 B를 택하면 직접 쓰거나 참고 |
| [robstride (crates.io)](https://crates.io/crates/robstride) | RobStride 전용 Rust 크레이트. `Protocol` 타입 존재 | 프로토콜 처리 참고 |
| [K-Scale K-Bot](https://docs.kscale.dev/robots/k-bot/quickstart/) | 오픈소스 휴머노이드. Onshape CAD + 관절/링크 정의 + Discord 아카이브 | 하드웨어·URDF 참고 |
| OpenArm | 오픈소스 로봇 팔 | CAN 컨버터 선정 근거로 사용됨 |
| [Seeed RobStride 가이드](https://wiki.seeedstudio.com/robstride_control/) | RobStride 제어 종합 문서 | 프로토콜 교차 확인 |
| SavvyCAN-FD-X2 | USB-CAN 컨버터, CAN-FD 12 Mbps | 하드웨어 선택지 |

---

## 3. 확장성에 대한 서술 4가지

### 3.1 상태 캐시 — 명시적으로 "scale well"이라 한 부분

> "One detail that **makes this scale well**: I don't sit waiting for each motor's reply
> in the middle of the loop. The library keeps a little memory box per motor with the
> last state it reported... and keeps updating that box **in the background** as the
> return frames come in. When my code asks where the motor is, it reads that memory
> right away, **without going out to the wire.**"

**RX 처리 방식은 세 단계로 나뉜다:**

| 방식 | 한 사이클의 RX 비용 | 모터 N개일 때 |
|---|---|---|
| ① 모터마다 보내고 응답 대기 | 왕복 × N | **O(N)** — USB-CAN 왕복이 ~1 ms면 6개에 6 ms. 10 ms 예산이 무너진다 |
| ② 일괄 전송 후 한 번에 드레인 | 드레인 1회 | O(1)에 가깝지만 **드레인이 제어 루프 안에** 있다 |
| ③ 백그라운드 RX 스레드 + 캐시 | **0** | 읽기가 메모리 접근. 모터를 늘려도 제어 루프가 안 느려진다 |

**HUPHY는 현재 ②다.** `sync_write_mit()`이 전부 보낸 뒤 `can.drain()`을 부르는데,
이것이 **호출한 스레드(=제어 루프)에서 돈다.** RX 전용 스레드가 없다.

`canbus.py`의 `drain()` docstring에 이미 적어둔 비용:
> "큐가 비면 마지막 recv가 timeout만큼 블로킹된다. 제어 루프 주기 예산에서 이 시간이
> 매 호출 소모된다."

즉 **그의 ③이 우리보다 한 단계 앞서 있다.** 모터 6개면 ②로도 되지만, 양다리 12개로
가면 차이가 난다.

### 3.2 대역폭 헤드룸

> "at peak use, with six motors at 100 times per second, I take up around
> **15% of the bus bandwidth**"

우리도 6개 × 100 Hz라 그대로 적용된다. **양다리 12개로 가도 30%.**

→ **대역폭은 병목이 아니다.** 병목은 위 3.1의 RX 처리 방식이다.

### 3.3 인터페이스 계약 준수 = 생태계 재사용

> "The base class contracts hold for any robot, so if mine follows those contracts,
> **it drops into that pipeline and reuses** the recording, training, and visualization
> tools that are already there."

인터페이스만 맞추면 기록·학습·시각화 도구가 따라온다. 우리 `Robot` ABC와
`observation_features`가 같은 발상이지만, **LeRobot 이름 규약은 아직 안 맞췄다**
(→ [architecture.md](architecture.md) §6-a 미결정 사항).

### 3.4 제어율과 기록율 분리

> "control runs at 100 times per second, the recording happens at 30 frames per second.
> Storing three images and writing everything to disk on every control step would be
> **too heavy**"
> "The image writing runs on **separate threads** so it doesn't stall the control loop"

우리 CSV flush 정책(N사이클 버퍼링 + E-STOP 시 즉시)과 같은 발상이고,
[monitoring.md](monitoring.md)의 `joint_field_decimation`도 같은 축이다.

---

## 4. 우리 설계와 같은 결론에 도달한 것들

시행착오로 도달한 지점을 우리는 이미 코드에 넣어뒀다. **설계 검증으로 좋은 신호다.**

| 그의 서술 | HUPHY 코드 |
|---|---|
| "모터마다 마지막 상태를 담는 작은 메모리 박스... 배선까지 나가지 않고 바로 읽는다" | `RobStrideBus._state` 캐시 + `states()` |
| "**토크를 켜는 순간, 먼저 각 관절의 현재 위치를 목표로 복사한다.** 안 그러면 0으로 확 당겨 튄다" | `SingleLeg.latch_target_from_state()` |
| "soft-stop은 강성을 0으로 두고 가벼운 감쇠만 남기는 것" | `SingleLeg.send_damping()` (`kp=0, kd=DAMPING_KD`) |
| "LeRobot은 degree로 계산하고 모터는 radian을 쓴다. 변환은 매 읽기/쓰기에서" | `codec/mit.py` — 내부 radian, 외부 degree |
| "스틱 기울기 × 최대속도 × 스텝시간을 **계속 커지는 위치 목표에 더한다**" | `SetpointRamp.advance()` — 같은 적분형 setpoint |
| "관절별 한계를 둬서 기구가 견딜 수 있는 범위를 넘지 않게" | `safety/limits.py` |

---

## 5. 유용한 수치와 팁

| 항목 | 내용 |
|---|---|
| **버스 여유** | 6모터 × 100 Hz ≈ 1 Mbps의 15% |
| **kd로 부드러움을 만든다** | "A higher kd is what gave me smooth motion, **right at the motor, without needing any filter in software**" — 소프트웨어 필터를 짜지 말고 kd를 올릴 것 |
| **CAN 중재** | ID가 낮을수록 버스 중재에서 이긴다. 우선순위가 필요한 관절에 낮은 ID를 배정할 수 있다 |
| **테스트 순서** | 모터를 하나씩 따로: 전원 → 버스 → 통신 → 응답 → 한계. 통합 전에 개별 검증 |
| **모터 크기 배분** | 어깨 쪽(토크 큼)에 큰 모델, 말단에 작은 모델. 우리도 hip/knee=RS02, 발목=RS00으로 같은 구조 |

---

## 6. 조치 목록 (우선순위)

| # | 할 일 | 이유 | 상태 |
|---|---|---|---|
| 1 | **모터의 프로토콜 모드 확인** | 틀리면 아무것도 안 움직이고 에러도 안 난다 | ❗ 미확인 |
| 2 | 확인 결과에 따라 경로 A(전환) 또는 B(`codec/private.py`) 결정 | | 미결정 |
| 3 | 텔레메트리를 브링업에 연결 → `loop_dt` 실측 | 측정 없이 최적화하지 않기 위해 | 미연결 |
| 4 | `loop_dt`가 실제로 안 나오면 RX 전용 스레드 도입(③) | 양다리 확장 대비 | 보류 |

**3번이 4번보다 앞인 것이 중요하다.** `loop_dt`를 재보지도 않고 RX 스레드를 넣는 것은
순서가 뒤바뀐 최적화다. 모터 6개면 현재 방식(②)으로 충분할 수도 있다.

### RX 스레드를 도입한다면 (4번, 아직 하지 말 것)

```
CanBus.start_rx_thread(handler)
    스레드가 recv()를 계속 돌며 handler(=bus._ingest) 호출

sync_write_mit(..., drain=False)
    루프에서 드레인 제거

sync_read_states()
    요청만 보내고 반환. 응답 처리는 스레드가 담당
```

`sync_write_mit`에 이미 `drain: bool = True` 인자가 있어 전환 지점은 준비되어 있다.
