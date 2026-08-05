# 옵션3 모터 제어 분석 — 각 항의 의미와 "왕복이 한 방향에서 멈추는" 이유

`single_leg_controller.py`의 대화형 메뉴 **옵션 3(Control motor, 범위 왕복)** 이
왜 한 방향(중력 거스르는 방향)에서 목표 직전에 멈추고 되돌아오지 못하는지를,
관련 코드의 각 항 의미부터 구조까지 정리한다.

---

## 1. 관련 코드 경로

옵션 3을 누르면 도는 흐름:

```
_menu_control            # 왕복 루프 (hi ↔ lo 목표 번갈아)
  └ _step_motor_toward_raw   # 한 스텝: 실측 읽고 목표 쪽으로 조금 이동
       ├ _request_state       # 현재 각도 읽기 (0xFB 핑)
       ├ _wrap_near_center    # 목표를 현재에서 최단 표현으로
       └ _prepare_mit_payload # kp/kd + 안전검사 + MIT 프레임 조립
            ├ _resolve_raw_target_near_current
            └ pack_mit_command # 8바이트 인코딩
       └ _send8               # CAN 전송
```

핵심 두 함수:

```python
# _step_motor_toward_raw  (한 스텝 이동)
cur  = state[mid].position_deg                       # ① 실측 현재각
goal = _wrap_near_center(target_raw, cur)            # ② 최단 목표
if abs(goal - cur) < 1.0: return True                # ③ 도달 판정(1° 이내)
nxt  = cur + max(-step_deg, min(step_deg, goal-cur)) # ④ 명령 = 실측 + 최대 ±step
payload = _prepare_mit_payload(mid, MotorCommand(position_deg=nxt), clamp=True)
_send8(mid, payload)
return False
```

```python
# _menu_control  (왕복 루프)
targets = [hi_raw, lo_raw]; idx = 0
while 실행중:
    while 실행중:
        if _step_motor_toward_raw(mid, targets[idx], step_deg):  # 도달해야
            break                                                #  break
        sleep(0.02)
    idx = 1 - idx        # ← 도달 후에만 방향 전환
```

---

## 2. 각 항의 의미

### 2.1 실제 제어 법칙 (모터 펌웨어 내부, MIT 위치 PD)

파이썬은 `(위치, 속도, kp, kd, 토크)`만 보내고, 실제 PD 계산은 **모터가** 한다:

```
τ = kp · (pos_cmd − pos_meas) + kd · (vel_cmd − vel_meas) + τ_ff
```

| 항 | 의미 | 이 코드에서 |
|---|---|---|
| `pos_cmd` | 명령 위치 | `nxt` (= 실측 + step) |
| `pos_meas` | 실측 위치 | 모터 내부 엔코더 |
| `vel_cmd` | 명령 속도(피드포워드) | **0** (속도 FF 안 씀) |
| `vel_meas` | 실측 속도 | 모터 내부 |
| `kp` | 위치 강성 (Nm/rad) | `self.gains[mid]` (= `DEFAULT_GAINS`) |
| `kd` | 감쇠 (Nm·s/rad) | `self.gains[mid]` |
| `τ_ff` | 토크 피드포워드 | **0** |

즉 실질적으로:
```
τ = kp · (nxt − pos_meas) − kd · vel_meas
```

### 2.2 코드 항들

| 코드 | 의미 |
|---|---|
| `cur` | 이번 사이클에 **다시 읽은** 실측 각도 |
| `goal` | 목표각을 현재 근처 최단 표현으로 wrap한 값 |
| `step_deg` | 한 사이클에 명령을 움직일 **최대 폭** (옵션3 기본 2°) |
| `nxt` | 이번에 보낼 명령 위치 = **`cur + clamp(goal−cur, ±step_deg)`** |
| `abs(goal−cur) < 1.0` | **도달 판정** — 1° 이내면 "도착"으로 보고 방향 전환 트리거 |
| `_wrap_near_center` | 360° 모호성 제거(최단경로 목표 선택) |
| `_resolve_raw_target_near_current` | 전송 직전 다시 최단 표현 보정 |
| `_prepare_mit_payload(clamp=True)` | 한계 클램프·점프가드 통과 후 프레임 조립 |
| `targets=[hi,lo]`, `idx` | 왕복 목표쌍과 현재 목표 인덱스 |
| `idx = 1 − idx` | **도달했을 때만** 반대 목표로 전환 |

---

## 3. 핵심 구조: "명령 = 실측 + step" (재앵커)

이 컨트롤러의 궤적 생성은 매 사이클 **실측에 다시 붙여서** 명령을 만든다:

```
nxt = cur + Δ        (Δ = clamp(goal−cur, ±step_deg),  |Δ| ≤ step_deg)
```

그래서 모터가 보는 **위치 오차**는:

```
오차 = pos_cmd − pos_meas = nxt − cur = Δ   ≤  step_deg
```

→ **오차가 구조적으로 `step_deg`(2°)를 절대 넘을 수 없다.** ("오차 갇힘")

따라서 낼 수 있는 최대 토크도 상한이 걸린다:

```
τ_max = kp · step_deg(rad) − kd·vel
      = kp · radians(2°)
      = kp · 0.0349
```

| kp | τ_max (근사) |
|---|---|
| 1 | 0.035 Nm |
| 8 | 0.28 Nm |
| 57 | 2.0 Nm |
| 114 | 4.0 Nm |

즉 **kp를 올려도 오차가 2°로 갇혀 있어 토크 상한이 `kp × 2°`로 고정**된다.

---

## 4. 지금 상황: 왕복이 한 방향(중력 거스름)에서 멈추는 이유

### 4.1 중력 모멘트는 각도에 따라 변한다

관절이 중력을 거슬러 올라갈수록 **필요 토크(중력 모멘트)가 커진다** (moment arm 증가).
목표(hi) 근처가 보통 **필요 토크 최대** 지점이다.

```
필요 토크(θ) = m·g·L·f(θ)   ← θ가 커질수록(거스름) 증가
가용 토크     = kp · step_deg = 2 Nm (고정 상한)
```

### 4.2 목표 직전에서 멈춘다 → 도달 실패 → 전환 안 됨

올라가다가 어느 각도에서 **필요 토크 = 가용 상한(2Nm)** 이 되면 그 지점에서 정지한다.
문제는 그게 hi(71.79°) **코앞**(예: 70.x°)이라는 것:

```
멈춘 위치 70.x°  →  |goal − cur| = |71.79 − 70.x| > 1.0
→ _step_motor_toward_raw 가 True(도달)를 반환하지 못함
→ 안쪽 while 이 break 되지 않음
→ idx = 1 − idx (방향 전환)이 실행되지 않음
→ 계속 '올라가는' 목표만 보내며 그 자리에 정지
```

즉 **"내려오는 게 막힌" 것이 아니라, 올라가기를 끝내지 못해 내려올 차례가 오지 않는 것.**
사용자 눈에는 "한 방향으로 갔다가 안 돌아온다"로 보인다.

### 4.3 왜 실측이 안 변하면 영원히 갇히나

```
멈춤(실측 정지) → 다음 사이클 cur 그대로 → nxt = cur + 2 (동일)
→ 오차 2°, 토크 2Nm (동일) → 여전히 부족 → 안 움직임 → 반복
```
자기 자신을 벗어날 수 없는 루프(self-locking).

### 4.4 수치 예 (kp=57, step=2°)

```
가정: hi 근처에서 관절을 마저 올리는 데 3 Nm 필요
가용: 57 × radians(2°) = 2.0 Nm  <  3 Nm
→ hi 직전에서 정지, 도달(1° 이내) 실패, 방향 전환 없음
```

필요 3 Nm를 이 스킴에서 내려면 `kp ≥ 3 / 0.0349 ≈ 86`. 하지만 그러면 쉬운 구간에서
`kp × 큰 순간오차`로 **과격/발진** 위험 → 게인으로 때우는 건 미봉책.

---

## 5. move_motor10 과 비교 (왜 저쪽은 되나)

`move_motor10.py`는 명령을 **실측에 재앵커하지 않고 절대 궤적**으로 만든다:

```python
pos = start + (target − start) · frac   # 절대 setpoint (실측 무관)
```

모터가 뒤처져도 setpoint가 계속 전진하므로 **오차가 누적**된다:

```
오차 = pos − pos_meas → 점점 커짐 → τ = kp·오차 도 커짐 → 필요한 만큼 나옴
```

| | 궤적 생성 | 오차 상한 | 목표 도달 |
|---|---|---|---|
| **move_motor10** | `start + 램프` (절대) | 없음(누적) | ✅ 끝까지 |
| **single_leg 옵션3** | `실측 + step` (재앵커) | **±step_deg** | ❌ 힘든 구간에서 정지 |

같은 kp라도 single_leg는 오차를 묶어 토크를 스스로 제한한다.

---

## 6. 표준 제어 방식

표준 위치 제어는 **기준 궤적 추종(reference following)**:

```
[궤적 생성기]  현재→목표 부드러운 기준 r(t)  (시작 시 실측 1회만 사용)
      │
[피드백 PD]   오차 = r(t) − 실측  →  τ = kp·오차 + kd·(ṙ − 실측속도)
      │
[모터]        이동
```

원칙:
1. 기준은 **계획(시간)** 으로 진행 — 매 순간 실측에 재앵커하지 않음.
2. 실측은 **오차 항으로만** 들어감 → 뒤처지면 오차가 커져 토크가 자연히 나옴.
3. 게인은 **고정**, 안전(급점프·한계)은 **속도/가속 제한 + 클램프**로 별도 처리.

single_leg의 "실측 + step"은 이 표준이 아니라 "실측을 뒤쫓는 슬루-레이트 setpoint"에
가깝고, 그 부작용이 **오차 갇힘 → 토크 상한**이다.

---

## 7. 해결 방안

옵션3의 **궤적 생성만** 표준(절대 setpoint 램프)으로 바꾸면 된다. 안전장치·다른 함수는 유지.

**핵심 변경**: 명령 기준을 "실측"이 아니라 **"직전 명령(setpoint)"** 으로.

```python
# 변경 전 (재앵커):   nxt = cur(실측) + clamp(goal − cur, ±step)
# 변경 후 (절대):     sp  = sp        + clamp(goal − sp,  ±step)   # 직전 setpoint에서 진행
#                     (시작 시 sp = 실측으로 1회 초기화, 점프 방지)
```

효과:
- 모터가 뒤처지면 `sp − 실측` 오차가 쌓여 **hi 근처 최대 모멘트도 넘길 토크**가 나옴 → 끝까지 도달.
- 도달하니 방향 전환 → **왕복 정상**.
- 게인은 정상값(kp 20~40)으로 충분.

유지할 것:
- `_prepare_mit_payload(clamp=True)` 의 한계 클램프·점프가드·E-STOP
- `step_deg`(속도 제한 역할) — 절대 setpoint의 전진 폭을 제한해 급가속 방지

---

## 8. 한 줄 요약

- **원인**: 옵션3이 명령을 `실측 + step`로 만들어 **오차가 step(2°)에 갇힘** → 토크 상한 `kp·step` → 중력 거스르는 구간에서 목표 직전 정지.
- **증상**: 목표(hi) 도달 실패 → 방향 전환(`idx=1−idx`)이 안 일어남 → "한 방향 가고 안 돌아옴".
- **정답**: 궤적을 **절대 setpoint 램프**(표준)로 바꿔 오차가 쌓이게 → 도달 → 왕복 정상. 게인으로 때우지 말 것.
