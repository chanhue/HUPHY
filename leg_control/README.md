# leg_control — 단일 다리 모터 제어 (레거시)

> ⚠️ **이 폴더는 참조용이다.** 현재 개발은 [`src/huphy/`](../src/huphy/)에서
> 이루어진다. 여기 코드는 실물 하드웨어 검증 전까지 대조용으로 남겨둔 것이며,
> 아래에 적힌 알려진 이슈들이 그대로 남아 있다.

기존 프로젝트에서 **다리 제어 부분만** 분리한 자립 패키지. 인버스펜듈럼 쪽
코드(`commission_motor.py`, `mjlab.*` 등)에 의존하지 않는다.

## 구조

```
leg_control/
├── single_leg_controller.py   # 메인: 한 다리(6모터) 제어 + 대화형 브링업 메뉴
├── robot_constant.py          # 하드웨어 상수 (모터ID·부호·한계·게인·안전마진)
├── ankle_kinematics.py        # 발목 2모터 ↔ pitch/roll IK/FK
└── utils/
    ├── __init__.py
    └── mit_codec.py           # MIT 프레임 인코딩/디코딩 (pack/decode)
```

문서는 저장소 최상단 [`docs/`](../docs/)로 옮겼다.

## 의존성

- Python 3.9+
- `python-can` (CAN 통신)
- `numpy`

```bash
pip install python-can numpy
```

## 모듈 관계

```
single_leg_controller.py
   ├─ import utils.mit_codec        (pack_mit_command / decode_state_frame / MotorState)
   ├─ import robot_constant         (상수·캘리브레이션)
   └─ import ankle_kinematics       (발목 IK/FK)
```

## 실행 (대화형 브링업 메뉴)

```bash
cd leg_control
python3 single_leg_controller.py
```
메뉴:
- **1) Set zero** — 모터 번호 입력 → 그 모터의 기계 영점(MIT 0xFE) 설정 (전원 후 유지)
- **2) Go to zero** — 전체 모터를 0으로 이동
- **3) Control motor** — 모터 번호 입력 → 범위 왕복 (`q`로 중지)
- **0) Quit**

라이브러리로 쓸 때:
```python
from single_leg_controller import SingleLegController
leg = SingleLegController(side="right", allow_uncalibrated=True)
leg.verify_and_start(auto_enable=True)
leg.set_leg_action(knee=30, ankle_pitch=10)   # 관절 공간(deg) 목표
q = leg.get_joint_state()
leg.stop()
```

## ⚠️ 캘리브레이션 (실물 전 필수)

`robot_constant.py`의 `CALIBRATED = False`. `MOTOR_SIGN / MOTOR_OFFSET_DEG /
JOINT_LIMITS_DEG / DEFAULT_GAINS`, 발목 기하값(`ankle_kinematics.py`)은
**플레이스홀더**다. 실물에 토크 걸기 전 반드시 실측값으로 교체하고
`CALIBRATED = True`로 바꿀 것. (벤치/시뮬레이터 테스트는 `allow_uncalibrated=True`)

## 알려진 이슈 (docs 참고)

- **옵션3 왕복이 한 방향에서 멈춤**: `_step_motor_toward_raw`가 명령을
  `실측 + step`으로 만들어 **오차가 step(2°)에 갇혀** 토크 상한이 `kp·step`으로
  묶임 → 중력 거스르는 구간에서 목표 직전 정지 → 방향 전환 안 됨.
  원인·표준 해결(절대 setpoint 램프)은 [docs/option3_control_analysis.md](docs/option3_control_analysis.md) 참고.

## 분리 시 원본과의 차이

- `utils/mit_codec.py`를 이 패키지 안에 포함하고, `single_leg_controller.py`의
  import 경로를 이 폴더 기준(`from utils.mit_codec`)으로 조정.
- 인버스펜듈럼 의존 스크립트(`move_motor10.py`, `test_zero_persistence.py`,
  `commission_motor.py`)는 **포함하지 않음** — 필요하면 mit_codec 기반으로 포팅.
