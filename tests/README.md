# `tests/`

```
tests/
└── test_safety.py   safety/ 순수 함수 42개 (294줄)
```

## 실행

```bash
PYTHONPATH=src python3 -m pytest tests -q
```

```
..........................................            [100%]
42 passed in 0.01s
```

**`python-can` 없이 실행됨.** 순수 계층만 다루기 때문임. 하드웨어도, CAN 인터페이스
설정도 필요 없음.

특정 클래스만 실행:

```bash
PYTHONPATH=src python3 -m pytest tests -q -k TestApply
PYTHONPATH=src python3 -m pytest tests -v          # 이름까지 출력
```

---

## 구성

| 클래스 | 개수 | 대상 |
|---|---|---|
| `TestSafeWindow` | 3 | `limits.safe_window` |
| `TestClamp` | 5 | `limits.clamp` |
| `TestMarginToLimit` | 5 | `limits.margin_to_limit` |
| `TestClosestToLimit` | 3 | `limits.closest_to_limit` |
| `TestIsFinite` | 4 | `guards.is_finite` |
| `TestNanIsDangerous` | 3 | 유한값 검사의 근거 |
| `TestClampJump` | 5 | `guards.clamp_jump` |
| `TestApply` | 10 | `guards.apply` — 세 관문 전체 |
| `TestGuardCounters` | 4 | `guards.GuardCounters` |

한계 상수는 실제 무릎(m10) 값 `-20.65 ~ 74.79` 를 사용함. **비대칭이라** 좌우 구분이
필요한 검사에 적합함.

---

## 고정한 것

### `TestNanIsDangerous` — 유한값 검사의 근거

파이썬 `min`/`max`가 NaN을 통과시키는 것과, `limits.clamp`도 NaN을 못 잡는 것을
**직접 실행해 확인함.**

```python
def test_python_minmax_passes_nan_through(self):
    assert min(10, NAN) == 10

def test_clamp_does_not_catch_nan(self):
    value, clipped = limits.clamp(NAN, KNEE, margin_deg=3)
    assert math.isnan(value)
    assert clipped is False        # 잘렸다고 보고하지도 않음

def test_guards_catches_it(self):
    r = guards.apply(NAN, 0.0, ...)
    assert r.reject is guards.RejectReason.NOT_FINITE
```

**"그래서 `guards`가 따로 검사해야 한다"는 근거가 코드로 남음.** 나중에 이 검사를
지우려는 사람이 왜 있는지 알게 됨.

### `test_limit_applied_before_jump` — 클리핑 순서

목표 200, 현재 60, `max_delta` 50 에서 **순서에 따라 결과가 달라짐.**

```
한계 먼저 (현재 구현)
  clamp(200) -> 71.79
  clamp_jump(71.79, 60, 50) -> 차이 11.79 < 50 이므로 그대로
  결과 71.79, clips = (LIMIT,)          ← 점프는 걸리지 않음

점프 먼저 (잘못된 순서)
  clamp_jump(200, 60, 50) -> 110        ← 한계(74.79)를 넘은 값
  clamp(110) -> 71.79
  결과 71.79, clips = (JUMP, LIMIT)     ← 불필요한 점프 클리핑이 기록됨
```

값은 같지만 경로가 다름. 역순은 중간에 한계 밖 값을 만들고, 실제로는 걸릴 필요가
없는 점프 클리핑을 카운터에 남김. `clips` 튜플로 순서를 고정함.

### `test_output_may_be_outside_limits_while_recovering`

현재가 이미 한계 밖(200)이면 **한 번에 복귀하지 않고** `max_delta`씩 돌아옴.

```python
r = guards.apply(0, 200, ...)
assert r.value == pytest.approx(150.0)
assert r.value > KNEE[1]          # 아직 한계 밖
```

의도된 동작임 — 한 번에 뛰면 위험함. 나중에 "왜 한계 밖 값이 나가지?"라며 고치려는
것을 막음.

### `test_reaches_far_target_over_cycles`

버리지 않고 자르므로 먼 목표에도 결국 도달함.

```python
cur = 0.0
for _ in range(3):
    cur, _ = guards.clamp_jump(100, cur, 50)
assert cur == pytest.approx(100.0)
```

버리는 방식이면 도달 불가. **클리핑 = 속도 제한**임을 고정함.

### `test_all_keys_always_present`

`as_fields()`가 0이어도 모든 키를 출력하는지.

```python
expected = {"clips", "rejects",
            "clips_limit", "clips_jump",
            "rejects_nan", "rejects_nostate"}
assert set(guards.GuardCounters().as_fields()) == expected
```

필드가 나타났다 사라지면 PlotJuggler 레이아웃과 CSV 헤더가 깨짐.

---

## 미커버

| 대상 | 사유 |
|---|---|
| `motors/` | 아직 작성 전 (2단계) |
| `kinematics/` | 아직 작성 전 (6단계) |
| 그 외 계층 | 아직 작성 전 |

작성 순서는 [docs/build_from_scratch.md](../docs/build_from_scratch.md) 참조.
각 단계마다 해당 계층의 테스트를 여기에 추가함.
