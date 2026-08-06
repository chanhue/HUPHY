"""벤더 중립 자료형 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q      # python-can 불필요

실제 값으로 씀. 무릎(모터 10, RS02)의 하드스톱은 cal 공간 (-20.65, 74.79) 이고,
발목은 두 모터(11=a1 RS00, 12=a2 RS00)가 하나의 관절을 움직임.
"""

from typing import Dict, Iterable, List, Optional, Tuple

import pytest

from huphy.motors.base import (
    Gains,
    Motor,
    MotorCalibration,
    MotorFault,
    MotorState,
    MotorsBus,
    resolve_motor_list,
)

KNEE_LIMITS = (-20.65, 74.79)


# ===========================================================================
# Gains
# ===========================================================================
class TestGains:
    def test_default_is_no_torque(self):
        """기본값 0은 토크 없음임.

        실측 전에 움직이지 않도록 일부러 0으로 둠. 기본값을 "적당한 값"으로 두면
        캘리브레이션을 건너뛴 채로 다리가 힘을 씀.
        """
        g = Gains()
        assert g.kp == 0.0
        assert g.kd == 0.0

    def test_scaled_lowers_both(self):
        """브링업 초반에 전체를 한꺼번에 낮춤. kp만 낮추면 감쇠비가 달라져 발진함."""
        g = Gains(kp=30.0, kd=1.0).scaled(0.1)
        assert g.kp == pytest.approx(3.0)
        assert g.kd == pytest.approx(0.1)

    def test_scaled_returns_new_object(self):
        """원본을 바꾸지 않음. 설정에서 읽은 값이 축소 한 번으로 영구히 깎이면 안 됨."""
        base = Gains(kp=30.0, kd=1.0)
        base.scaled(0.5)
        assert base.kp == 30.0

    def test_is_frozen(self):
        with pytest.raises(Exception):
            Gains(kp=1.0).kp = 2.0


# ===========================================================================
# Motor — 사람이 적는 것
# ===========================================================================
class TestMotor:
    def test_holds_authored_values(self):
        m = Motor(id=10, model="RS02", limits_deg=KNEE_LIMITS, gains=Gains(kp=30.0, kd=1.0))
        assert m.id == 10
        assert m.model == "RS02"
        assert m.limits_deg == KNEE_LIMITS

    def test_model_is_a_plain_string(self):
        """이 계층은 벤더를 모름. 유효한 모델인지는 벤더 모듈이 판단함.

        여기서 enum으로 좁히면 벤더를 추가할 때마다 중립 계층을 고쳐야 함.
        """
        assert Motor(id=1, model="아무거나").model == "아무거나"

    def test_rejects_reversed_limits(self):
        """cal 공간에는 sign이 개입하지 않아 순서가 뒤집히지 않음.

        따라서 lo > hi 는 오타이지 정상 입력이 아님. 변환할 때마다 min/max로
        재정렬하는 대신 입력 시점에 막음.
        """
        with pytest.raises(ValueError, match="lo 가 hi 보다"):
            Motor(id=10, model="RS02", limits_deg=(74.79, -20.65))

    def test_rejects_zero_width_limits(self):
        """lo == hi 면 통과할 구간이 없음."""
        with pytest.raises(ValueError, match="lo 가 hi 보다"):
            Motor(id=10, model="RS02", limits_deg=(30.0, 30.0))

    def test_negative_range_is_fine(self):
        """양쪽이 다 음수여도 lo < hi 면 정상임. hipz가 그런 관절임."""
        m = Motor(id=7, model="RS02", limits_deg=(-117.07, -21.07))
        assert m.limits_deg == (-117.07, -21.07)

    def test_unknown_limits_is_not_unlimited(self):
        """None은 "아직 모름"이고 "제한 없음"이 아님.

        한계를 모르는 채로 토크를 넣는 것이 가장 위험하므로 미완성으로 판정함.
        """
        assert Motor(id=1, model="RS02").limits_deg is None
        assert Motor(id=1, model="RS02").is_configured is False

    def test_limits_alone_is_not_enough(self):
        """게인이 0이면 토크가 안 나가므로 제어에 쓸 수 없음."""
        m = Motor(id=10, model="RS02", limits_deg=KNEE_LIMITS)
        assert m.is_configured is False

    def test_fully_configured(self):
        m = Motor(id=10, model="RS02", limits_deg=KNEE_LIMITS, gains=Gains(kp=30.0, kd=1.0))
        assert m.is_configured is True

    def test_is_frozen(self):
        """설정은 읽고 나면 바뀌지 않음. 제어 중 누가 한계를 넓히면 안 됨."""
        m = Motor(id=10, model="RS02", limits_deg=KNEE_LIMITS)
        with pytest.raises(Exception):
            m.id = 99


# ===========================================================================
# MotorCalibration — raw 와 cal
# ===========================================================================
class TestMotorCalibration:
    def test_identity_when_unmeasured(self):
        """sign=1, offset=0 이면 두 공간이 같은 숫자임.

        지금 12개 모터가 전부 이 상태라, 한계값을 raw로 보든 cal로 보든 동일하게
        동작함. 실측값을 넣는 순간 갈라짐 (이슈 #2).
        """
        c = MotorCalibration(motor_id=10)
        for raw in (-50.0, 0.0, 33.0):
            assert c.raw_to_cal(raw) == raw

    def test_offset_shifts_the_zero(self):
        """무릎을 12도 굽은 자세에서 영점 잡은 경우.

        편 상태가 raw -12 이고, 그걸 cal 0 으로 부르고 싶으므로 offset = +12.
        """
        c = MotorCalibration(motor_id=10, offset_deg=12.0)
        assert c.raw_to_cal(-12.0) == pytest.approx(0.0)      # 편 상태
        assert c.raw_to_cal(0.0) == pytest.approx(12.0)       # 영점 잡은 자세
        assert c.raw_to_cal(62.79) == pytest.approx(74.79)    # 하드스톱

    def test_hard_stop_moves_in_raw_but_not_in_cal(self):
        """하드스톱은 쇳덩어리라 안 움직이는데 raw 숫자만 바뀜.

        영점을 3도 다른 자세에서 다시 잡으면 raw 는 달라지고 cal 은 그대로임.
        이슈 #2에서 한계를 cal 공간에 둔 이유임.
        """
        before = MotorCalibration(motor_id=10, offset_deg=12.0)
        after = MotorCalibration(motor_id=10, offset_deg=15.0)   # 3도 밀림

        hard_stop_cal = 74.79
        assert before.cal_to_raw(hard_stop_cal) == pytest.approx(62.79)
        assert after.cal_to_raw(hard_stop_cal) == pytest.approx(59.79)

    def test_mirrored_legs_share_one_cal_number(self):
        """양다리는 거울상이라 같은 굽힘에 raw 부호가 반대임.

        cal 로 말하면 양쪽이 같은 숫자가 됨. 보행 궤적이 "무릎 45도" 라고 하면
        양다리가 같은 동작을 함 -- cal 공간이 존재하는 이유임.
        """
        right = MotorCalibration(motor_id=10, sign=1.0)
        left = MotorCalibration(motor_id=4, sign=-1.0)

        assert right.raw_to_cal(45.0) == pytest.approx(45.0)
        assert left.raw_to_cal(-45.0) == pytest.approx(45.0)

    def test_sign_does_not_reorder_cal_limits(self):
        """sign=-1 이어도 cal 공간의 lo < hi 는 유지됨.

        한계를 raw 에 두면 변환 후 순서가 뒤집혀 min/max 재정렬이 필요했음.
        cal 에 두면 sign 이 개입하지 않아 그 처리가 통째로 없어짐.
        """
        c = MotorCalibration(motor_id=4, sign=-1.0)
        lo_cal, hi_cal = KNEE_LIMITS
        assert lo_cal < hi_cal
        # raw 로 내리면 순서가 뒤집힘 -- 그래서 raw 를 저장 공간으로 쓰지 않음
        assert c.cal_to_raw(lo_cal) > c.cal_to_raw(hi_cal)

    @pytest.mark.parametrize("sign,offset", [(1.0, 0.0), (-1.0, 12.0), (1.0, -30.0)])
    def test_round_trip(self, sign, offset):
        c = MotorCalibration(motor_id=10, sign=sign, offset_deg=offset)
        for cal in (-20.65, 0.0, 45.0, 74.79):
            assert c.raw_to_cal(c.cal_to_raw(cal)) == pytest.approx(cal)
        for raw in (-62.79, 0.0, 33.0):
            assert c.cal_to_raw(c.raw_to_cal(raw)) == pytest.approx(raw)

    def test_sign_zero_is_rejected(self):
        """sign=0 이면 모든 raw 가 같은 cal 로 뭉개져 역변환이 불가능함.

        캘리브레이션 파일이 채워지지 않았을 때 나올 수 있는 값이라 조용히 넘기지 않음.
        """
        c = MotorCalibration(motor_id=10, sign=0.0)
        with pytest.raises(ValueError, match="sign이 0"):
            c.cal_to_raw(45.0)

    def test_has_no_limits(self):
        """한계는 여기 없음. 재는 값이 아니라 적는 값이라 Motor 에 있음 (이슈 #2)."""
        assert not hasattr(MotorCalibration(motor_id=10), "limits")

    def test_has_no_gains(self):
        """게인은 재는 값이 아니라 맞추는 값이라 Motor 에 있음.

        모터를 다시 달면 offset 은 무효지만 게인은 그대로 쓸 수 있음 -- 무효화
        시점이 다르므로 한 파일에 두지 않음.
        """
        c = MotorCalibration(motor_id=10)
        assert not hasattr(c, "kp")

    def test_zero_reference_is_a_note(self):
        """0xFE 는 모터가 저장하지만 "그때 어떤 자세였는지" 는 어디에도 안 남음."""
        c = MotorCalibration(motor_id=10, zero_reference="다리 편 상태, 발바닥 평면 접촉")
        assert "편 상태" in c.zero_reference


# ===========================================================================
# MotorState — 신선도
# ===========================================================================
class TestMotorState:
    def test_never_received(self):
        s = MotorState()
        assert s.is_valid is False
        assert s.age(100.0) == float("inf")
        assert s.is_fresh(100.0, max_age_s=1.0) is False

    def test_age(self):
        s = MotorState(stamp=100.0)
        assert s.age(100.5) == pytest.approx(0.5)

    def test_stale_state_fails_freshness(self):
        """점프 가드는 "지금 위치" 를 기준으로 자름.

        그 기준이 몇 주기 전 값이면 실제로는 큰 점프를 통과시킴. 100Hz 루프에서
        10ms 를 넘으면 이미 한 주기를 놓친 것임.
        """
        s = MotorState(stamp=100.0)
        assert s.is_fresh(100.005, max_age_s=0.01) is True
        assert s.is_fresh(100.050, max_age_s=0.01) is False

    def test_age_never_negative(self):
        """시계가 뒤로 가도 음수가 나오지 않음.

        stamp 에 monotonic 을 쓰는 이유이기도 함 -- 벽시계는 NTP 보정에 뒤로 감.
        """
        s = MotorState(stamp=100.0)
        assert s.age(99.0) == 0.0

    def test_position_is_raw_space(self):
        """버스가 채우는 값이라 raw 임. cal 변환은 robots/ 가 함."""
        s = MotorState(position_deg=62.79, stamp=1.0)
        cal = MotorCalibration(motor_id=10, offset_deg=12.0).raw_to_cal(s.position_deg)
        assert cal == pytest.approx(74.79)


# ===========================================================================
# MotorFault
# ===========================================================================
class TestMotorFault:
    def test_zero_is_ok(self):
        f = MotorFault()
        assert f.ok is True
        assert f.active() == []

    def test_multiple_bits_can_be_set(self):
        """비트 필드라 여러 고장이 동시에 섬. 과열이 오면 스톨도 같이 오기 쉬움."""
        f = MotorFault(raw=0b101, bits={"overtemperature": True, "driver_ic": False,
                                        "undervoltage": True})
        assert f.ok is False
        assert set(f.active()) == {"overtemperature", "undervoltage"}

    def test_raw_decides_ok_not_bits(self):
        """해석표에 없는 비트가 서도 정상으로 보고하지 않음.

        비트 이름은 벤더마다 다르고 표가 불완전할 수 있으므로, 판정은 원본 워드가 함.
        """
        f = MotorFault(raw=1 << 20, bits={})
        assert f.ok is False


# ===========================================================================
# resolve_motor_list
# ===========================================================================
class TestResolveMotorList:
    LEG = (7, 8, 9, 10, 11, 12)

    def test_none_means_all(self):
        assert resolve_motor_list(None, self.LEG) == list(self.LEG)

    def test_subset_keeps_given_order(self):
        """호출부가 정한 순서를 지킴. 발목 두 모터는 보내는 순서가 의미를 가짐."""
        assert resolve_motor_list([11, 12], self.LEG) == [11, 12]
        assert resolve_motor_list([12, 11], self.LEG) == [12, 11]

    def test_unknown_id_raises(self):
        """조용히 무시하면 "명령을 보냈는데 안 움직인다" 가 되고,
        원인이 오타인지 배선인지 구분되지 않음.
        """
        with pytest.raises(ValueError, match=r"없는 모터 id: \[99\]"):
            resolve_motor_list([10, 99], self.LEG)

    def test_error_lists_available_ids(self):
        """다른 다리의 id를 넣은 실수를 바로 알아보게 함."""
        with pytest.raises(ValueError, match=r"가용: \[7, 8, 9, 10, 11, 12\]"):
            resolve_motor_list([4], self.LEG)

    def test_empty_list_means_nothing(self):
        """빈 리스트는 전체가 아님. None 과 구분됨."""
        assert resolve_motor_list([], self.LEG) == []


# ===========================================================================
# MotorsBus — 계약
# ===========================================================================
class FakeBus(MotorsBus):
    """ABC 계약만 채운 최소 구현. 하드웨어 없이 수명 동작을 확인하는 용도임."""

    def __init__(self, ids=(10,)):
        self._ids = tuple(ids)
        self._connected = False
        self.log: List[str] = []

    @property
    def motor_ids(self) -> Tuple[int, ...]:
        return self._ids

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        self._connected = True
        self.log.append("connect")

    def disconnect(self) -> None:
        self._connected = False
        self.log.append("disconnect")

    def enable_torque(self, motors: Optional[Iterable[int]] = None) -> None:
        self.log.append(f"enable{resolve_motor_list(motors, self._ids)}")

    def disable_torque(self, motors: Optional[Iterable[int]] = None) -> None:
        self.log.append("disable")

    def refresh_states(self, motors: Optional[Iterable[int]] = None) -> List[int]:
        return []

    def state(self, motor_id: int) -> MotorState:
        return MotorState()

    def states(self) -> Dict[int, MotorState]:
        return {}


class TestMotorsBusContract:
    def test_cannot_instantiate_abstract(self):
        """구현을 빠뜨린 채로 버스를 만들 수 없음."""
        with pytest.raises(TypeError):
            MotorsBus()

    def test_context_manager_connects_and_disconnects(self):
        bus = FakeBus()
        with bus as b:
            assert b is bus
            assert bus.is_connected is True
        assert bus.is_connected is False

    def test_disconnects_even_on_exception(self):
        """제어 중 예외가 나도 토크가 끊겨야 함 (이슈 #6).

        이게 없으면 모터가 마지막 명령을 계속 유지함 -- 사람이 전원을 뽑을 때까지
        다리가 힘을 주고 있음.
        """
        bus = FakeBus()
        with pytest.raises(RuntimeError):
            with bus:
                raise RuntimeError("제어 루프에서 터진 예외")
        assert bus.is_connected is False
        assert bus.log == ["connect", "disconnect"]

    def test_resolve_is_usable_from_implementations(self):
        bus = FakeBus(ids=(7, 8, 9))
        bus.enable_torque()
        bus.enable_torque([8])
        assert bus.log == ["enable[7, 8, 9]", "enable[8]"]
