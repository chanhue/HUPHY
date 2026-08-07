"""커미셔닝 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

여기 있는 조작은 전부 되돌리기 어려움. 그래서 확인하는 것 대부분이 **거부 조건**임 —
어떤 상황에서 나가면 안 되는지를 고정함.

가짜 버스는 명령을 받으면 응답함. 하지만 **명령이 실제로 반영됐는지는 재현하지
않음** — 실물에서도 상태 프레임만으로는 알 수 없기 때문임.
"""

import math
import sys
import types
from collections import deque

import pytest

from huphy.motors.base import Motor
from huphy.motors.canbus import CanBus
from huphy.motors.robstride import commissioning as C
from huphy.motors.robstride import tables as T
from huphy.motors.robstride.bus import RobStrideBus
from huphy.motors.robstride.codec import mit

LEG = {
    10: Motor(id=10, model="RS02", limits_deg=(-20.65, 74.79)),
    11: Motor(id=11, model="RS00", limits_deg=(-79.77, 43.16)),
}
MODELS_BY_ID = {10: T.Model.RS02, 11: T.Model.RS00}

FAST = {"steps": 2, "hz": 100000.0}
"""테스트에서 nudge 를 기다리지 않기 위한 값. 동작은 같고 간격만 짧음."""


# ===========================================================================
# 가짜 python-can
# ===========================================================================
class FakeMessage:
    def __init__(self, arbitration_id, data, is_extended_id=False):
        self.arbitration_id = arbitration_id
        self.data = data
        self.is_extended_id = is_extended_id


class FakeCanBus:
    instances = []

    def __init__(self, **kwargs):
        self.sent = []
        self.rx = deque()
        self.responses = {}
        FakeCanBus.instances.append(self)

    def send(self, msg):
        self.sent.append(msg)
        reply = self.responses.get(msg.arbitration_id)
        if reply is not None:
            self.rx.append(reply)

    def recv(self, timeout=None):
        return self.rx.popleft() if self.rx else None

    def shutdown(self):
        pass


def state_frame(motor_id, pos_deg=0.0, *, model=T.Model.RS02):
    enc = T.encoding_for(model)
    pos = mit.float_to_uint(math.radians(pos_deg), -enc.pmax_rad, enc.pmax_rad, 16)
    vel = mit.float_to_uint(0.0, -enc.vmax_rad_s, enc.vmax_rad_s, 12)
    tau = mit.float_to_uint(0.0, -enc.tmax_nm, enc.tmax_nm, 12)
    return FakeMessage(motor_id, bytes([
        motor_id,
        (pos >> 8) & 0xFF, pos & 0xFF,
        (vel >> 4) & 0xFF,
        ((vel & 0x0F) << 4) | ((tau >> 8) & 0x0F),
        tau & 0xFF,
        0x01, 0x2C,
    ]))


@pytest.fixture
def fake_can(monkeypatch):
    FakeCanBus.instances = []
    mod = types.ModuleType("can")
    mod.Message = FakeMessage
    mod.interface = types.SimpleNamespace(Bus=FakeCanBus)
    monkeypatch.setitem(sys.modules, "can", mod)
    return mod


@pytest.fixture
def leg(fake_can):
    b = RobStrideBus(CanBus("can1"), LEG)
    b.connect()
    return b


@pytest.fixture
def raw(leg):
    r = FakeCanBus.instances[-1]
    r.sent.clear()
    return r


@pytest.fixture
def live(leg, raw):
    """모든 모터가 응답하는 상태."""
    raw.responses = {10: state_frame(10), 11: state_frame(11, model=T.Model.RS00)}
    raw.sent.clear()
    return raw


# ===========================================================================
# 제어 모드
# ===========================================================================
class TestSetControlMode:
    def test_frame_carries_mode_in_f_cmd(self, leg, live):
        C.set_control_mode(leg, 10, T.ControlMode.MIT)
        msg = live.sent[-1]
        assert msg.arbitration_id == 10
        assert msg.data[7] == T.CMD_SET_MODE
        assert msg.data[6] == int(T.ControlMode.MIT)

    def test_position_mode_differs(self, leg, live):
        C.set_control_mode(leg, 10, T.ControlMode.POSITION)
        assert live.sent[-1].data[6] == int(T.ControlMode.POSITION)

    def test_no_answer_raises(self, leg, raw):
        """응답이 없으면 반영됐는지 알 수 없음. 조용히 넘기지 않음."""
        with pytest.raises(C.CommissioningError, match="응답이 없음"):
            C.set_control_mode(leg, 10, T.ControlMode.MIT)


# ===========================================================================
# 기계 영점
# ===========================================================================
class TestSetZero:
    def test_sends_command_4(self, leg, live):
        C.set_zero(leg, 10, zero_reference="다리 편 상태, 발바닥 평면 접촉")
        msg = live.sent[-1]
        assert msg.data[7] == T.CMD_SET_ZERO
        assert msg.data[6] == T.F_CMD_DEFAULT

    def test_requires_a_reference_note(self, leg, live):
        """모터는 영점 값을 저장하지만 "그때 어떤 자세였는지" 는 어디에도 안 남음.

        메모가 없으면 영점을 재현할 수 없고, 재현할 수 없으면 offset 실측이
        무의미해짐.
        """
        with pytest.raises(ValueError, match="zero_reference 가 비어 있음"):
            C.set_zero(leg, 10, zero_reference="")

    def test_whitespace_is_not_a_note(self, leg, live):
        with pytest.raises(ValueError, match="zero_reference"):
            C.set_zero(leg, 10, zero_reference="   ")

    def test_refuses_while_torque_is_on(self, leg, live):
        """영점을 잡으면 좌표계가 통째로 옮겨가는데 직전 목표각은 옛 좌표계 값임.

        그대로 유지되면 그 차이만큼 관절이 튐.
        """
        leg.enable_torque([10])
        with pytest.raises(C.CommissioningError, match="토크가 켜져 있음"):
            C.set_zero(leg, 10, zero_reference="편 상태")

    def test_allowed_after_disabling(self, leg, live):
        leg.enable_torque([10])
        leg.disable_torque([10])
        C.set_zero(leg, 10, zero_reference="편 상태")
        assert live.sent[-1].data[7] == T.CMD_SET_ZERO

    def test_no_answer_raises(self, leg, raw):
        with pytest.raises(C.CommissioningError, match="응답이 없음"):
            C.set_zero(leg, 10, zero_reference="편 상태")


# ===========================================================================
# CAN id
# ===========================================================================
class TestSetCanId:
    def test_sends_new_id_in_f_cmd_then_verifies(self, leg, live):
        live.responses[20] = state_frame(20)
        C.set_can_id(leg, 10, 20)
        change = live.sent[0]
        assert change.arbitration_id == 10
        assert change.data[7] == T.CMD_SET_CAN_ID
        assert change.data[6] == 20
        assert live.sent[-1].arbitration_id == 20     # 새 id 로 확인함

    def test_rejects_out_of_range(self, leg, live):
        with pytest.raises(ValueError, match="1~127"):
            C.set_can_id(leg, 10, 200)
        with pytest.raises(ValueError, match="1~127"):
            C.set_can_id(leg, 10, 0)

    def test_rejects_same_id(self, leg, live):
        with pytest.raises(ValueError, match="새 id 가 현재 id 와 같음"):
            C.set_can_id(leg, 10, 10)

    def test_rejects_id_in_use(self, leg, live):
        """같은 id 가 둘이 되면 응답이 충돌해서 구분조차 안 됨."""
        with pytest.raises(C.CommissioningError, match="다른 모터가 쓰고 있음"):
            C.set_can_id(leg, 10, 11)

    def test_nothing_is_sent_when_rejected(self, leg, live):
        """거부 조건은 프레임을 내보내기 전에 걸러야 함."""
        with pytest.raises(C.CommissioningError):
            C.set_can_id(leg, 10, 11)
        assert live.sent == []

    def test_silent_new_id_raises(self, leg, live):
        """반영됐는데 응답만 놓쳤을 수 있으므로 양쪽을 확인하라고 알림."""
        with pytest.raises(C.CommissioningError, match="양쪽 id 로 다시 확인"):
            C.set_can_id(leg, 10, 20)


# ===========================================================================
# 프로토콜
# ===========================================================================
class TestSetProtocol:
    def test_sends_protocol_in_f_cmd(self, leg, live):
        C.set_protocol(leg, 10, T.Protocol.MIT)
        msg = live.sent[-1]
        assert msg.data[7] == T.CMD_SET_PROTOCOL
        assert msg.data[6] == int(T.Protocol.MIT)

    def test_private_differs(self, leg, live):
        C.set_protocol(leg, 10, T.Protocol.PRIVATE)
        assert live.sent[-1].data[6] == int(T.Protocol.PRIVATE)

    def test_warns_about_power_cycle(self, leg, live, caplog):
        """전원 재투입 전까지는 옛 포맷으로 계속 통신해야 함."""
        with caplog.at_level("WARNING"):
            C.set_protocol(leg, 10, T.Protocol.MIT)
        assert "재투입" in caplog.text

    def test_no_answer_raises(self, leg, raw):
        with pytest.raises(C.CommissioningError, match="응답이 없음"):
            C.set_protocol(leg, 10, T.Protocol.MIT)


# ===========================================================================
# nudge — 모터 id ↔ 관절 매핑 확인 (이슈 #8)
# ===========================================================================
class TestNudge:
    def test_returns_to_start(self, leg, live):
        """갔다가 돌아옴. 마지막 동작 명령의 목표가 시작 위치여야 함."""
        result = C.nudge(leg, 10, delta_deg=5.0, **FAST)
        assert result.start_deg == pytest.approx(0.0, abs=0.05)

        enc = leg.encoding(10)
        targets = [
            math.degrees(mit.uint_to_float((m.data[0] << 8) | m.data[1],
                                           -enc.pmax_rad, enc.pmax_rad, 16))
            for m in live.sent if m.data[0] != 0xFF
        ]
        assert max(targets) == pytest.approx(5.0, abs=0.05)
        assert targets[-1] == pytest.approx(0.0, abs=0.05)

    def test_relative_to_current_position(self, leg, live):
        """캘리브레이션 전이라 cal 공간이 없음. 절대 한계를 쓸 수 없음.

        지금 있는 자리에서 조금 움직이는 것만 안전하게 할 수 있음.
        """
        live.responses[10] = state_frame(10, 30.0)
        result = C.nudge(leg, 10, delta_deg=5.0, **FAST)
        assert result.start_deg == pytest.approx(30.0, abs=0.05)

        enc = leg.encoding(10)
        targets = [
            math.degrees(mit.uint_to_float((m.data[0] << 8) | m.data[1],
                                           -enc.pmax_rad, enc.pmax_rad, 16))
            for m in live.sent if m.data[0] != 0xFF
        ]
        assert max(targets) == pytest.approx(35.0, abs=0.05)

    def test_ends_with_torque_off(self, leg, live):
        """확인이 끝나면 힘이 빠져 있어야 함. 다음 모터로 넘어가기 전에 정리됨."""
        C.nudge(leg, 10, **FAST)
        assert live.sent[-1].data[7] == T.CMD_STOP
        assert leg.is_torque_on(10) is False

    def test_passive_before_stop(self, leg, live):
        """토크를 바로 끊으면 관절이 떨어짐. 게인을 뺀 명령을 먼저 보냄."""
        C.nudge(leg, 10, **FAST)
        motion = [m for m in live.sent if m.data[0] != 0xFF]
        last = motion[-1]
        assert ((last.data[3] & 0x0F) << 8) | last.data[4] == 0      # Kp
        assert (last.data[5] << 4) | (last.data[6] >> 4) == 0        # Kd

    def test_torque_off_even_on_failure(self, leg, live):
        """중간에 터져도 힘이 빠져야 함. 사람이 옆에 있는 작업임."""
        original = live.send
        calls = {"n": 0}

        def flaky(msg):
            calls["n"] += 1
            if calls["n"] == 5:
                raise KeyboardInterrupt("사람이 중단함")
            original(msg)

        live.send = flaky
        with pytest.raises(KeyboardInterrupt):
            C.nudge(leg, 10, **FAST)
        live.send = original
        assert leg.is_torque_on(10) is False

    def test_caps_the_amplitude(self, leg, live):
        """확인용이라 크게 움직일 이유가 없음. 본격적인 동작은 control/ 에서 함."""
        with pytest.raises(ValueError, match="20도까지만"):
            C.nudge(leg, 10, delta_deg=45.0)
        with pytest.raises(ValueError, match="20도까지만"):
            C.nudge(leg, 10, delta_deg=-45.0)

    def test_default_gains_are_low(self, leg, live):
        """브링업 초반이고 사람이 옆에 있음. 걸리면 못 움직이고 마는 편이 나음."""
        import inspect

        sig = inspect.signature(C.nudge)
        assert sig.parameters["kp"].default <= 10.0
        assert sig.parameters["delta_deg"].default <= 10.0

    def test_unknown_motor_rejected(self, leg, live):
        with pytest.raises(ValueError, match="없는 모터 id"):
            C.nudge(leg, 99, **FAST)

    def test_no_answer_stops_before_moving(self, leg, raw):
        """응답이 없는 모터에 토크를 넣지 않음. 배선부터 확인할 일임."""
        with pytest.raises(C.CommissioningError, match="배선과 CAN id"):
            C.nudge(leg, 10, **FAST)
        assert not any(m.data[7] == T.CMD_ENABLE for m in raw.sent if m.data[0] == 0xFF)

    def test_moved_deg_reports_what_happened(self, leg, live):
        """명령한 양과 다르면 게인이 낮거나 걸린 것임."""
        result = C.nudge(leg, 10, delta_deg=5.0, **FAST)
        assert result.moved_deg == pytest.approx(0.0, abs=0.05)   # 가짜는 안 움직임
        assert len(result.samples) == FAST["steps"] * 2


# ===========================================================================
# scan
# ===========================================================================
class TestScan:
    def test_finds_responders(self, leg, live):
        assert C.scan(leg) == [10, 11]

    def test_reports_partial(self, leg, raw):
        raw.responses = {11: state_frame(11, model=T.Model.RS00)}
        assert C.scan(leg, timeout_s=0.001) == [11]

    def test_empty_when_nothing_answers(self, leg, raw):
        """응답 없음과 프로토콜 불일치가 구분되지 않음. 둘 다 조용히 빠짐."""
        assert C.scan(leg, timeout_s=0.001) == []

    def test_logs_the_missing(self, leg, raw, caplog):
        with caplog.at_level("WARNING"):
            C.scan(leg, timeout_s=0.001)
        assert "응답 없는 모터" in caplog.text


# ===========================================================================
# sweep — 가동 범위 측정
# ===========================================================================
class TestSweep:
    def test_records_min_and_max(self, leg, live):
        """토크를 끄고 사람이 미는 동안 최대·최소를 기록함."""
        positions = [0.0, 30.0, -20.0, 45.0, 10.0]
        step = {"i": 0}

        def moving_send(msg):
            live.sent.append(msg)
            mid = msg.arbitration_id
            index = min(step["i"], len(positions) - 1)
            live.rx.append(state_frame(mid, positions[index], model=MODELS_BY_ID[mid]))

        live.send = moving_send

        def should_stop():
            step["i"] += 1
            return step["i"] > len(positions)

        results = C.sweep(leg, [10], should_stop=should_stop, hz=10000.0)
        assert results[10].lo_deg == pytest.approx(-20.0, abs=0.05)
        assert results[10].hi_deg == pytest.approx(45.0, abs=0.05)
        assert results[10].span_deg == pytest.approx(65.0, abs=0.1)

    def test_cuts_torque_first(self, leg, live):
        """힘이 들어간 채로 밀면 모터와 싸우게 되고 범위가 좁게 나옴."""
        leg.enable_torque()
        live.sent.clear()
        C.sweep(leg, [10], should_stop=lambda: True)
        assert live.sent[0].data[7] == T.CMD_STOP
        assert leg.is_torque_on(10) is False

    def test_starts_from_the_current_position(self, leg, live):
        """시작 위치가 범위 안에 들어가야 함. 안 그러면 첫 값이 빠짐."""
        results = C.sweep(leg, [10], should_stop=lambda: True)
        assert results[10].lo_deg == results[10].hi_deg
        assert results[10].samples == 0

    def test_all_motors_by_default(self, leg, live):
        """발목은 발을 잡고 움직이면 두 모터가 같이 따라옴. 함께 재야 함."""
        results = C.sweep(leg, should_stop=lambda: True)
        assert set(results) == set(leg.motor_ids)

    def test_no_answer_is_rejected(self, leg, raw):
        """응답이 없으면 잴 것이 없음. 배선부터 확인할 일임."""
        with pytest.raises(C.CommissioningError, match="배선과 CAN id"):
            C.sweep(leg, should_stop=lambda: True)

    def test_offsets_move_the_recorded_range(self, leg, live):
        """0도를 먼저 정하므로 나온 값이 곧 관절 좌표계 각도임."""
        positions = [0.0, 30.0, -20.0]
        step = {"i": 0}

        def moving_send(msg):
            live.sent.append(msg)
            mid = msg.arbitration_id
            index = min(step["i"], len(positions) - 1)
            live.rx.append(state_frame(mid, positions[index], model=MODELS_BY_ID[mid]))

        live.send = moving_send

        def should_stop():
            step["i"] += 1
            return step["i"] > len(positions)

        results = C.sweep(
            leg, [10], should_stop=should_stop, hz=10000.0, offsets={10: 12.0}
        )
        assert results[10].offset_deg == 12.0
        assert results[10].lo_deg == pytest.approx(-8.0, abs=0.05)
        assert results[10].hi_deg == pytest.approx(42.0, abs=0.05)
        assert results[10].span_deg == pytest.approx(50.0, abs=0.1)

    def test_no_offset_means_raw(self, leg, live):
        """오프셋을 안 주면 raw 그대로임. 캘리브레이션 전에도 쓸 수 있어야 함."""
        results = C.sweep(leg, [10], should_stop=lambda: True)
        assert results[10].offset_deg == 0.0

    def test_crossing_the_boundary_does_not_blow_up(self, leg, live):
        """모터는 `[-180, 180)` 로만 보고함. 경계를 지나면 보고값이 360 만큼
        건너뛰는데, 접지 않으면 40도 움직인 것이 356도 범위로 나옴."""
        positions = [170.0, 179.0, -177.0, -165.0, -160.0]
        step = {"i": 0}

        def moving_send(msg):
            live.sent.append(msg)
            mid = msg.arbitration_id
            index = min(step["i"], len(positions) - 1)
            live.rx.append(state_frame(mid, positions[index], model=MODELS_BY_ID[mid]))

        live.send = moving_send

        def should_stop():
            step["i"] += 1
            return step["i"] > len(positions)

        results = C.sweep(
            leg, [10], should_stop=should_stop, hz=10000.0, offsets={10: -170.0}
        )
        assert results[10].span_deg == pytest.approx(30.0, abs=0.1)
        assert results[10].lo_deg == pytest.approx(0.0, abs=0.05)
        assert results[10].hi_deg == pytest.approx(30.0, abs=0.05)


class TestMeasureOffset:
    """지금 자세를 관절 0도로 놓는 값을 냄. `sweep` 이 단계마다 부름."""

    def test_current_pose_becomes_zero(self, leg, live):
        """cal = raw + offset 이므로 지금 각도가 0으로 읽히려면 offset = -raw 임."""
        live.send = lambda msg: live.rx.append(
            state_frame(msg.arbitration_id, 33.4, model=MODELS_BY_ID[msg.arbitration_id])
        )
        offsets = C.measure_offset(leg, [10])
        assert offsets[10] == pytest.approx(-33.4, abs=0.05)

    def test_cuts_torque_first(self, leg, live):
        """사람이 자세를 잡고 있는 중임. 힘이 들어가 있으면 그 자세가 아님."""
        leg.enable_torque()
        live.sent.clear()
        C.measure_offset(leg, [10])
        assert live.sent[0].data[7] == T.CMD_STOP

    def test_all_motors_by_default(self, leg, live):
        offsets = C.measure_offset(leg)
        assert set(offsets) == set(leg.motor_ids)

    def test_one_missing_motor_is_rejected(self, leg, raw):
        """한 관절이라도 빠지면 그 관절의 0도를 모름. 0으로 두면 조용히 틀림."""
        with pytest.raises(C.CommissioningError, match="배선과 CAN id"):
            C.measure_offset(leg)

    def test_reports_in_raw_space(self, leg, live):
        """캘리브레이션 전에도 쓸 수 있어야 하므로 raw 공간임."""
        results = C.sweep(leg, [10], should_stop=lambda: True)
        assert results[10].lo_deg == pytest.approx(leg.state(10).position_deg, abs=0.05)
