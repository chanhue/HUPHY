"""센서 계층 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

`pyserial` 이 없어도 돌아야 함. 벤더 모듈을 늦게 import 하는 것이 그 때문임.
"""

import sys
import types

import pytest

from huphy.sensors import make_imu
from huphy.sensors.base import Imu, ImuState
from huphy.sensors.registry import MODELS


# ===========================================================================
# ImuState
# ===========================================================================
class TestImuState:
    def test_nothing_received_is_not_zero_degrees(self):
        """값이 0인 것과 모르는 것은 다름. is_valid 로 구분함."""
        state = ImuState()
        assert state.is_valid is False
        assert state.roll_deg == 0.0

    def test_age_is_minus_one_before_the_first_packet(self):
        """무한대는 JSON 으로 못 보내고 CSV 에서도 읽기 어려움."""
        assert ImuState().age_ms() == -1.0

    def test_age_grows_from_the_stamp(self):
        state = ImuState(stamp=100.0, is_valid=True)
        assert state.age_ms(now=100.5) == pytest.approx(500.0)

    def test_triples_default_to_zero(self):
        state = ImuState()
        assert state.accel_mps2 == (0.0, 0.0, 0.0)
        assert state.gyro_dps == (0.0, 0.0, 0.0)


# ===========================================================================
# registry
# ===========================================================================
class FakeConfig:
    def __init__(self, model="xsens_mti", name="main"):
        self.name = name
        self.model = model
        self.port = "/dev/null"
        self.baudrate = 921600


class TestRegistry:
    def test_an_unknown_model_names_what_is_available(self):
        with pytest.raises(ValueError, match="xsens_mti"):
            make_imu(FakeConfig(model="없는것"))

    def test_making_one_does_not_open_the_port(self):
        """설정이 맞는지는 포트를 열기 전에 알 수 있어야 함."""
        imu = make_imu(FakeConfig())
        assert imu.is_connected is False

    def test_it_satisfies_the_protocol(self):
        assert isinstance(make_imu(FakeConfig()), Imu)

    def test_the_name_comes_from_the_config(self):
        """텔레메트리 필드 앞에 붙는 이름임."""
        assert make_imu(FakeConfig(name="torso_imu")).name == "torso_imu"


# ===========================================================================
# XsensImu
# ===========================================================================
class FakeReader:
    """`xbus.imu_reader.ImuReader` 자리. 시리얼을 열지 않음."""

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.started = False
        self.data = None

    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def get_data(self):
        return self.data


@pytest.fixture
def fake_xbus(monkeypatch):
    """`pyserial` 없이 XsensImu 를 돌릴 수 있게 리더를 갈아 끼움."""
    made = []

    def factory(**kwargs):
        reader = FakeReader(**kwargs)
        made.append(reader)
        return reader

    module = types.ModuleType("huphy.sensors.xsens.xbus.imu_reader")
    module.ImuReader = factory
    monkeypatch.setitem(sys.modules, "huphy.sensors.xsens.xbus.imu_reader", module)
    return made


@pytest.fixture
def imu(fake_xbus):
    from huphy.sensors.xsens import XsensImu

    return XsensImu("main", "/dev/xsens_mti", baudrate=115200)


class TestXsensLifecycle:
    def test_connect_starts_the_reader(self, imu, fake_xbus):
        imu.connect()
        assert imu.is_connected
        assert fake_xbus[0].started

    def test_connect_twice_is_safe(self, imu, fake_xbus):
        imu.connect()
        imu.connect()
        assert len(fake_xbus) == 1

    def test_disconnect_twice_is_safe(self, imu):
        imu.connect()
        imu.disconnect()
        imu.disconnect()
        assert imu.is_connected is False

    def test_the_baudrate_reaches_the_reader(self, imu, fake_xbus):
        """센서에 저장된 값과 다르면 조용히 아무 패킷도 안 들어옴."""
        imu.connect()
        assert fake_xbus[0].kwargs["baudrate"] == 115200

    def test_reading_before_connect_is_not_an_error(self, imu):
        """센서 하나 때문에 제어 루프가 멈추면 안 됨."""
        assert imu.read().is_valid is False


class TestXsensRead:
    def _packet(self, **extra):
        data = {
            "euler": (1.0, 2.0, 3.0),
            "acceleration": (0.0, 0.0, 9.81),
            "rate_of_turn": (0.0, 0.0, 0.0),
            "temperature": 31.5,
            "timestamp": 1000.0,
        }
        data.update(extra)
        return data

    def test_euler_becomes_roll_pitch_yaw(self, imu, fake_xbus):
        imu.connect()
        fake_xbus[0].data = self._packet()
        state = imu.read()
        assert (state.roll_deg, state.pitch_deg, state.yaw_deg) == (1.0, 2.0, 3.0)
        assert state.is_valid

    def test_rate_of_turn_becomes_degrees(self, imu, fake_xbus):
        """센서는 rad/s 로 줌. 프로젝트 전체가 도를 씀."""
        import math

        imu.connect()
        fake_xbus[0].data = self._packet(rate_of_turn=(math.pi, 0.0, 0.0))
        assert imu.read().gyro_dps[0] == pytest.approx(180.0)

    def test_a_missing_field_becomes_zero(self, imu, fake_xbus):
        """필드가 나타났다 사라지면 텔레메트리 열이 밀림."""
        imu.connect()
        fake_xbus[0].data = {"timestamp": 1.0}
        state = imu.read()
        assert state.accel_mps2 == (0.0, 0.0, 0.0)
        assert state.is_valid

    def test_an_empty_reader_is_not_valid(self, imu, fake_xbus):
        imu.connect()
        assert imu.read().is_valid is False

    def test_age_grows_while_the_sensor_is_stuck(self, imu, fake_xbus):
        """같은 패킷이 계속 나오면 stamp 를 갱신하지 않음. 그래야 멈춘 걸 알아챔."""
        imu.connect()
        fake_xbus[0].data = self._packet()
        first = imu.read().stamp
        second = imu.read().stamp
        assert first == second

    def test_a_new_packet_moves_the_stamp(self, imu, fake_xbus):
        imu.connect()
        fake_xbus[0].data = self._packet()
        first = imu.read().stamp
        fake_xbus[0].data = self._packet(timestamp=1001.0)
        assert imu.read().stamp >= first

    def test_a_broken_reader_does_not_raise(self, imu, fake_xbus):
        imu.connect()

        def boom():
            raise RuntimeError("포트가 빠짐")

        fake_xbus[0].get_data = boom
        assert imu.read().is_valid is False


def test_every_registered_model_is_reachable():
    """표에 적힌 것은 전부 만들어져야 함."""
    for model in MODELS:
        assert make_imu(FakeConfig(model=model)) is not None
