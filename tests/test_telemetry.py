"""텔레메트리 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

UDP 는 진짜 소켓으로 자기 자신에게 보내 받아 봄. CSV 는 임시 폴더에 씀.

로봇은 가짜를 씀 — 여기서 확인하는 것은 **필드 이름과 실패했을 때의 태도**이지
모터 통신이 아님.
"""

import json
import socket
import time

import pytest

from huphy import telemetry
from huphy.sensors.base import ImuState
from huphy.telemetry import snapshot
from huphy.telemetry import Telemetry
from huphy.telemetry.csv_log import CsvSink
from huphy.telemetry.udp import MTU_LIMIT, UdpSink


class FakeCounters:
    def __init__(self):
        self.clips = {"limit": 2, "jump": 1}
        self.rejects = {"nan": 0, "nostate": 3}


class FakeCanCounters:
    tx_errors = 4
    rx_errors = 0
    drain_timeouts = 7


class FakeCanBus:
    counters = FakeCanCounters()


class FakeMotorBus:
    bus = FakeCanBus()


class FakeRobot:
    """`Robot` 계약 중 텔레메트리가 쓰는 부분만 채움."""

    name = "leg"

    def __init__(self, robot_id="right_leg", motors=("knee", "ankle_a")):
        self.id = robot_id
        self.motor_names = motors
        self.counters = FakeCounters()
        self.bus = FakeMotorBus()
        self._obs = {}
        for m in motors:
            self._obs.update({
                f"{m}.pos": 10.0, f"{m}.vel": 1.0,
                f"{m}.torque": 0.5, f"{m}.temp": 34.5,
            })
        self._obs["stale_motors"] = 0
        self.last_sent = {motors[0]: 12.0}

    def get_observation(self):
        return dict(self._obs)


@pytest.fixture
def robot():
    return FakeRobot()


@pytest.fixture
def receiver():
    """진짜 UDP 소켓. 자기 자신에게 보내 받아 봄."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    s.settimeout(1.0)
    yield s
    s.close()


# ===========================================================================
# 필드 이름
# ===========================================================================
class TestFieldNames:
    def test_known_before_running(self, robot):
        """CSV 헤더와 PlotJuggler 레이아웃이 이 목록을 씀.

        실행해 보고 나서야 필드를 알면 로그 형식이 매 실행마다 달라짐.
        """
        assert snapshot.field_names(robot) == snapshot.field_names(robot)

    def test_matches_what_build_produces(self, robot):
        """두 군데에서 필드를 만들면 반드시 어긋남. 여기서 고정함."""
        assert set(snapshot.build(robot, t=0.0)) == set(snapshot.field_names(robot))

    def test_limb_name_is_prefixed(self, robot):
        """양다리를 같이 기록할 때 knee 가 둘이 됨."""
        assert "right_leg/knee/pos" in snapshot.field_names(robot)

    def test_every_motor_gets_every_field(self, robot):
        names = snapshot.field_names(robot)
        for motor in robot.motor_names:
            for field in snapshot.FAST_MOTOR_FIELDS + snapshot.DIAG_MOTOR_FIELDS:
                assert f"right_leg/{motor}/{field}" in names

    def test_order_is_stable(self, robot):
        """CSV 열 순서가 이 순서임. 바뀌면 기록을 이어 볼 수 없음."""
        assert snapshot.field_names(robot)[:2] == ("t", "loop_dt")

    def test_csv_is_fast_plus_diag(self, robot):
        """CSV 는 나누지 않음. 한 줄에 다 있어야 나중에 대조하기 쉬움."""
        fast = set(snapshot.fast_field_names(robot))
        diag = set(snapshot.diag_field_names(robot))
        assert set(snapshot.field_names(robot)) == fast | diag

    def test_t_appears_once_in_csv(self, robot):
        """양쪽 패킷에 다 있지만 열은 하나여야 함."""
        assert list(snapshot.field_names(robot)).count("t") == 1

    def test_fast_and_diag_are_disjoint_apart_from_t(self, robot):
        """같은 값이 두 패킷에 실리면 대역만 낭비함."""
        fast = set(snapshot.fast_field_names(robot))
        diag = set(snapshot.diag_field_names(robot))
        assert fast & diag == {"t"}


# ===========================================================================
# 스냅샷
# ===========================================================================
class TestBuild:
    def test_error_is_target_minus_measured(self, robot):
        """게인 튜닝에서 제일 먼저 보는 값임."""
        data = snapshot.build(robot, t=1.0)
        assert data["right_leg/knee/tgt"] == 12.0
        assert data["right_leg/knee/pos"] == 10.0
        assert data["right_leg/knee/err"] == pytest.approx(2.0)

    def test_target_is_what_actually_went_out(self, robot):
        """명령한 값이 아니라 잘리고 남은 값임.

        오차를 보면 모터가 왜 그렇게 움직였는지가 설명됨.
        """
        robot.last_sent = {"knee": 71.79}
        assert snapshot.build(robot, t=0.0)["right_leg/knee/tgt"] == 71.79

    def test_unsent_motor_reports_zero_error(self, robot):
        """발목은 명령이 관절로 오므로 모터별 목표가 없음.

        실측을 목표로 둬서 오차가 0이 되게 함 — 가짜 오차가 그래프에 남는 것보다 나음.
        """
        data = snapshot.build(robot, t=0.0)
        assert data["right_leg/ankle_a/err"] == 0.0

    def test_counters_are_included(self, robot):
        data = snapshot.build(robot, t=0.0)
        assert data["right_leg/guard/clip_limit"] == 2
        assert data["right_leg/guard/reject_nostate"] == 3
        assert data["right_leg/can/tx_errors"] == 4

    def test_missing_counters_become_zero(self):
        """카운터가 없어도 키는 나감. 사라지면 그래프가 끊기고 CSV 열이 밀림."""
        bare = FakeRobot()
        del bare.counters
        del bare.bus
        data = snapshot.build(bare, t=0.0)
        assert data["right_leg/guard/clip_limit"] == 0.0
        assert data["right_leg/can/tx_errors"] == 0.0

    def test_only_numbers(self, robot):
        """PlotJuggler 는 숫자만 그림."""
        assert all(isinstance(v, (int, float)) for v in snapshot.build(robot, t=0.0).values())

    def test_does_not_touch_the_bus(self, robot):
        """새로 통신하지 않음. 기록이 주기를 흔들면 안 됨."""
        calls = []
        robot.get_observation = lambda: (calls.append(1), dict(robot._obs))[1]
        snapshot.build_fast(robot, t=0.0)
        assert len(calls) == 1

    def test_merge_keeps_both_limbs(self):
        left = snapshot.build(FakeRobot("left_leg"), t=1.0)
        right = snapshot.build(FakeRobot("right_leg"), t=1.0)
        merged = snapshot.merge(left, right)
        assert "left_leg/knee/pos" in merged
        assert "right_leg/knee/pos" in merged


# ===========================================================================
# UDP
# ===========================================================================
class TestUdp:
    def test_disabled_when_no_host(self):
        """설정에서 꺼 두는 것이 기본. 호출부가 분기하지 않아도 되게 함."""
        sink = UdpSink(None)
        assert sink.enabled is False
        assert sink.send({"t": 1.0}) is False

    def test_sends_json(self, receiver):
        port = receiver.getsockname()[1]
        with UdpSink("127.0.0.1", port) as sink:
            assert sink.send({"t": 1.5, "x": 2.0}) is True
        assert json.loads(receiver.recv(4096)) == {"t": 1.5, "x": 2.0}

    def test_rounds_to_shrink_the_packet(self, receiver):
        """소수점 줄이기가 패킷 크기를 반으로 줄이는 데 제일 큼.

        0.01도는 모터 해상도(0.022도)보다 촘촘해서 정보를 잃지 않음.
        """
        port = receiver.getsockname()[1]
        with UdpSink("127.0.0.1", port) as sink:
            sink.send({"x": 1.23456789})
        assert json.loads(receiver.recv(4096))["x"] == 1.23

    @pytest.mark.parametrize("builder", [snapshot.build_fast, snapshot.build_diag])
    def test_each_packet_fits(self, receiver, builder):
        """다리 하나의 한 패킷은 들어감. 둘을 합치면 넘침 — 그래서 나눠 보냄."""
        full = FakeRobot(motors=("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_a", "ankle_b"))
        port = receiver.getsockname()[1]
        with UdpSink("127.0.0.1", port) as sink:
            sink.send(builder(full, t=0.0))
        assert len(receiver.recv(4096)) < MTU_LIMIT
        assert sink.counters.oversize == 0

    def test_merged_would_not_fit(self, receiver):
        """나눈 이유를 고정함. 합치면 MTU 를 넘어 조각나고, 조각 하나만 잃어도
        패킷 전체가 버려짐.
        """
        full = FakeRobot(motors=("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_a", "ankle_b"))
        both = snapshot.build(full, t=0.0)
        left = FakeRobot("left_leg", motors=("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_a", "ankle_b"))
        merged = snapshot.merge(both, snapshot.build(left, t=0.0))
        port = receiver.getsockname()[1]
        with UdpSink("127.0.0.1", port) as sink:
            sink.send(merged)
        assert sink.counters.oversize == 1

    def test_oversize_is_counted(self, receiver):
        """조각나면 조각 하나만 잃어도 패킷 전체가 버려짐. 조용히 두면 안 됨."""
        port = receiver.getsockname()[1]
        with UdpSink("127.0.0.1", port) as sink:
            sink.send({f"aaaaaaaaaaaaaaaaaaaa{i}": 1.0 for i in range(100)})
        assert sink.counters.oversize == 1

    def test_send_failure_does_not_raise(self):
        """관측이 제어를 멈추면 관측할 대상이 없어짐."""
        sink = UdpSink("127.0.0.1", 9)
        sink.open()
        sink._socket.close()          # 소켓을 망가뜨림
        assert sink.send({"t": 1.0}) is False
        assert sink.counters.errors == 1

    def test_close_twice_is_safe(self):
        sink = UdpSink("127.0.0.1", 9999)
        sink.open()
        sink.close()
        sink.close()

    def test_counters_are_not_sent_over_udp(self, robot):
        """나가는 경로가 고장났는데 그 사실을 같은 경로로 알릴 수는 없음."""
        assert not any(k.startswith("udp.") for k in snapshot.build(robot, t=0.0))


# ===========================================================================
# CSV
# ===========================================================================
class TestCsv:
    def test_disabled_when_no_path(self):
        sink = CsvSink(None, ("t",))
        assert sink.enabled is False
        assert sink.write({"t": 1.0}) is False

    def test_header_comes_first(self, tmp_path, robot):
        fields = snapshot.field_names(robot)
        path = tmp_path / "log.csv"
        with CsvSink(path, fields) as sink:
            sink.write(snapshot.build(robot, t=0.0))
        assert path.read_text().splitlines()[0].split(",") == list(fields)

    def test_columns_keep_their_order(self, tmp_path, robot):
        """열이 밀리면 기록 전체가 못 쓰게 됨. 그걸 알아채는 것은 보통 사고 조사 중임."""
        fields = snapshot.field_names(robot)
        path = tmp_path / "log.csv"
        with CsvSink(path, fields) as sink:
            sink.write(snapshot.build(robot, t=1.25))
        header, row = path.read_text().splitlines()
        assert row.split(",")[header.split(",").index("t")] == "1.250"

    def test_unknown_field_is_dropped_and_counted(self, tmp_path):
        """열 순서를 지키는 것이 값 하나보다 중요함."""
        path = tmp_path / "log.csv"
        with CsvSink(path, ("t",)) as sink:
            sink.write({"t": 1.0, "새필드": 2.0})
        assert sink.counters.dropped_fields == 1
        assert path.read_text().splitlines()[1] == "1.000"

    def test_missing_field_becomes_blank(self, tmp_path):
        path = tmp_path / "log.csv"
        with CsvSink(path, ("t", "x")) as sink:
            sink.write({"t": 1.0})
        assert path.read_text().splitlines()[1] == "1.000,"

    def test_close_flushes(self, tmp_path):
        """버퍼에 남은 몇 줄이 사라지면 하필 사고 직전 부분을 잃음."""
        path = tmp_path / "log.csv"
        sink = CsvSink(path, ("t",), flush_every=1000)
        sink.open()
        for i in range(5):
            sink.write({"t": float(i)})
        sink.close()
        assert len(path.read_text().splitlines()) == 6

    def test_creates_parent_directory(self, tmp_path):
        path = tmp_path / "없던폴더" / "log.csv"
        with CsvSink(path, ("t",)) as sink:
            sink.write({"t": 1.0})
        assert path.is_file()

    def test_write_failure_does_not_raise(self, tmp_path):
        """디스크가 가득 차도 제어 루프는 계속 돌아야 함."""
        path = tmp_path / "log.csv"
        sink = CsvSink(path, ("t",))
        sink.open()
        sink._file.close()            # 파일을 망가뜨림
        assert sink.write({"t": 1.0}) is False

    def test_close_twice_is_safe(self, tmp_path):
        sink = CsvSink(tmp_path / "log.csv", ("t",))
        sink.open()
        sink.close()
        sink.close()


# ===========================================================================
# 합친 것
# ===========================================================================
class TestTelemetry:
    def test_both_get_the_same_snapshot(self, tmp_path, receiver, robot):
        """두 군데에서 필드를 만들면 어긋남. 같은 사전을 소비하는지 확인함."""
        port = receiver.getsockname()[1]
        path = tmp_path / "log.csv"
        with telemetry.Telemetry(robot, host="127.0.0.1", port=port,
                                 csv_path=str(path), flush_every=1, diag_every=1) as tm:
            tm.record(loop_dt_ms=10.0)

        fast = json.loads(receiver.recv(4096))
        diag = json.loads(receiver.recv(4096))
        header, row = path.read_text().splitlines()
        assert set(fast) | set(diag) == set(header.split(","))
        assert len(row.split(",")) == len(header.split(","))

    def test_time_starts_at_zero(self, tmp_path, robot):
        """벽시계를 쓰면 그래프 x 축이 1.7e9 에서 시작해 읽을 수 없음."""
        with telemetry.Telemetry(robot, csv_path=str(tmp_path / "log.csv")) as tm:
            assert tm.record()["t"] == 0.0
            assert tm.record()["t"] > 0.0

    def test_works_with_everything_off(self, robot):
        """호출부가 분기하지 않아도 되게 함."""
        tm = telemetry.Telemetry(robot)
        assert tm.enabled is False
        tm.open()
        assert set(tm.record()) == set(tm.fields)
        tm.close()

    def test_from_config(self, robot):
        from huphy.config.schema import TelemetryConfig

        tm = telemetry.Telemetry.from_config(robot, TelemetryConfig(host="1.2.3.4", port=1234))
        assert tm.udp.host == "1.2.3.4"
        assert tm.udp.port == 1234

    def test_fields_are_fixed_at_construction(self, robot):
        """CSV 헤더를 쓰고 나면 열이 고정됨. 중간에 바뀌면 기록이 밀림."""
        tm = telemetry.Telemetry(robot)
        assert tm.fields == snapshot.field_names(robot)

    def test_diag_is_decimated(self, tmp_path, receiver, robot):
        """temp 는 초 단위로 변하고 카운터는 사건이 있을 때만 변함.

        매 주기 보낼 이유가 없고, 합치면 패킷이 MTU 를 넘음.
        """
        port = receiver.getsockname()[1]
        with telemetry.Telemetry(robot, host="127.0.0.1", port=port, diag_every=3) as tm:
            for _ in range(6):
                tm.record()

        packets = []
        receiver.settimeout(0.2)
        while True:
            try:
                packets.append(json.loads(receiver.recv(4096)))
            except socket.timeout:
                break
        assert sum(1 for p in packets if "loop_dt" in p) == 6
        assert sum(1 for p in packets if "missing" in p) == 2

    def test_csv_gets_every_field_every_row(self, tmp_path, robot):
        """진단 값도 매번 계산함. 나누는 것은 보내는 쪽뿐임 — CSV 는 매 줄에 다
        있어야 나중에 대조하기 쉬움.
        """
        path = tmp_path / "log.csv"
        with telemetry.Telemetry(robot, csv_path=str(path), flush_every=1,
                                 diag_every=100) as tm:
            for _ in range(3):
                tm.record()
        rows = path.read_text().splitlines()
        assert len(rows) == 4
        assert all(len(r.split(",")) == len(rows[0].split(",")) for r in rows)
        assert all(cell != "" for cell in rows[-1].split(","))

    def test_self_counters_are_separate(self, tmp_path, robot):
        with telemetry.Telemetry(robot, csv_path=str(tmp_path / "log.csv")) as tm:
            tm.record()
            assert tm.as_fields()["csv.rows"] == 1
            assert "csv.rows" not in tm.fields


# ===========================================================================
# 발목 관절
# ===========================================================================
class AnkleRobot(FakeRobot):
    """발목 관절을 가진 로봇. 모터 값과 축이 다름."""

    joint_names = ("hip_pitch", "knee", "ankle_pitch", "ankle_roll")

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._obs["ankle_pitch.pos"] = 10.0
        self._obs["ankle_roll.pos"] = 5.0
        self.last_sent = dict(self.last_sent)
        self.last_sent["ankle_pitch"] = 12.0
        self.last_sent["ankle_roll"] = 5.0

    def ankle_velocity(self):
        return (1.5, -0.5)


class TorqueRobot(FakeRobot):
    """토크를 실어 보낼 수 있는 로봇."""

    torque_motors = ("ankle_a",)

    def __init__(self, commanded=None, **kwargs):
        super().__init__(**kwargs)
        self.last_torque = commanded if commanded is not None else {"ankle_a": 1.5}


class TestCommandedTorque:
    """모터가 보고한 `tau` 와 우리가 시킨 `tau_cmd` 는 다른 값임."""

    def test_the_column_exists(self):
        assert "right_leg/ankle_a/tau_cmd" in snapshot.field_names(TorqueRobot())

    def test_it_is_what_we_sent(self):
        row = snapshot.build_fast(TorqueRobot(), t=0.0)
        assert row["right_leg/ankle_a/tau_cmd"] == 1.5

    def test_it_sits_next_to_the_reported_torque(self):
        """둘을 같이 봐야 적게 시킨 것인지 못 낸 것인지 구분됨."""
        row = snapshot.build_fast(TorqueRobot(), t=0.0)
        assert "right_leg/ankle_a/tau" in row
        assert "right_leg/ankle_a/tau_cmd" in row

    def test_nothing_commanded_is_zero(self):
        """위치 모드에서는 늘 0임. 열이 사라지면 CSV 가 밀림."""
        row = snapshot.build_fast(TorqueRobot(commanded={}), t=0.0)
        assert row["right_leg/ankle_a/tau_cmd"] == 0.0

    def test_a_robot_without_torque_motors_gets_no_column(self, robot):
        assert not any("tau_cmd" in n for n in snapshot.field_names(robot))


class TestAnkleJointFields:
    def test_no_ankle_adds_no_columns(self, robot):
        """팔에는 발목이 없음."""
        assert not any("ankle_pitch/" in n for n in snapshot.field_names(robot))

    def test_the_joint_axis_is_separate_from_the_motors(self):
        """모터 값만 보면 발이 어떤 자세인지 안 보임."""
        names = snapshot.field_names(AnkleRobot())
        assert "right_leg/ankle_pitch/pos" in names
        assert "right_leg/ankle_a/pos" in names          # 모터 값도 그대로

    def test_build_matches_the_field_names(self):
        ankle = AnkleRobot()
        assert set(snapshot.build(ankle, t=0.0)) == set(snapshot.field_names(ankle))

    def test_error_is_target_minus_measured(self):
        row = snapshot.build_fast(AnkleRobot(), t=0.0)
        assert row["right_leg/ankle_pitch/pos"] == 10.0
        assert row["right_leg/ankle_pitch/tgt"] == 12.0
        assert row["right_leg/ankle_pitch/err"] == pytest.approx(2.0)

    def test_velocity_comes_from_the_robot(self):
        row = snapshot.build_fast(AnkleRobot(), t=0.0)
        assert row["right_leg/ankle_pitch/vel"] == 1.5
        assert row["right_leg/ankle_roll/vel"] == -0.5

    def test_no_target_means_zero_error(self):
        """명령하지 않았으면 실측을 목표로 둠. 가짜 오차가 남는 것보다 나음."""
        ankle = AnkleRobot()
        ankle.last_sent = {}
        row = snapshot.build_fast(ankle, t=0.0)
        assert row["right_leg/ankle_pitch/err"] == 0.0

    def test_a_broken_velocity_does_not_stop_recording(self):
        class Broken(AnkleRobot):
            def ankle_velocity(self):
                raise RuntimeError("특이점")

        row = snapshot.build_fast(Broken(), t=0.0)
        assert row["right_leg/ankle_pitch/vel"] == 0.0

    def test_the_fast_packet_still_fits(self):
        """다리 하나가 이미 MTU 에 가까움. 열이 늘면 확인해야 함."""
        row = snapshot.build_fast(AnkleRobot(motors=(
            "hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_a", "ankle_b")), t=0.0)
        payload = json.dumps({k: round(v, 2) for k, v in row.items()},
                             separators=(",", ":")).encode()
        assert len(payload) <= MTU_LIMIT


# ===========================================================================
# IMU — 붙었을 때만 나감
# ===========================================================================
class FakeImu:
    """오일러를 주는 센서. 고유 값 목록이 벤더마다 다름."""

    extra_fields = ("roll", "pitch", "yaw", "temp", "sensor_ms")

    def __init__(self, name="main"):
        self.name = name

    def read(self):
        return ImuState(
            gravity=(0.1, -0.2, -0.97),
            accel_mps2=(0.0, 0.0, 9.81),
            gyro_dps=(4.0, 5.0, 6.0),
            extra={"roll": 1.0, "pitch": 2.0, "yaw": 3.0,
                   "temp": 31.5, "sensor_ms": 1000.0},
            stamp=time.monotonic(),
            is_valid=True,
        )


class QuatImu(FakeImu):
    """쿼터니언을 주는 센서. 고유 열이 다름."""

    extra_fields = ("qw", "qx", "qy", "qz", "temp")

    def read(self):
        state = super().read()
        state.extra = {"qw": 0.9, "qx": 0.1, "qy": 0.2, "qz": 0.3, "temp": 31.5}
        return state


class ImuRobot(FakeRobot):
    def __init__(self, imus=("main",), kind=FakeImu, **kwargs):
        super().__init__(**kwargs)
        self.imus = tuple(kind(n) for n in imus)

    def imu_states(self):
        return {imu.name: imu.read() for imu in self.imus}


class TestImuFields:
    def test_no_imu_adds_no_columns(self, robot):
        """IMU 가 없는 로봇은 전과 같은 열을 가짐."""
        assert not any(n.startswith("imu/") for n in snapshot.field_names(robot))

    def test_the_imu_name_is_prefixed_not_the_limb(self):
        """다리에서 몸통으로 옮겨도 필드 이름이 그대로여야 함."""
        names = snapshot.field_names(ImuRobot())
        assert "imu/main/gx" in names
        assert "right_leg/imu/gx" not in names

    def test_build_matches_the_field_names(self):
        imu_robot = ImuRobot()
        assert set(snapshot.build(imu_robot, t=0.0)) == set(
            snapshot.field_names(imu_robot)
        )

    def test_values_come_through(self):
        row = snapshot.build_imu(ImuRobot(), t=0.0)
        assert row["imu/main/roll"] == 1.0
        assert row["imu/main/az"] == 9.81
        assert row["imu/main/gz"] == 6.0

    def test_the_gravity_the_policy_saw_is_recorded(self):
        """자세가 이상할 때 센서 원본과 중력방향 계산 중 어느 쪽인지 갈림."""
        row = snapshot.build_imu(ImuRobot(), t=0.0)
        assert (row["imu/main/grav_x"], row["imu/main/grav_y"], row["imu/main/grav_z"]) == (
            pytest.approx(0.1), pytest.approx(-0.2), pytest.approx(-0.97)
        )

    def test_common_columns_are_the_same_for_every_sensor(self):
        """센서를 바꿔도 공통 열은 그대로여야 예전 그래프 레이아웃이 맞음."""
        euler = {n for n in snapshot.field_names(ImuRobot())}
        quat = {n for n in snapshot.field_names(ImuRobot(kind=QuatImu))}
        common = {f"imu/main/{f}" for f in snapshot.IMU_FIELDS}
        assert common <= euler and common <= quat

    def test_vendor_columns_differ(self):
        """고유 열은 센서가 정함. extra_fields 가 목록을 냄."""
        quat = set(snapshot.field_names(ImuRobot(kind=QuatImu)))
        assert "imu/main/qw" in quat
        assert "imu/main/roll" not in quat

    def test_sensor_dt_is_emitted_when_the_sensor_stamps(self):
        """패킷 손실은 age 로 안 잡힘. 센서 시각 증가량으로 드러남."""
        names = snapshot.field_names(ImuRobot())
        assert "imu/main/sensor_dt" in names
        assert "imu/main/sensor_dt" not in snapshot.field_names(ImuRobot(kind=QuatImu))

    def test_several_imus_each_get_a_group(self):
        names = snapshot.field_names(ImuRobot(imus=("main", "foot")))
        assert "imu/main/gx" in names
        assert "imu/foot/gx" in names

    def test_a_silent_imu_still_emits_keys(self):
        """키가 사라지면 그래프가 끊기고 CSV 열이 밀림."""
        class Silent(ImuRobot):
            def imu_states(self):
                return {}

        row = snapshot.build_imu(Silent(), t=0.0)
        assert row["imu/main/roll"] == 0.0
        assert row["imu/main/age"] == -1.0
        assert row["imu/main/sensor_dt"] == -1.0

    def test_a_broken_imu_does_not_stop_recording(self):
        class Broken(ImuRobot):
            def imu_states(self):
                raise RuntimeError("포트가 빠짐")

        row = snapshot.build_imu(Broken(), t=0.0)
        assert row["imu/main/age"] == -1.0


class TestImuPacket:
    def test_it_goes_out_separately(self, receiver):
        """다리 하나가 이미 MTU 에 가까움. 합치면 조각남."""
        port = receiver.getsockname()[1]
        t = telemetry.Telemetry(ImuRobot(), host="127.0.0.1", port=port)
        t.open()
        t.record()
        t.close()

        packets = []
        for _ in range(3):
            try:
                packets.append(json.loads(receiver.recv(65535)))
            except socket.timeout:
                break

        imu_packets = [p for p in packets if any(k.startswith("imu/") for k in p)]
        assert len(imu_packets) == 1
        assert not any("knee" in k for k in imu_packets[0])

    def test_no_imu_means_no_extra_packet(self, robot, receiver):
        port = receiver.getsockname()[1]
        t = telemetry.Telemetry(robot, host="127.0.0.1", port=port)
        t.open()
        t.record()
        t.close()

        packets = []
        for _ in range(3):
            try:
                packets.append(json.loads(receiver.recv(65535)))
            except socket.timeout:
                break
        assert not any(any(k.startswith("imu/") for k in p) for p in packets)


# ===========================================================================
# 팔다리가 여럿일 때
# ===========================================================================
class FakeComposite:
    """팔다리 둘을 든 합성 로봇. 텔레메트리가 쓰는 부분만 채움."""

    name = "biped"

    def __init__(self, parts=None, imus=()):
        self.id = "huphy"
        self.parts = tuple(
            parts if parts is not None
            else (FakeRobot("right_leg"), FakeRobot("left_leg"))
        )
        self.imus = tuple(imus)

    @property
    def all_imus(self):
        return self.imus + tuple(i for p in self.parts for i in getattr(p, "imus", ()))

    def imu_states(self):
        return {imu.name: imu.read() for imu in self.all_imus}


class TestComposite:
    def test_parts_of_a_single_robot_is_itself(self, robot):
        assert snapshot.parts(robot) == (robot,)

    def test_parts_of_a_composite_are_the_limbs(self):
        composite = FakeComposite()
        assert snapshot.parts(composite) == composite.parts

    def test_columns_keep_the_single_leg_layout(self):
        """`huphy/right_leg/knee/pos` 처럼 깊어지면 예전 로그와 안 맞음."""
        names = snapshot.field_names(FakeComposite())
        assert "right_leg/knee/pos" in names
        assert "huphy/right_leg/knee/pos" not in names

    def test_both_limbs_get_columns(self):
        names = snapshot.field_names(FakeComposite())
        assert "right_leg/knee/pos" in names
        assert "left_leg/knee/pos" in names

    def test_one_time_column_appears_once(self):
        names = snapshot.field_names(FakeComposite())
        assert names.count("t") == 1

    def test_a_row_covers_both_limbs(self, tmp_path):
        """같은 시각의 두 다리를 나란히 봐야 함."""
        composite = FakeComposite()
        tele = Telemetry(composite, csv_path=str(tmp_path / "log.csv"))
        row = tele.record()
        tele.close()

        assert row["right_leg/knee/pos"] == 10.0
        assert row["left_leg/knee/pos"] == 10.0

    def test_the_row_matches_the_columns(self, tmp_path):
        """열과 값이 어긋나면 CSV 가 밀림."""
        composite = FakeComposite()
        tele = Telemetry(composite, csv_path=str(tmp_path / "log.csv"))
        row = tele.record()
        tele.close()

        assert set(row) == set(tele.fields)

    def test_udp_sends_one_packet_per_limb(self, receiver):
        """합쳐 보내면 MTU 를 넘어 조각남. 조각 하나만 잃어도 전체가 버려짐."""
        host, port = receiver.getsockname()
        tele = Telemetry(FakeComposite(), host=host, port=port)
        tele.open()
        tele.record()
        tele.close()

        seen = []
        for _ in range(4):
            try:
                seen.append(json.loads(receiver.recv(65535)))
            except socket.timeout:
                break

        fast = [p for p in seen if "right_leg/knee/pos" in p or "left_leg/knee/pos" in p]
        assert len(fast) == 2
        assert not any(
            "right_leg/knee/pos" in p and "left_leg/knee/pos" in p for p in seen
        )

    def test_imu_is_recorded_once_for_the_robot(self):
        """몸통 센서는 어느 다리의 것도 아님. 다리마다 반복하지 않음."""
        composite = FakeComposite(imus=[FakeImu("torso")])
        names = snapshot.field_names(composite)
        assert names.count("imu/torso/gx") == 1

    def test_leg_sensors_land_in_the_same_row(self):
        """몸통 센서와 다리 센서가 한 줄에 같이 있어야 대조가 됨."""
        right = FakeRobot("right_leg")
        right.imus = (FakeImu("shin"),)
        composite = FakeComposite(parts=(right,), imus=[FakeImu("torso")])

        names = snapshot.field_names(composite)
        assert "imu/torso/gx" in names
        assert "imu/shin/gx" in names
