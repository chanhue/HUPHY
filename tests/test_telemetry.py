"""텔레메트리 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

UDP 는 진짜 소켓으로 자기 자신에게 보내 받아 봄. CSV 는 임시 폴더에 씀.

로봇은 가짜를 씀 — 여기서 확인하는 것은 **필드 이름과 실패했을 때의 태도**이지
모터 통신이 아님.
"""

import json
import socket

import pytest

from huphy import telemetry
from huphy.telemetry import snapshot
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

    def __init__(self, robot_id="right_leg", motors=("knee", "ankle_a1")):
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
            for field in snapshot.MOTOR_FIELDS:
                assert f"right_leg/{motor}/{field}" in names

    def test_order_is_stable(self, robot):
        """CSV 열 순서가 이 순서임. 바뀌면 기록을 이어 볼 수 없음."""
        assert snapshot.field_names(robot)[:3] == ("t", "loop_dt", "missing")


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
        assert data["right_leg/ankle_a1/err"] == 0.0

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
        snapshot.build(robot, t=0.0)
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

    def test_one_leg_fits_in_a_packet(self, receiver, robot):
        """다리 하나는 들어감. 둘을 합치면 넘침 — 팔다리마다 한 패킷씩 보냄."""
        full = FakeRobot(motors=("hipz", "hipx", "hipy", "knee", "ankle_a1", "ankle_a2"))
        port = receiver.getsockname()[1]
        with UdpSink("127.0.0.1", port) as sink:
            sink.send(snapshot.build(full, t=0.0))
        assert len(receiver.recv(4096)) < MTU_LIMIT
        assert sink.counters.oversize == 0

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
                                 csv_path=str(path), flush_every=1) as tm:
            tm.record(loop_dt_ms=10.0)

        packet = json.loads(receiver.recv(4096))
        header, row = path.read_text().splitlines()
        assert set(packet) == set(header.split(","))
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

    def test_self_counters_are_separate(self, tmp_path, robot):
        with telemetry.Telemetry(robot, csv_path=str(tmp_path / "log.csv")) as tm:
            tm.record()
            assert tm.as_fields()["csv.rows"] == 1
            assert "csv.rows" not in tm.fields
