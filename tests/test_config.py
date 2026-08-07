"""설정 읽기 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

여기서 확인하는 것 대부분은 **거부 조건**임. 설정이 잘못됐을 때 조용히 기본값으로
돌지 않고 멈추는지를 고정함.

실제 `config/robot.yaml` 도 함께 읽음 — 스키마와 파일이 어긋나면 여기서 걸림.
"""

from pathlib import Path

import pytest

from huphy.config import ConfigError, LimbConfig, RobotConfig, SafetyConfig, load_robot
from huphy.config.schema import TelemetryConfig
from huphy.motors.base import Gains, Motor

REPO = Path(__file__).resolve().parent.parent
ROBOT_YAML = REPO / "config" / "robot.yaml"

BASE = """
name: t
limbs:
  right_leg:
    kind: leg
    side: right
    channel: can1
    motors:
      knee: {id: 10, model: RS02, limits_deg: [-20.65, 74.79], kp: 30.0, kd: 1.0}
"""


@pytest.fixture
def write(tmp_path):
    def _write(text, name="robot.yaml"):
        p = tmp_path / name
        p.write_text(text, encoding="utf-8")
        return p
    return _write


# ===========================================================================
# 실제 파일
# ===========================================================================
class TestRealFile:
    def test_loads(self):
        """스키마와 config/robot.yaml 이 어긋나면 여기서 걸림."""
        r = load_robot(ROBOT_YAML)
        assert r.name == "huphy"
        assert set(r.limbs) == {"right_leg", "left_leg"}
        assert r.motor_count == 12

    def test_two_legs_two_channels(self):
        """다리 하나가 버스 하나를 씀. 두 버스는 물리적으로 독립이라 전송이 겹침."""
        r = load_robot(ROBOT_YAML)
        assert r.limb("right_leg").channel == "can1"
        assert r.limb("left_leg").channel == "can0"
        assert len(r.channels) == 2

    def test_motor_id_mapping(self):
        """설정상 매핑임. 실물 확인은 아직 안 됨 (이슈 #8)."""
        right = load_robot(ROBOT_YAML).limb("right_leg")
        assert [right.motors[j].id for j in
                ("hipz", "hipx", "hipy", "knee", "ankle_a1", "ankle_a2")] == [7, 8, 9, 10, 11, 12]

    def test_ankle_uses_rs00(self):
        """발목만 RS00 임. 토크 범위가 달라 같은 바이트가 다른 값을 뜻함."""
        right = load_robot(ROBOT_YAML).limb("right_leg")
        assert right.motors["knee"].model == "RS02"
        assert right.motors["ankle_a1"].model == "RS00"

    def test_gains_are_set(self):
        """튜닝 시작값이 들어 있음. 0이면 토크가 아예 안 나가 재볼 수도 없음 (이슈 #9)."""
        right = load_robot(ROBOT_YAML).limb("right_leg")
        assert all(m.gains.kp > 0.0 for m in right.motors.values())
        assert all(m.gains.kd > 0.0 for m in right.motors.values())
        assert right.is_configured is True

    def test_left_leg_has_no_gains(self):
        """한계를 모르는 관절에 게인만 넣으면 어디까지 가도 되는지 모르는 채로
        토크가 나감."""
        left = load_robot(ROBOT_YAML).limb("left_leg")
        assert all(m.gains.kp == 0.0 for m in left.motors.values())
        assert left.is_configured is False

    def test_left_leg_has_no_limits(self):
        """미실측 한계를 둥근 수로 채워 두면 실측한 값처럼 보여서 위험함.

        비워 두면 is_configured 가 False 가 되어 제어 진입 자체가 막힘.
        """
        left = load_robot(ROBOT_YAML).limb("left_leg")
        assert all(m.limits_deg is None for m in left.motors.values())

    def test_calibration_path_is_absolute(self):
        """실행 위치가 달라져도 같은 파일을 가리켜야 함."""
        p = load_robot(ROBOT_YAML).limb("right_leg").calibration_path
        assert p.is_absolute()
        assert p.is_file()

    def test_limits_are_not_in_calibration(self):
        """한계는 사람이 주석과 함께 관리하는 값이라 robot.yaml 에 있음 (이슈 #2)."""
        import json

        p = load_robot(ROBOT_YAML).limb("right_leg").calibration_path
        data = json.loads(p.read_text(encoding="utf-8"))
        for entry in data["motors"].values():
            assert "limit_lo_deg" not in entry
            assert "kp" not in entry
            assert set(entry) == {"sign", "offset_deg", "zero_reference"}


# ===========================================================================
# 오타 거부
# ===========================================================================
class TestUnknownKeys:
    def test_limb_typo(self, write):
        """YAML 은 오타를 조용히 삼킴. 설정을 고쳤는데 아무것도 안 바뀌는 상황이 됨."""
        p = write(BASE.replace("channel: can1", "channel: can1\n    contorl_hz: 200"))
        with pytest.raises(ConfigError, match=r"모르는 키 \['contorl_hz'\]"):
            load_robot(p)

    def test_motor_typo(self, write):
        p = write(BASE.replace("kp: 30.0", "kpp: 30.0"))
        with pytest.raises(ConfigError, match=r"모르는 키 \['kpp'\]"):
            load_robot(p)

    def test_safety_typo(self, write):
        p = write(BASE + "safety:\n  max_delta: 10\n")
        with pytest.raises(ConfigError, match=r"모르는 키 \['max_delta'\]"):
            load_robot(p)

    def test_top_level_typo(self, write):
        p = write(BASE + "telemetery:\n  port: 1\n")
        with pytest.raises(ConfigError, match=r"모르는 키 \['telemetery'\]"):
            load_robot(p)

    def test_error_lists_valid_keys(self, write):
        """뭘 쓸 수 있는지 알려줘야 고칠 수 있음."""
        p = write(BASE.replace("kp: 30.0", "kpp: 30.0"))
        with pytest.raises(ConfigError, match="limits_deg"):
            load_robot(p)


# ===========================================================================
# 구조 거부
# ===========================================================================
class TestStructure:
    def test_missing_file(self, tmp_path):
        with pytest.raises(ConfigError, match="설정 파일이 없음"):
            load_robot(tmp_path / "없음.yaml")

    def test_broken_yaml(self, write):
        p = write("name: t\n  limbs: [\n")
        with pytest.raises(ConfigError, match="YAML 을 읽을 수 없음"):
            load_robot(p)

    def test_missing_kind(self, write):
        """어떤 기구학을 쓸지 정하는 값임. 없으면 팔인지 다리인지 알 수 없음."""
        p = write(BASE.replace("    kind: leg\n", ""))
        with pytest.raises(ConfigError, match="kind 가 없음"):
            load_robot(p)

    def test_no_limbs(self, write):
        p = write("name: t\nlimbs: {}\n")
        with pytest.raises(ConfigError, match="limbs 가 비어 있음"):
            load_robot(p)

    def test_no_motors(self, write):
        p = write(BASE.split("    motors:")[0] + "    motors: {}\n")
        with pytest.raises(ConfigError, match="모터가 하나도 없음"):
            load_robot(p)

    def test_motors_as_list_is_rejected(self, write):
        """관절 이름이 키여야 함. 목록이면 이름이 없어 어느 관절인지 말할 수 없음."""
        p = write(BASE.replace("      knee: {id: 10", "      - {id: 10"))
        with pytest.raises(ConfigError, match="사전이어야 함"):
            load_robot(p)

    def test_missing_motor_id(self, write):
        p = write(BASE.replace("id: 10, ", ""))
        with pytest.raises(ConfigError, match="id 항목이 없음"):
            load_robot(p)

    def test_missing_model(self, write):
        p = write(BASE.replace("model: RS02, ", ""))
        with pytest.raises(ConfigError, match="model 항목이 없음"):
            load_robot(p)


# ===========================================================================
# 값 거부
# ===========================================================================
class TestValues:
    def test_reversed_limits(self, write):
        """cal 공간에는 sign 이 개입하지 않아 순서가 뒤집히지 않음. lo > hi 는 오타임."""
        p = write(BASE.replace("[-20.65, 74.79]", "[74.79, -20.65]"))
        with pytest.raises(ConfigError, match="lo 가 hi 보다"):
            load_robot(p)

    def test_limits_need_exactly_two(self, write):
        p = write(BASE.replace("[-20.65, 74.79]", "[1, 2, 3]"))
        with pytest.raises(ConfigError, match=r"\[최소, 최대\] 두 개"):
            load_robot(p)

    def test_zero_control_hz(self, write):
        p = write(BASE.replace("channel: can1", "channel: can1\n    control_hz: 0"))
        with pytest.raises(ConfigError, match="control_hz 는 0보다 커야 함"):
            load_robot(p)

    def test_bad_side(self, write):
        p = write(BASE.replace("side: right", "side: rihgt"))
        with pytest.raises(ConfigError, match="side 는 left/right"):
            load_robot(p)

    def test_negative_margin(self, write):
        p = write(BASE + "safety:\n  command_margin_deg: -1\n")
        with pytest.raises(ConfigError, match="command_margin_deg"):
            load_robot(p)

    def test_zero_max_delta(self, write):
        """0이면 어떤 명령도 통과하지 못해 다리가 굳음."""
        p = write(BASE + "safety:\n  max_delta_deg: 0\n")
        with pytest.raises(ConfigError, match="max_delta_deg"):
            load_robot(p)


# ===========================================================================
# id 충돌
# ===========================================================================
class TestIdCollision:
    def test_within_a_limb(self, write):
        """한 버스에서 id 가 겹치면 응답이 충돌해 구분되지 않음."""
        p = write(BASE + "      hipz: {id: 10, model: RS02}\n")
        with pytest.raises(ConfigError, match="같은 CAN id"):
            load_robot(p)

    def test_across_limbs_on_same_channel(self, write):
        """다른 팔다리라도 같은 채널이면 같은 선을 씀."""
        p = write(BASE + """  left_leg:
    kind: leg
    channel: can1
    motors:
      knee: {id: 10, model: RS02}
""")
        with pytest.raises(ConfigError, match=r"can1: CAN id 10"):
            load_robot(p)

    def test_same_id_on_different_channels_is_fine(self, write):
        """버스가 다르면 id 가 겹쳐도 됨. 실제로 왼다리 1~6, 오른다리 7~12 로 나눠
        두었지만 그건 사람이 헷갈리지 않기 위한 것이지 필수는 아님.
        """
        p = write(BASE + """  left_leg:
    kind: leg
    channel: can0
    motors:
      knee: {id: 10, model: RS02}
""")
        assert load_robot(p).motor_count == 2

    def test_error_names_both_joints(self, write):
        p = write(BASE + "      hipz: {id: 10, model: RS02}\n")
        with pytest.raises(ConfigError, match=r"\[10\]"):
            load_robot(p)


# ===========================================================================
# 기본값
# ===========================================================================
class TestDefaults:
    def test_omitted_sections_use_dataclass_defaults(self, write):
        """기본값이 두 군데 있으면 어느 쪽이 쓰이는지 알 수 없음. 스키마에만 둠."""
        r = load_robot(write(BASE))
        assert r.safety == SafetyConfig()
        assert r.telemetry == TelemetryConfig()

    def test_partial_section_keeps_other_defaults(self, write):
        r = load_robot(write(BASE + "safety:\n  max_delta_deg: 10\n"))
        assert r.safety.max_delta_deg == 10.0
        assert r.safety.command_margin_deg == 3.0     # 건드리지 않은 값

    def test_interface_defaults_to_socketcan(self, write):
        assert load_robot(write(BASE)).limb("right_leg").interface == "socketcan"

    def test_side_is_optional(self, write):
        """허리처럼 좌우가 없는 부위가 생길 수 있음."""
        p = write(BASE.replace("    side: right\n", ""))
        assert load_robot(p).limb("right_leg").side is None

    def test_gains_default_to_zero(self, write):
        """실측 전에 움직이지 않도록 0으로 둠."""
        p = write(BASE.replace(", kp: 30.0, kd: 1.0", ""))
        assert load_robot(p).limb("right_leg").motors["knee"].gains == Gains()

    def test_no_calibration_is_allowed(self, write):
        assert load_robot(write(BASE)).limb("right_leg").calibration_path is None


# ===========================================================================
# 스키마 자체
# ===========================================================================
class TestSchema:
    def test_frozen(self):
        """설정은 한 번 읽고 나면 바뀌지 않음. 제어 중에 누가 한계를 넓히면 안 됨."""
        c = SafetyConfig()
        with pytest.raises(Exception):
            c.max_delta_deg = 999

    def test_period_from_hz(self):
        limb = LimbConfig(name="l", kind="leg", control_hz=100.0,
                          motors={"knee": Motor(id=10, model="RS02")})
        assert limb.period_s == pytest.approx(0.01)

    def test_joint_lookup(self):
        limb = LimbConfig(name="l", kind="leg",
                          motors={"knee": Motor(id=10, model="RS02")})
        assert limb.joint_of(10) == "knee"
        assert limb.joint_of(99) is None

    def test_motors_by_id_drops_names(self):
        """RobStrideBus 가 받는 형태임. 그 계층은 관절 이름을 모름."""
        limb = LimbConfig(name="l", kind="leg",
                          motors={"knee": Motor(id=10, model="RS02")})
        assert set(limb.motors_by_id()) == {10}

    def test_unconfigured_lists_names(self):
        """무엇을 더 재야 하는지 사람에게 보여주는 값임."""
        limb = LimbConfig(name="l", kind="leg", motors={
            "knee": Motor(id=10, model="RS02", limits_deg=(-20.0, 70.0),
                          gains=Gains(kp=30.0, kd=1.0)),
            "hipz": Motor(id=7, model="RS02"),
        })
        assert limb.unconfigured() == ("hipz",)
        assert limb.is_configured is False

    def test_unknown_limb_lists_available(self):
        r = RobotConfig(name="t", limbs={
            "right_leg": LimbConfig(name="right_leg", kind="leg",
                                    motors={"knee": Motor(id=10, model="RS02")})
        })
        with pytest.raises(KeyError, match="right_leg"):
            r.limb("left_leg")

    def test_limbs_of_kind(self):
        """두 다리에 같은 처리를 걸 때 씀. 팔이 붙어도 그대로 동작함."""
        r = RobotConfig(name="t", limbs={
            "right_leg": LimbConfig(name="right_leg", kind="leg", channel="can1",
                                    motors={"knee": Motor(id=10, model="RS02")}),
            "right_arm": LimbConfig(name="right_arm", kind="arm", channel="can2",
                                    motors={"elbow": Motor(id=20, model="RS02")}),
        })
        assert set(r.limbs_of_kind("leg")) == {"right_leg"}
        assert set(r.limbs_of_kind("arm")) == {"right_arm"}

    def test_channels_are_deduplicated_in_order(self):
        r = RobotConfig(name="t", limbs={
            "a": LimbConfig(name="a", kind="leg", channel="can1",
                            motors={"j": Motor(id=1, model="RS02")}),
            "b": LimbConfig(name="b", kind="arm", channel="can1",
                            motors={"j": Motor(id=2, model="RS02")}),
            "c": LimbConfig(name="c", kind="arm", channel="can0",
                            motors={"j": Motor(id=3, model="RS02")}),
        })
        assert r.channels == ("can1", "can0")
