"""캘리브레이션 파일 읽기·쓰기 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

실제 `config/calibration/*.json` 도 함께 읽음 — 스키마와 파일이 어긋나면 여기서
걸림.
"""

import json
from pathlib import Path

import pytest

from huphy import calibration as cal
from huphy.calibration.store import SCHEMA_VERSION, CalibrationError
from huphy.config import load_robot
from huphy.motors.base import Motor, MotorCalibration

REPO = Path(__file__).resolve().parent.parent
ROBOT_YAML = REPO / "config" / "robot.yaml"

JOINTS = ("hipz", "hipx", "hipy", "knee", "ankle_a1", "ankle_a2")


def payload(motors=None, **over):
    data = {
        "schema_version": SCHEMA_VERSION,
        "limb": "right_leg",
        "note": "",
        "motors": motors if motors is not None else {
            "knee": {"sign": 1.0, "offset_deg": 0.0, "zero_reference": ""}
        },
    }
    data.update(over)
    return data


@pytest.fixture
def write(tmp_path):
    def _write(data, name="cal.json"):
        p = tmp_path / name
        p.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
        return p
    return _write


# ===========================================================================
# 실제 파일
# ===========================================================================
class TestRealFiles:
    @pytest.mark.parametrize("limb", ["right_leg", "left_leg"])
    def test_loads(self, limb):
        """스키마와 실제 파일이 어긋나면 여기서 걸림."""
        path = load_robot(ROBOT_YAML).limb(limb).calibration_path
        c = cal.load(path)
        assert set(c) == set(JOINTS)

    @pytest.mark.parametrize("limb", ["right_leg", "left_leg"])
    def test_joint_names_match_robot_yaml(self, limb):
        """한쪽에만 있는 관절이 있으면 그 관절이 조용히 항등변환으로 돎."""
        cfg = load_robot(ROBOT_YAML).limb(limb)
        c = cal.load(cfg.calibration_path)
        assert set(c) == set(cfg.motors)

    def test_everything_is_unmeasured(self):
        """지금은 전부 미실측임. zero_reference 가 비어 있음 (이슈 #9)."""
        path = load_robot(ROBOT_YAML).limb("right_leg").calibration_path
        assert set(cal.unmeasured(cal.load(path))) == set(JOINTS)

    def test_identity_for_now(self):
        """sign=1, offset=0 이라 cal == raw 임.

        어느 쪽으로 해석해도 동작이 같아서 두 공간을 섞어 써도 안 드러남 (이슈 #2).
        """
        path = load_robot(ROBOT_YAML).limb("right_leg").calibration_path
        for c in cal.load(path).values():
            assert c.raw_to_cal(45.0) == 45.0


# ===========================================================================
# 읽기 거부
# ===========================================================================
class TestLoadRejects:
    def test_missing_file(self, tmp_path):
        with pytest.raises(CalibrationError, match="파일이 없음"):
            cal.load(tmp_path / "없음.json")

    def test_broken_json(self, tmp_path):
        p = tmp_path / "cal.json"
        p.write_text("{ this is not json", encoding="utf-8")
        with pytest.raises(CalibrationError, match="JSON 을 읽을 수 없음"):
            cal.load(p)

    def test_old_schema_version(self, write):
        """버전 1은 한계와 게인을 함께 담고 모터 id 로 키를 잡았음.

        조용히 읽으면 한계가 두 군데 있게 되고, 어느 쪽이 쓰이는지 알 수 없음.
        """
        with pytest.raises(CalibrationError, match="schema_version"):
            cal.load(write(payload(schema_version=1)))

    def test_error_says_what_changed(self, write):
        """무엇을 고쳐야 하는지 알려줘야 옮길 수 있음."""
        with pytest.raises(CalibrationError, match="robot.yaml 로 옮기고"):
            cal.load(write(payload(schema_version=1)))

    def test_limits_key_is_rejected(self, write):
        """한계는 적는 값이라 robot.yaml 에 있음 (이슈 #2).

        여기 남아 있으면 값이 두 군데가 되어 한쪽만 고쳐질 수 있음.
        """
        p = write(payload(motors={"knee": {"sign": 1.0, "limit_lo_deg": -20.0}}))
        with pytest.raises(CalibrationError, match=r"모르는 키 \['limit_lo_deg'\]"):
            cal.load(p)

    def test_gain_key_is_rejected(self, write):
        p = write(payload(motors={"knee": {"sign": 1.0, "kp": 30.0}}))
        with pytest.raises(CalibrationError, match="robot.yaml 에 있음"):
            cal.load(p)

    def test_unknown_top_level_key(self, write):
        with pytest.raises(CalibrationError, match=r"모르는 키 \['한계'\]"):
            cal.load(write(payload(**{"한계": {}})))

    def test_sign_zero_is_rejected(self, write):
        """모든 raw 가 같은 cal 로 뭉개져 역변환이 불가능해짐."""
        p = write(payload(motors={"knee": {"sign": 0.0}}))
        with pytest.raises(CalibrationError, match="sign 이 0임"):
            cal.load(p)

    def test_non_numeric(self, write):
        p = write(payload(motors={"knee": {"sign": "왼쪽"}}))
        with pytest.raises(CalibrationError, match="숫자가 아닌 값"):
            cal.load(p)

    def test_motors_must_be_a_dict(self, write):
        with pytest.raises(CalibrationError, match="motors 는 사전이어야 함"):
            cal.load(write(payload(motors=[])))

    def test_missing_fields_use_identity(self, write):
        """빠진 항목은 항등변환으로 봄. 파일을 새로 만들 때 편함."""
        c = cal.load(write(payload(motors={"knee": {}})))
        assert c["knee"].sign == 1.0
        assert c["knee"].offset_deg == 0.0
        assert c["knee"].zero_reference == ""


# ===========================================================================
# 쓰기
# ===========================================================================
class TestSave:
    def test_round_trip(self, tmp_path):
        p = tmp_path / "cal.json"
        original = {
            "knee": MotorCalibration(motor_id=10, sign=-1.0, offset_deg=12.0,
                                     zero_reference="다리 편 상태"),
            "hipz": MotorCalibration(motor_id=7, sign=1.0, offset_deg=-3.5),
        }
        cal.save(p, original, limb="right_leg", note="테스트")
        back = cal.load(p)

        assert back["knee"].sign == -1.0
        assert back["knee"].offset_deg == 12.0
        assert back["knee"].zero_reference == "다리 편 상태"
        assert back["hipz"].offset_deg == -3.5

    def test_motor_id_is_not_stored(self, tmp_path):
        """robot.yaml 이 가진 값임. 두 군데 있으면 어긋날 수 있음."""
        p = tmp_path / "cal.json"
        cal.save(p, {"knee": MotorCalibration(motor_id=10)})
        entry = json.loads(p.read_text(encoding="utf-8"))["motors"]["knee"]
        assert set(entry) == {"sign", "offset_deg", "zero_reference"}

    def test_writes_current_schema_version(self, tmp_path):
        p = tmp_path / "cal.json"
        cal.save(p, {"knee": MotorCalibration(motor_id=10)})
        assert json.loads(p.read_text(encoding="utf-8"))["schema_version"] == SCHEMA_VERSION

    def test_korean_survives(self, tmp_path):
        """메모가 한글이라 이스케이프되면 사람이 못 읽음."""
        p = tmp_path / "cal.json"
        cal.save(p, {"knee": MotorCalibration(motor_id=10, zero_reference="발바닥 평면 접촉")})
        assert "발바닥 평면 접촉" in p.read_text(encoding="utf-8")

    def test_creates_parent_directory(self, tmp_path):
        p = tmp_path / "없던폴더" / "cal.json"
        cal.save(p, {"knee": MotorCalibration(motor_id=10)})
        assert p.is_file()

    def test_original_survives_a_crash(self, tmp_path, monkeypatch):
        """실측값을 잃으면 다시 재는 수밖에 없는데, 로봇을 분해해야 할 수도 있음.

        임시 파일에 쓰고 바꿔치기하므로 도중에 죽어도 원본이 남음.
        """
        p = tmp_path / "cal.json"
        cal.save(p, {"knee": MotorCalibration(motor_id=10, offset_deg=12.0)})

        def boom(*a, **k):
            raise OSError("디스크가 가득 참")

        monkeypatch.setattr("os.replace", boom)
        with pytest.raises(OSError):
            cal.save(p, {"knee": MotorCalibration(motor_id=10, offset_deg=99.0)})

        assert cal.load(p)["knee"].offset_deg == 12.0

    def test_no_temp_files_left_behind(self, tmp_path, monkeypatch):
        p = tmp_path / "cal.json"
        cal.save(p, {"knee": MotorCalibration(motor_id=10)})

        monkeypatch.setattr("os.replace", lambda *a, **k: (_ for _ in ()).throw(OSError()))
        with pytest.raises(OSError):
            cal.save(p, {"knee": MotorCalibration(motor_id=10)})

        assert list(tmp_path.glob("*.tmp")) == []


# ===========================================================================
# 설정과 합치기
# ===========================================================================
class TestAttach:
    MOTORS = {
        "knee": Motor(id=10, model="RS02"),
        "hipz": Motor(id=7, model="RS02"),
    }

    def test_rekeys_by_motor_id(self):
        """RobStrideBus 는 관절 이름을 모름. 모터 id 로만 말함."""
        c = cal.identity(self.MOTORS)
        assert set(cal.attach(c, self.MOTORS)) == {10, 7}

    def test_fills_in_motor_id(self):
        """파일에 없는 값이라 읽을 때는 -1 로 남아 있음."""
        c = cal.identity(self.MOTORS)
        assert c["knee"].motor_id == -1
        assert cal.attach(c, self.MOTORS)[10].motor_id == 10

    def test_carries_measured_values(self):
        c = {"knee": MotorCalibration(motor_id=-1, sign=-1.0, offset_deg=12.0,
                                      zero_reference="편 상태"),
             "hipz": MotorCalibration(motor_id=-1)}
        got = cal.attach(c, self.MOTORS)
        assert got[10].sign == -1.0
        assert got[10].offset_deg == 12.0
        assert got[10].zero_reference == "편 상태"

    def test_missing_joint_is_an_error(self):
        """관절 하나가 조용히 항등변환으로 도는 것이 가장 나쁨.

        sign 이 반대인 관절이 그렇게 되면 목표에서 멀어지는 방향으로 토크가 걸림.
        """
        with pytest.raises(CalibrationError, match=r"캘리브레이션에 없는 관절 \['hipz'\]"):
            cal.attach({"knee": MotorCalibration(motor_id=-1)}, self.MOTORS)

    def test_extra_joint_is_an_error(self):
        """설정에 없는 관절이 있으면 이름이 바뀌었거나 파일이 다른 다리 것임."""
        c = cal.identity(list(self.MOTORS) + ["ankle_a1"])
        with pytest.raises(CalibrationError, match=r"설정에 없는 관절 \['ankle_a1'\]"):
            cal.attach(c, self.MOTORS)

    def test_real_files_attach(self):
        """실제 두 파일이 실제 설정과 맞는지."""
        r = load_robot(ROBOT_YAML)
        for name in ("right_leg", "left_leg"):
            limb = r.limb(name)
            got = cal.attach(cal.load(limb.calibration_path), limb.motors)
            assert set(got) == set(limb.motor_ids)


# ===========================================================================
# 미실측 판정
# ===========================================================================
class TestUnmeasured:
    def test_empty_note_means_unmeasured(self):
        c = {"knee": MotorCalibration(motor_id=10, sign=-1.0, offset_deg=12.0)}
        assert cal.unmeasured(c) == ("knee",)

    def test_note_means_measured(self):
        c = {"knee": MotorCalibration(motor_id=10, zero_reference="편 상태")}
        assert cal.unmeasured(c) == ()

    def test_whitespace_is_not_a_note(self):
        c = {"knee": MotorCalibration(motor_id=10, zero_reference="   ")}
        assert cal.unmeasured(c) == ("knee",)

    def test_identity_values_are_not_the_test(self):
        """실측 결과가 우연히 1.0/0.0 일 수 있음.

        메모는 사람이 적는 것이라 우연히 채워지지 않으므로 그쪽으로 판정함.
        """
        c = {"knee": MotorCalibration(motor_id=10, sign=1.0, offset_deg=0.0,
                                      zero_reference="재봤더니 항등이었음")}
        assert cal.unmeasured(c) == ()

    def test_identity_helper_is_all_unmeasured(self):
        assert set(cal.unmeasured(cal.identity(JOINTS))) == set(JOINTS)
