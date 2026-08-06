"""발목 링키지 기구학 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q

순수 계산이라 가짜 버스가 필요 없음. 실측 기하값을 그대로 씀.

**여기서 확인되는 것은 계산의 자기일관성뿐임.** 기하값이 실제 로봇과 맞는지는
실물에서만 확인됨 (이슈 #13).
"""

import pytest

from huphy.kinematics import (
    AnkleEnvelope,
    AnkleGeometry,
    AnkleKinematics,
    AnkleUnreachableError,
)

POSES = [(0, 0), (10, 0), (-10, 0), (0, 10), (0, -10), (20, 15), (-30, -20), (40, 25)]


@pytest.fixture
def ankle():
    return AnkleKinematics()


@pytest.fixture
def mirrored():
    return AnkleKinematics(AnkleGeometry().mirrored())


# ===========================================================================
# 기하
# ===========================================================================
class TestGeometry:
    def test_rod_lengths_come_from_the_neutral_pose(self, ankle):
        """로드 길이는 실측값이 아니라 중립 자세에서 계산된 것임.

        따라서 좌표를 고치면 길이도 따라 바뀜 — 따로 관리할 값이 아님.
        """
        l1, l2 = ankle.rod_lengths
        assert l1 == pytest.approx(143.02, abs=0.01)
        assert l2 == pytest.approx(85.04, abs=0.01)

    def test_two_rods_have_different_lengths(self, ankle):
        """대칭 링키지가 아님. 두 로드가 다른 길이로 같은 발판을 움직임."""
        l1, l2 = ankle.rod_lengths
        assert abs(l1 - l2) > 50.0

    def test_geometry_is_frozen(self):
        """제어 중에 기하가 바뀌면 안 됨."""
        with pytest.raises(Exception):
            AnkleGeometry().crank_r = 50.0

    def test_mirror_keeps_rod_lengths(self, ankle, mirrored):
        """거울상은 같은 기계임. 길이가 달라지면 뒤집기가 틀린 것임."""
        assert mirrored.rod_lengths == pytest.approx(ankle.rod_lengths)

    def test_mirror_flips_signs_too(self):
        """좌표만 뒤집으면 안 됨.

        `crank_t` 는 x 방향 오프셋이라 부호가 같이 뒤집혀야 하고, 모터 회전축이
        x 축이므로 yz 평면 반사에서 회전 방향도 뒤집힘.
        """
        g = AnkleGeometry()
        m = g.mirrored()
        assert m.a1[0] == -g.a1[0]
        assert m.a1[1] == g.a1[1]
        assert m.offset_sign_1 == -g.offset_sign_1
        assert m.rotation_sign_1 == -g.rotation_sign_1

    def test_mirror_twice_is_identity(self):
        assert AnkleGeometry().mirrored().mirrored() == AnkleGeometry()


# ===========================================================================
# 역기구학
# ===========================================================================
class TestSolveIk:
    def test_neutral_is_zero(self, ankle):
        """중립 자세에서 두 모터각이 0이어야 함. 로드 길이의 기준점임."""
        a1, a2 = ankle.solve_ik(0.0, 0.0)
        assert a1 == pytest.approx(0.0, abs=1e-9)
        assert a2 == pytest.approx(0.0, abs=1e-9)

    def test_pitch_moves_both_motors_oppositely(self, ankle):
        """pitch 는 발끝을 올리고 내림. 두 모터가 반대로 돎."""
        a1, a2 = ankle.solve_ik(10.0, 0.0)
        assert a1 * a2 < 0

    def test_roll_moves_both_motors_the_same_way(self, ankle):
        """roll 은 발을 옆으로 기울임. 두 모터가 같은 쪽으로 돎."""
        a1, a2 = ankle.solve_ik(0.0, 10.0)
        assert a1 * a2 > 0

    def test_opposite_poses_give_opposite_angles(self, ankle):
        """부호를 뒤집으면 각도도 대체로 뒤집힘. 링키지가 비대칭이라 정확히는 아님."""
        pos = ankle.solve_ik(10.0, 0.0)
        neg = ankle.solve_ik(-10.0, 0.0)
        assert pos[0] * neg[0] < 0
        assert pos[1] * neg[1] < 0

    @pytest.mark.parametrize("pitch,roll", POSES)
    def test_both_angles_are_in_one_convention(self, ankle, pitch, roll):
        """두 모터 각도가 모두 [-180, 180) 이어야 함.

        모터가 zero_sta=1 로 그 범위를 보고함. 한쪽만 [0, 360) 이면 IK 가 340도를
        돌려주고 모터는 -20도를 보고해 360도 차이가 남 (이슈 #1).
        """
        for a in ankle.solve_ik(pitch, roll):
            assert -180.0 <= a < 180.0

    def test_is_deterministic(self, ankle):
        """상태를 갖지 않음. 같은 입력에 항상 같은 출력임."""
        assert ankle.solve_ik(15.0, 8.0) == ankle.solve_ik(15.0, 8.0)


# ===========================================================================
# 순기구학
# ===========================================================================
class TestSolveFk:
    @pytest.mark.parametrize("pitch,roll", POSES)
    def test_round_trip(self, ankle, pitch, roll):
        a1, a2 = ankle.solve_ik(pitch, roll)
        back_pitch, back_roll = ankle.solve_fk(a1, a2)
        assert back_pitch == pytest.approx(pitch, abs=1e-4)
        assert back_roll == pytest.approx(roll, abs=1e-4)

    def test_guess_helps_far_poses(self, ankle):
        """같은 모터각 조합에 대응하는 자세가 여럿임.

        추정이 멀면 다른 자세로 수렴함. 제어 루프에서는 한 주기 전 자세가 늘 가까움.
        """
        a1, a2 = ankle.solve_ik(90.0, 90.0, enforce_envelope=False)
        got = ankle.solve_fk(a1, a2, guess_pitch_deg=90.0, guess_roll_deg=90.0)
        assert got[0] == pytest.approx(90.0, abs=1e-3)
        assert got[1] == pytest.approx(90.0, abs=1e-3)

    def test_fk_is_not_unique(self, ankle):
        """**같은 모터각 조합이 서로 다른 자세 둘에 대응함.**

        링키지의 성질이지 버그가 아님. 어느 쪽으로 수렴할지는 초기 추정이 정함.
        여기서는 (90, 90) 과 (2.41, -40.48) 이 같은 모터각을 냄.

        따라서 FK 결과를 "지금 자세" 로 믿으려면 추정이 실제와 가까워야 함.
        """
        a1, a2 = ankle.solve_ik(90.0, 90.0, enforce_envelope=False)
        other = ankle.solve_fk(a1, a2)                    # 기본 추정 (0, 0)

        assert abs(other[0] - 90.0) > 1.0                 # 다른 자세로 감
        assert ankle.solve_ik(*other, enforce_envelope=False) == pytest.approx(
            (a1, a2), abs=1e-6
        )                                                  # 그런데 둘 다 맞는 해임

    @pytest.mark.parametrize("pitch", range(-40, 41, 10))
    @pytest.mark.parametrize("roll", range(-25, 26, 5))
    def test_default_guess_is_enough_inside_the_envelope(self, ankle, pitch, roll):
        """시험 범위 안에서는 추정 (0, 0) 으로도 항상 원래 자세를 찾음.

        다중해가 실제 사용에서 문제가 되지 않는 근거임. 범위를 넓히면 이 성질이
        깨질 수 있으므로 그때 다시 확인할 것.
        """
        a1, a2 = ankle.solve_ik(pitch, roll)
        got = ankle.solve_fk(a1, a2)
        assert got[0] == pytest.approx(pitch, abs=1e-3)
        assert got[1] == pytest.approx(roll, abs=1e-3)

    def test_impossible_angles_do_not_return_nan(self, ankle):
        """발산한 값을 그대로 돌려주면 NaN 이 상위로 흘러감.

        그건 safety.guards 가 잡기 전까지 조용함.
        """
        with pytest.raises(AnkleUnreachableError):
            ankle.solve_fk(170.0, -170.0)

    def test_neutral(self, ankle):
        assert ankle.solve_fk(0.0, 0.0) == pytest.approx((0.0, 0.0), abs=1e-9)


# ===========================================================================
# 닿지 않는 자세
# ===========================================================================
class TestReachability:
    def test_envelope_is_enforced_by_default(self, ankle):
        """링키지가 닿는 범위보다 좁게 잡아 둠. 브링업 단계라 보수적으로 감."""
        with pytest.raises(AnkleUnreachableError, match="시험 범위"):
            ankle.solve_ik(50.0, 0.0)
        with pytest.raises(AnkleUnreachableError, match="시험 범위"):
            ankle.solve_ik(0.0, 30.0)

    def test_envelope_can_be_bypassed(self, ankle):
        """시험 범위는 소프트웨어 값이라 링키지 한계와 다름."""
        assert ankle.solve_ik(50.0, 0.0, enforce_envelope=False) is not None

    def test_custom_envelope(self):
        narrow = AnkleKinematics(envelope=AnkleEnvelope(roll_deg=(-5, 5), pitch_deg=(-5, 5)))
        narrow.solve_ik(4.0, 4.0)
        with pytest.raises(AnkleUnreachableError):
            narrow.solve_ik(6.0, 0.0)

    def test_far_pose_has_no_rod_solution(self, ankle):
        """로드가 물리적으로 닿지 않는 자세. 자르지 않고 던짐.

        발목은 두 모터가 물려 있어 한쪽만 잘리면 관절이 비틀림.
        """
        with pytest.raises(AnkleUnreachableError, match="로드 해가 없음"):
            ankle.solve_ik(80.0, 0.0, enforce_envelope=False)

    def test_error_says_which_rod_failed(self, ankle):
        with pytest.raises(AnkleUnreachableError, match=r"a1 가능=False"):
            ankle.solve_ik(80.0, 0.0, enforce_envelope=False)

    def test_is_reachable_ignores_the_envelope(self, ankle):
        assert ankle.is_reachable(50.0, 0.0) is True      # 범위 밖이지만 닿음
        assert ankle.is_reachable(80.0, 0.0) is False     # 로드가 안 닿음

    def test_envelope_is_inside_what_the_linkage_can_do(self, ankle):
        """시험 범위 네 귀퉁이가 전부 닿아야 함. 아니면 설정이 잘못된 것임."""
        e = ankle.envelope
        for pitch in e.pitch_deg:
            for roll in e.roll_deg:
                assert ankle.is_reachable(pitch, roll), f"({pitch}, {roll}) 가 안 닿음"


# ===========================================================================
# 거울상
# ===========================================================================
class TestMirror:
    @pytest.mark.parametrize("pitch,roll", POSES)
    def test_antisymmetry(self, ankle, mirrored, pitch, roll):
        """거울상.solve_ik(pitch, -roll) == -원본.solve_ik(pitch, roll)

        이게 성립해야 보행 궤적 한 벌로 양다리를 움직일 수 있음. 좌표만 뒤집으면
        근사로만 맞아 좌우가 미묘하게 다르게 움직임.
        """
        right = ankle.solve_ik(pitch, roll)
        left = mirrored.solve_ik(pitch, -roll)
        assert left[0] == pytest.approx(-right[0], abs=1e-9)
        assert left[1] == pytest.approx(-right[1], abs=1e-9)

    def test_mirror_round_trips_too(self, mirrored):
        a1, a2 = mirrored.solve_ik(20.0, 15.0)
        assert mirrored.solve_fk(a1, a2) == pytest.approx((20.0, 15.0), abs=1e-4)

    def test_reachability_is_mirrored(self, ankle, mirrored):
        assert mirrored.is_reachable(50.0, 0.0) == ankle.is_reachable(50.0, 0.0)
        assert mirrored.is_reachable(80.0, 0.0) == ankle.is_reachable(80.0, 0.0)


# ===========================================================================
# 각도 접기
# ===========================================================================
class TestWrap:
    def test_wraps_into_one_convention(self):
        from huphy.kinematics.ankle import _wrap180

        assert _wrap180(0.0) == 0.0
        assert _wrap180(190.0) == pytest.approx(-170.0)
        assert _wrap180(-190.0) == pytest.approx(170.0)
        assert _wrap180(360.0) == pytest.approx(0.0)
        assert _wrap180(-5.0) == pytest.approx(-5.0)     # 355 가 되면 안 됨

    def test_boundary(self):
        from huphy.kinematics.ankle import _wrap180

        assert _wrap180(180.0) == pytest.approx(-180.0)
        assert -180.0 <= _wrap180(179.999) < 180.0
