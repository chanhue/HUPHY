"""MIT 프레임 코덱과 사양 테이블 테스트 — 하드웨어 없이 실행됨.

    PYTHONPATH=src python3 -m pytest tests -q      # python-can 불필요
"""

import math

import pytest

from huphy.motors.robstride import tables as T
from huphy.motors.robstride.codec import mit

RS02 = T.encoding_for(T.Model.RS02, T.Protocol.MIT)
NAN = float("nan")


# ===========================================================================
# tables — 사양
# ===========================================================================
class TestEncodingTables:
    def test_ranges_match_across_protocols(self):
        """260713 판본에서는 두 프로토콜의 범위가 같음.

        예전 판본은 MIT 표에 다른 숫자를 적고 있었음(RS02 속도 33). 그 값을
        그대로 두면 명령한 속도가 배율만큼 어긋남.
        """
        for model in T.Model:
            mit = T.encoding_for(model, T.Protocol.MIT)
            private = T.encoding_for(model, T.Protocol.PRIVATE)
            assert mit.pmax_rad == private.pmax_rad
            assert mit.vmax_rad_s == private.vmax_rad_s
            assert mit.tmax_nm == private.tmax_nm
            assert mit.kp_max == private.kp_max
            assert mit.kd_max == private.kd_max

    def test_bit_widths_differ(self):
        """범위는 같고 비트 폭만 다름. MIT 은 8바이트에 다섯 값을 넣어야 함."""
        m = T.encoding_for(T.Model.RS02, T.Protocol.MIT)
        p = T.encoding_for(T.Model.RS02, T.Protocol.PRIVATE)
        assert (m.vel_bits, m.tau_bits) == (12, 12)
        assert (p.vel_bits, p.tau_bits) == (16, 16)

    def test_gain_range_differs_by_model(self):
        """RS03·RS04 는 Kp 가 10배, Kd 가 20배임.

        500 으로 두고 RS04 에 kp=20 을 보내면 모터가 199 로 받음.
        """
        for model in (T.Model.RS00, T.Model.RS02):
            assert T.encoding_for(model).kp_max == 500.0
            assert T.encoding_for(model).kd_max == 5.0
        for model in (T.Model.RS03, T.Model.RS04):
            assert T.encoding_for(model).kp_max == 5000.0
            assert T.encoding_for(model).kd_max == 100.0

    def test_pmax_is_four_pi(self):
        """매뉴얼의 12.57은 4pi를 반올림한 값임 (= +-720도)."""
        assert RS02.pmax_rad == pytest.approx(4 * math.pi, abs=0.01)
        assert math.degrees(RS02.pmax_rad) == pytest.approx(720, abs=1)

    def test_default_protocol_is_mit(self):
        """본 프로젝트는 MIT를 쓰므로 기본 인자가 MIT임."""
        assert T.encoding_for(T.Model.RS02) is T.encoding_for(T.Model.RS02, T.Protocol.MIT)

    def test_missing_combination_raises(self):
        """조용히 기본값으로 때우지 않음 — 범위가 틀리면 실물에서 찾기 어려움."""
        with pytest.raises(KeyError, match="tables.py"):
            T.encoding_for(T.Model.RS00, T.Protocol.CANOPEN)

    def test_ankle_model_differs_in_torque_and_speed(self):
        """0.5 의 발목은 RS00. 각도만 RS02 와 같음."""
        rs00 = T.encoding_for(T.Model.RS00)
        assert rs00.tmax_nm == 14.0
        assert rs00.vmax_rad_s == 33.0 != RS02.vmax_rad_s
        assert rs00.pmax_rad == RS02.pmax_rad


class TestCommandBytes:
    def test_enable_and_set_mode_share_a_byte(self):
        """같은 0xFC가 두 명령을 겸함. F_CMD로 구분됨."""
        assert T.CMD_ENABLE == T.CMD_SET_MODE == 0xFC

    def test_stop_and_set_protocol_share_a_byte(self):
        assert T.CMD_STOP == T.CMD_SET_PROTOCOL == 0xFD

    def test_fault_query_differs_from_default(self):
        """F_CMD가 0xFF면 클리어, 아니면 조회임."""
        assert T.F_CMD_FAULT_QUERY != T.F_CMD_DEFAULT


# ===========================================================================
# codec — 양자화
# ===========================================================================
class TestQuantization:
    def test_round_trip(self):
        for value in (0.0, 5.0, -5.0, 12.0, -12.0):
            u = mit.float_to_uint(value, -12.57, 12.57, 16)
            back = mit.uint_to_float(u, -12.57, 12.57, 16)
            assert back == pytest.approx(value, abs=0.001)

    def test_clamps_instead_of_wrapping(self):
        """범위를 넘으면 잘림. 감싸지 않음.

        따라서 전송 전에 범위 확인이 필요함 — 넘으면 조용히 최대/최소값이 나감.
        """
        assert mit.float_to_uint(999.0, -10.0, 10.0, 12) == (1 << 12) - 1
        assert mit.float_to_uint(-999.0, -10.0, 10.0, 12) == 0

    def test_nan_passes_the_clamp(self):
        """NaN은 min/max 비교가 전부 False라 클램프를 통과함.

        결과가 최대값이 되므로 safety.guards가 미리 걸러야 함.
        이 동작을 고정해 두면 나중에 "코덱이 알아서 막겠지"라는 오해를 막을 수 있음.
        """
        assert mit.float_to_uint(NAN, -12.57, 12.57, 16) == 65535

    def test_zero_encodes_to_midpoint(self):
        """0은 범위와 무관하게 중간값이 됨.

        따라서 vmax를 44에서 33으로 고쳐도 (속도 FF가 0인) 명령 바이트는 바뀌지
        않음. 바뀌는 것은 읽기(디코딩) 쪽임.
        """
        assert mit.float_to_uint(0.0, -33.0, 33.0, 12) == 2047
        assert mit.float_to_uint(0.0, -44.0, 44.0, 12) == 2047

    def test_resolution(self):
        """위치 16bit로 +-720도 -> 약 0.022도."""
        step = math.degrees(2 * RS02.pmax_rad) / (2**16 - 1)
        assert step == pytest.approx(0.022, abs=0.001)


class TestPackCommand:
    def test_frame_is_eight_bytes(self):
        f = mit.pack_command(
            position_deg=0, velocity_deg_s=0, kp=0, kd=0, torque_nm=0, enc=RS02
        )
        assert len(f) == mit.FRAME_LEN == 8

    @pytest.mark.parametrize("pos_deg", [0.0, 45.0, -45.0, 180.0, -180.0, 700.0])
    def test_position_round_trip(self, pos_deg):
        f = mit.pack_command(
            position_deg=pos_deg, velocity_deg_s=0, kp=0, kd=0, torque_nm=0, enc=RS02
        )
        q = (f[0] << 8) | f[1]
        back = math.degrees(mit.uint_to_float(q, -RS02.pmax_rad, RS02.pmax_rad, 16))
        assert back == pytest.approx(pos_deg, abs=0.03)

    def test_gains_land_in_expected_bytes(self):
        """Kp가 Byte3 하위4 + Byte4에, Kd가 Byte5 + Byte6 상위4에 실림."""
        f = mit.pack_command(
            position_deg=0, velocity_deg_s=0, kp=500.0, kd=5.0, torque_nm=0, enc=RS02
        )
        kp_u = ((f[3] & 0x0F) << 8) | f[4]
        kd_u = (f[5] << 4) | (f[6] >> 4)
        assert kp_u == 4095        # 최대
        assert kd_u == 4095

    def test_zero_gains_are_zero(self):
        f = mit.pack_command(
            position_deg=0, velocity_deg_s=0, kp=0, kd=0, torque_nm=0, enc=RS02
        )
        assert ((f[3] & 0x0F) << 8) | f[4] == 0
        assert (f[5] << 4) | (f[6] >> 4) == 0


class TestDecodeState:
    def build(self, motor_id, pos_deg, vel_rad_s, tau_nm, temp_c):
        pos = mit.float_to_uint(math.radians(pos_deg), -RS02.pmax_rad, RS02.pmax_rad, 16)
        vel = mit.float_to_uint(vel_rad_s, -RS02.vmax_rad_s, RS02.vmax_rad_s, 12)
        tau = mit.float_to_uint(tau_nm, -RS02.tmax_nm, RS02.tmax_nm, 12)
        temp = int(temp_c * 10)
        return bytes([
            motor_id,
            (pos >> 8) & 0xFF, pos & 0xFF,
            (vel >> 4) & 0xFF,
            ((vel & 0x0F) << 4) | ((tau >> 8) & 0x0F),
            tau & 0xFF,
            (temp >> 8) & 0xFF, temp & 0xFF,
        ])

    def test_round_trip(self):
        data = self.build(10, 30.0, 1.0, 2.0, 34.5)
        mid, pos, vel, tau, temp = mit.decode_state(data, enc=RS02)
        assert mid == 10
        assert pos == pytest.approx(30.0, abs=0.05)
        assert math.radians(vel) == pytest.approx(1.0, abs=0.05)
        assert tau == pytest.approx(2.0, abs=0.02)
        assert temp == pytest.approx(34.5)

    def test_negative_values(self):
        data = self.build(7, -40.0, -2.0, -5.0, 25.0)
        _, pos, vel, tau, _ = mit.decode_state(data, enc=RS02)
        assert pos == pytest.approx(-40.0, abs=0.05)
        assert math.radians(vel) == pytest.approx(-2.0, abs=0.05)
        assert tau == pytest.approx(-5.0, abs=0.02)

    def test_rejects_short_frame(self):
        with pytest.raises(ValueError, match="8바이트"):
            mit.decode_state(bytes([1, 2, 3]), enc=RS02)

    def test_wrong_vmax_scales_velocity(self):
        """vmax가 틀리면 속도 읽기가 배율만큼 어긋남.

        private 값(44)으로 디코딩하면 실제보다 44/33 = 1.33배 크게 나옴.
        위치·토크는 영향 없음.
        """
        u = mit.float_to_uint(10.0, -33.0, 33.0, 12)
        right = mit.uint_to_float(u, -33.0, 33.0, 12)
        wrong = mit.uint_to_float(u, -44.0, 44.0, 12)
        assert right == pytest.approx(10.0, abs=0.02)
        assert wrong / right == pytest.approx(44 / 33, abs=0.01)


class TestDecodeFault:
    def test_zero_means_normal(self):
        _, word = mit.decode_fault(bytes([7, 0, 0, 0, 0, 0, 0, 0]))
        assert word == 0

    def test_bit_extraction(self):
        """bit0 = 과열."""
        mid, word = mit.decode_fault(bytes([10, 0x00, 0x00, 0x00, 0x01, 0, 0, 0]))
        assert mid == 10
        assert word & (1 << T.FAULT_BITS["overtemperature"])

    def test_high_bit(self):
        """bit14 = 스톨/과부하. 상위 바이트에 실림."""
        word = 1 << T.FAULT_BITS["stall_overload"]
        data = bytes([10, (word >> 24) & 0xFF, (word >> 16) & 0xFF,
                      (word >> 8) & 0xFF, word & 0xFF, 0, 0, 0])
        _, decoded = mit.decode_fault(data)
        assert decoded & (1 << T.FAULT_BITS["stall_overload"])

    def test_rejects_short_frame(self):
        with pytest.raises(ValueError, match="5바이트"):
            mit.decode_fault(bytes([1, 2]))
