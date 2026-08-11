"""
imu_reader.py

Xsens MTi 시리즈 IMU(MTi-680G 등)를 시리얼(Xbus 프로토콜)로 읽는
재사용 가능한 클래스.

https://github.com/jiminghe/Xsens_MTi_Serial_Reader 의
SerialHandler / XbusPacket / DataPacketParser 모듈을 감싸서,
백그라운드 스레드에서 계속 데이터를 받고 메인 스레드에서는
get_data()로 최신 값만 꺼내 쓰는 구조로 정리했습니다.

이 파일과 함께 아래 파일들을 같은 폴더에 복사해서 써야 합니다
(원본 레포에서 그대로 가져오면 됩니다):
    SerialHandler.py
    XbusPacket.py
    DataPacketParser.py
    SetOutput.py   (출력 설정을 직접 지정하고 싶을 때만 필요)

사용 예:
    from imu_reader import ImuReader

    imu = ImuReader(port="/dev/xsens_mti", baudrate=921600)
    imu.start()
    try:
        while True:
            data = imu.get_data()
            if data:
                print(data.get("euler"), data.get("acceleration"))
            time.sleep(0.01)
    finally:
        imu.stop()

    # 또는 컨텍스트 매니저로:
    with ImuReader("/dev/xsens_mti", 921600) as imu:
        data = imu.get_data()
"""

import threading
import time

from .SerialHandler import SerialHandler
from .XbusPacket import XbusPacket
from .DataPacketParser import DataPacketParser, XsDataPacket


class ImuReader:
    """Xsens MTi IMU를 백그라운드 스레드로 읽는 리더 클래스."""

    GO_TO_CONFIG = bytes.fromhex('FA FF 30 00')
    GO_TO_MEASUREMENT = bytes.fromhex('FA FF 10 00')

    def __init__(self, port, baudrate=921600, output_config_fn=None, on_data=None):
        """
        port: 시리얼 장치 경로, 예) "/dev/xsens_mti" 또는 "COM3"
        baudrate: 센서에 설정된 실제 보드레이트와 일치해야 함
        output_config_fn: 측정 모드 진입 전 출력 설정을 바꾸고 싶을 때
            전달하는 콜백. serial_handler를 인자로 받는 함수.
            (예: SetOutput.set_output_conf 를 그대로 넘기면 됨)
            None이면 센서에 이미 저장된 출력 설정을 그대로 사용.
        on_data: 매 패킷마다 호출할 콜백(dict). 백그라운드 스레드에서
            호출되므로, 콜백 내부에서 UI를 직접 건드리지 말 것.
        """
        self._port = port
        self._baudrate = baudrate
        self._output_config_fn = output_config_fn
        self._on_data = on_data

        self._serial = None
        self._packet = None
        self._thread = None
        self._running = False

        self._lock = threading.Lock()
        self._latest = None

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------

    def start(self):
        """시리얼 포트를 열고, 설정 후 측정 모드로 진입해 스트리밍을 시작."""
        if self._running:
            return

        self._serial = SerialHandler(self._port, self._baudrate)

        self._serial.send_with_checksum(self.GO_TO_CONFIG)
        time.sleep(0.1)

        if self._output_config_fn is not None:
            self._output_config_fn(self._serial)
            time.sleep(0.1)

        self._packet = XbusPacket(on_data_available=self._on_packet)

        self._serial.send_with_checksum(self.GO_TO_MEASUREMENT)

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """리더 스레드를 정지하고 시리얼 포트를 닫음."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._serial is not None:
            close = getattr(self._serial, "close", None)
            if callable(close):
                close()
            self._serial = None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ------------------------------------------------------------------
    # internals
    # ------------------------------------------------------------------

    def _read_loop(self):
        while self._running:
            try:
                byte = self._serial.read_byte()
                self._packet.feed_byte(byte)
            except RuntimeError:
                # 개별 바이트 읽기 오류는 건너뛰고 계속 진행
                continue
            except Exception:
                if self._running:
                    raise

    def _on_packet(self, packet):
        xbus_data = XsDataPacket()
        DataPacketParser.parse_data_packet(packet, xbus_data)
        data = self._to_dict(xbus_data)

        with self._lock:
            self._latest = data

        if self._on_data is not None:
            self._on_data(data)

    @staticmethod
    def _to_dict(x):
        """XsDataPacket을 평범한 dict로 변환. 현재 출력 설정에서
        실제로 전송된 필드만 포함됨."""
        data = {}
        if getattr(x, "packetCounterAvailable", False):
            data["packet_counter"] = x.packetCounter
        if getattr(x, "sampleTimeFineAvailable", False):
            data["sample_time_fine"] = x.sampleTimeFine
        if getattr(x, "utcTimeAvailable", False):
            data["utc_time"] = x.utcTime
        if getattr(x, "eulerAvailable", False):
            data["euler"] = tuple(x.euler)          # (roll, pitch, yaw) deg
        if getattr(x, "quaternionAvailable", False):
            data["quaternion"] = tuple(x.quat)       # (q0, q1, q2, q3)
        if getattr(x, "rotAvailable", False):
            data["rate_of_turn"] = tuple(x.rot)      # rad/s
        if getattr(x, "accAvailable", False):
            data["acceleration"] = tuple(x.acc)      # m/s^2
        if getattr(x, "magAvailable", False):
            data["magnetic_field"] = tuple(x.mag)
        if getattr(x, "latlonAvailable", False) and getattr(x, "altitudeAvailable", False):
            data["lat_lon"] = tuple(x.latlon)
            data["altitude"] = x.altitude
        if getattr(x, "velocityAvailable", False):
            data["velocity"] = tuple(x.vel)
        if getattr(x, "temperatureAvailable", False):
            data["temperature"] = x.temperature
        if getattr(x, "baropressureAvailable", False):
            data["baro_pressure"] = x.baropressure
        if getattr(x, "deltaVAvailable", False):
            data["delta_v"] = tuple(x.deltaV)
        if getattr(x, "deltaQAvailable", False):
            data["delta_q"] = tuple(x.deltaQ)

        data["timestamp"] = time.time()
        return data

    # ------------------------------------------------------------------
    # public data access
    # ------------------------------------------------------------------

    def get_data(self):
        """가장 최근 파싱된 패킷의 스냅샷 dict를 반환. 아직 아무것도
        수신하지 못했으면 None."""
        with self._lock:
            return dict(self._latest) if self._latest is not None else None

    def is_running(self):
        return self._running
