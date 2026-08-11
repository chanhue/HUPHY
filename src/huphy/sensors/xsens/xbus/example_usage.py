"""
example_usage.py

ImuReader 클래스 사용 예제. imu_reader.py와 SerialHandler.py,
XbusPacket.py, DataPacketParser.py를 같은 폴더에 두고 실행하세요.
"""

import time

from .imu_reader import ImuReader

PORT = "/dev/xsens_mti"   # udev로 고정해둔 심볼릭 링크 경로
BAUDRATE = 921600


def main():
    imu = ImuReader(port=PORT, baudrate=BAUDRATE)
    imu.start()

    try:
        while True:
            data = imu.get_data()
            if data is not None:
                euler = data.get("euler")
                acc = data.get("acceleration")
                if euler and acc:
                    print(
                        f"Roll={euler[0]:.2f} Pitch={euler[1]:.2f} Yaw={euler[2]:.2f} | "
                        f"Ax={acc[0]:.2f} Ay={acc[1]:.2f} Az={acc[2]:.2f}"
                    )
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        imu.stop()


if __name__ == "__main__":
    main()
