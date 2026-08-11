"""Xsens MTi 시리얼(Xbus) 읽기 — **받아 온 코드를 그대로 둔 자리.**

출처: `maido-39/Huphychan-RIP_Sim2Real` 의 `utils_imu/`. 그쪽은 다시
`jiminghe/Xsens_MTi_Serial_Reader` 의 세 모듈을 감싼 것임.

파일 이름과 내용을 바꾸지 않음. 원본과 대조할 수 있어야 함 — 센서 프로토콜은
직접 확인하기 어렵고, 고쳐 두면 어디까지가 남의 코드인지 알 수 없어짐.

**딱 하나 고친 것은 import 임.** 같은 폴더 기준 평평한 import(`from SerialHandler
import ...`)를 상대 import 로 바꿈. 패키지 안에서는 그대로 두면 못 찾음.

    SerialHandler.py      시리얼 포트. `pyserial` 을 씀
    XbusPacket.py         프레임 조립·체크섬
    DataPacketParser.py   패킷 해석. `numpy` 를 씀
    SetOutput.py          센서의 출력 항목 설정
    imu_reader.py         위 셋을 감싼 리더. 백그라운드 스레드로 받음
    example_usage.py      원본 사용 예

**여기 것을 직접 쓰지 말 것.** 위쪽은 `sensors/xsens/imu.py` 의 `XsensImu` 를 씀 --
그것이 `ImuState` 로 바꿔 올림.
"""
