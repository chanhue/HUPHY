# docs — 문서 목록

| 문서 | 내용 | 언제 읽나 |
|---|---|---|
| [architecture.md](architecture.md) | 계층 설계와 근거. 폴더 구조, 확장 시나리오, 이행 순서, 미결정 사항 | **먼저** — 전체 구조를 잡을 때 |
| [flow_diagrams.md](flow_diagrams.md) | 호출 관계와 흐름도 (mermaid 9개) | 구조를 그림으로 볼 때 |
| [refactor_layering.md](refactor_layering.md) | `leg_control/` → `src/huphy/` 분리 기록. 원본↔신규 대응표 | 옛 코드를 찾을 때 |
| [monitoring.md](monitoring.md) | 무엇을 모니터링하고 왜. PlotJuggler + UDP 설계, 필드 정의 | 텔레메트리 작업 시 |
| [lerobot_study_guide.md](lerobot_study_guide.md) | LeRobot 저장소를 읽으며 로봇 제어를 익히는 8단계 로드맵 | 로봇 제어가 처음일 때 |
| [build_from_scratch.md](build_from_scratch.md) | 직접 처음부터 다시 작성해보는 9단계 순서표. 단계별 완료 기준과 함정 | 손으로 익히고 싶을 때 |
| [issues.md](issues.md) | **발견한 버그·불일치·미확인 사항 로그.** 근거와 조치 계획 | 작업 중 수시로 |
| [lerobot_calibration.md](lerobot_calibration.md) | LeRobot 캘리브레이션 전체 원리. 저장소 3곳(EEPROM/메모리/JSON), 절차, 호출 시점, HUPHY와의 차이 | 캘리브레이션을 손볼 때 |

## 읽는 순서

**처음이라면**
```
lerobot_study_guide.md        남의 코드로 구조 익히기
    ↓
architecture.md               우리 구조
    ↓
flow_diagrams.md              그림으로 확인
```

**구조를 이미 안다면** `architecture.md` → `flow_diagrams.md` → 각 폴더의
[`src/huphy/*/README.md`](../src/huphy/)

## 코드 쪽 문서

폴더별 상세 설명은 소스 옆에 있다.

| 위치 | 내용 |
|---|---|
| [`README.md`](../README.md) | 계층 구조, 호출 관계, 계층별 설계 근거 |
| `src/huphy/*/README.md` | 폴더별 구성요소·호출 관계 |
| [`config/README.md`](../config/README.md) | yaml/json 작성법, 측정 방법 |
| [`tests/README.md`](../tests/README.md) | 테스트가 고정하는 것 |

## 현재 우선순위

`architecture.md` §6과 `refactor_layering.md` §7에 미결정·미완 사항이 정리되어 있다.
가장 앞선 것들:

1. **모터의 통신 프로토콜 모드 확인** — 틀리면 명령이 무시되고 에러도 안 난다
2. 모터 id ↔ 관절 매핑 실물 확인
3. 텔레메트리를 브링업에 연결 → `loop_dt` 실측
