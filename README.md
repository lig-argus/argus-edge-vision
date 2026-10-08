# argus-edge-vision

드론 탑재 컴퓨터(Raspberry Pi 5 + Hailo NPU)에서 IR 영상으로 표적을 탐지·추종하고, 지상 통제 프로그램에서 감시·명령하는 시스템.

## 폴더

| 폴더 | 내용 |
|---|---|
| `onboard/` | Raspberry Pi에 설치할 C++·Python 실행 패키지 |
| `gcs/` | 지상 통제 프로그램(GCS, Qt) |
| `common/` | onboard와 gcs가 함께 쓰는 라이브러리 (MAVLink 메시지 정의·생성 헤더, 영상 전송 형식). 표준 C++ |
| `simulation/` | 시뮬레이션 환경(PX4 SITL·Gazebo)과 실험 |
| `models/` | YOLO 학습·평가, ONNX/HEF 변환 |
| `tools/` | 헤더 생성, clang-format·clang-tidy, 저장소 규칙 검사 스크립트 |
| `docs/` | 설계, 통신 규칙(ICD), 코딩 규칙 등 각종 문서 |
| `build/` | 빌드 결과물 (git 제외): `build/onboard`, `build/gcs`, `build/simulation` |

- 프로그램 폴더(`onboard`, `gcs`)와 공용 폴더(`common`)는 같은 레벨에 둔다.
- 폴더 이름: 역할을 나타내는 폴더는 단수(`src`, `include`, `test`), 같은 종류를 모아 둔 폴더는 복수(`docs`, `tools`, `models`).
- 커밋·PR 규칙, 코딩 컨벤션: [CONTRIBUTING.md](CONTRIBUTING.md)

