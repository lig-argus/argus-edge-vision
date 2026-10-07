# ARGUS — C++ 실행부 + Python 영상 작업자

Raspberry Pi 5 + Hailo-8용 인지 패키지다. 실행 진입은 `build/bin/argus-onboard`
(OnboardMain → CompositionRoot → PerceptionWorker)이며, 현재 기능 범위는 **perception_only**다.

이 폴더는 Raspberry Pi에 설치하는 독립 실행 패키지다. 저장소의 다른 최상위 폴더 없이
빌드·실행할 수 있도록 CMake/Python 정의, 실행 코드, 프로파일·라벨, 스크립트와 문서를 함께 둔다.
실제 `config/runtime.env`, HEF·배포 라벨, `.venv-rpi`, `build/`는 각 Pi에 생성·배치하며 Git에서 제외한다.
HailoRT/PyHailoRT와 카메라 SDK는 각 Pi에 별도로 준비해야 한다.

## Raspberry Pi 최초 설치

```bash
git clone --filter=blob:none --sparse -b develop \
  https://github.com/s-jje/argus-edge-vision.git
cd argus-edge-vision
git sparse-checkout set --cone onboard
cd onboard
```

테스트 Pi는 `develop`, 드론 장착 Pi는 `main` 또는 검증한 릴리즈 태그를 사용한다.
[설치·실행 안내](docs/INSTALL_AND_RUN_RPI5_HAILO8.md)에 따라 의존성 준비와 빌드를 수행하고,
[실행용 모델 안내](models/README.md)에 따라 검증한 HEF·라벨을 함께 `models/deployed/`에 배치한다.
`setup_rpi.sh`는 Python 의존성·HailoRT·SDK를 자동 설치하지 않는다.

## 실행 흐름

```text
C++ SDK/V4L2/Replay → FrameQueue(기본 2, FIFO/drop-oldest)
  → private IPC → Python RAW14/letterbox → YOLOX/Hailo
  → C++ full_box 검증·Observation 구성 → MessageBus → Python BusProxy → 구독자

같은 프레임·검출 → Pi Python 박스 그리기/JPEG → C++ 발행 → PC 미리보기
```

C++가 카메라 I/O·인지 큐·공개 발행·프로세스 생명주기를 소유한다.
SDK/OpenCV 캡처는 별도 소유 프로세스이며, Python 추론 요청은 한 번에 하나다.
JPEG는 별도 Python 스레드/최신 1개 미리보기 큐와 C++ 발행 스레드를 사용한다.
인지 CSV Recorder도 별도 스레드와 유계 대기 공간을 사용하며 초과 폐기 수를 기록한다.

현재 발행은 관측·진단·옵션 JPEG다. 실제 Tracker, FC 회전 CMC, POSE3D, 제어/FSM,
Guard/Watchdog, 기체 연결·명령 송신은 미구현이다. 30개 역할의 표시 파일 54개는
주석뿐이며, 기존 인터페이스나 파일 존재를 실행 기능으로 간주하지 않는다.

## 폴더

| 경로 | 내용 |
|---|---|
| `cpp/` | C++17 진입점, 계약·포트, 실행 조립, 입력·IPC·버스와 미구현 표시 파일 |
| `python/argus_workers/` | 전처리·YOLOX·JPEG, BusProxy, 점검·구독 도구 |
| `config/` | 프로파일·라벨 및 각 Pi에서 생성하는 runtime.env의 위치 |
| `models/` | 검증한 실행용 HEF·라벨 세트의 배치 위치 |
| `scripts/`, `deploy/` | 빌드·설치·실행 스크립트와 systemd 템플릿 |
| `cpp/tests/`, `tests/integration/` | C++ 계약 검사와 C++/Python/Bus 통합 검사 |
| `docs/design/` | 전체·상세 설계 흐름도 원본 |
| `legacy/` | 실행하지 않는 Python 원본과 과거 문서 |

## 문서 안내

작업 전 README → [AGENTS](AGENTS.md) → [C++ 전환 결정](docs/CPP_MIGRATION.md)
→ [흐름도 준수](docs/FLOW_ALIGNMENT.md)를 읽는다.

| 목적 | 문서 |
|---|---|
| 작업 규칙·승인 범위 | [AGENTS.md](AGENTS.md) |
| C++/Python 책임과 원본 보관 결정 | [CPP_MIGRATION.md](docs/CPP_MIGRATION.md) |
| 현재 구현 상태·설계 불변 조건 | [FLOW_ALIGNMENT.md](docs/FLOW_ALIGNMENT.md) |
| 기존 코드 위치 | [FILE_MAP.md](docs/FILE_MAP.md) |
| 미구현 30개 역할·설계 조건 | [UNIMPLEMENTED_FILES.md](docs/UNIMPLEMENTED_FILES.md) |
| 공개 토픽·필드·내부 IPC | [BUS_CONTRACT.md](docs/BUS_CONTRACT.md) |
| 입력·전처리·모델 점검 | [CAMERA_AND_MODEL_NOTES.md](docs/CAMERA_AND_MODEL_NOTES.md) |
| 의존성·빌드·실행·미리보기·검증 명령 | [INSTALL_AND_RUN_RPI5_HAILO8.md](docs/INSTALL_AND_RUN_RPI5_HAILO8.md) |
| Pi 검증 결과·백업·미검증 범위 | [VALIDATION.md](docs/VALIDATION.md) |

[과거 기록](legacy/README.md)은 당시 상태의 참고용이며 현행 실행 지침이 아니다.
