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

## onboard 구조와 빌드 (`onboard/`)

Raspberry Pi 에서 도는 프로세스 5개(onboard, 영상 송신부, 추종 제어, 추적기, 탐지)의 코드를 모두 `onboard/` 아래에 둔다.
`onboard/` 는 RPi 패키지 전체를 가리키고, onboard 프로세스(FC 통신·통제기 통신)는 `onboard/apps/onboard/` 이다.

```
onboard/
  CMakeLists.txt          # 상위 CMake 프로젝트 하나, 실행 파일 여러 개
  cmake/                  # CMake 함수 (argus_add_test)
  libs/                   # 프로세스들이 함께 쓰는 라이브러리
    core/                 # IClock·IPoller·EventLoop·Subscription 공통 기반
    core_testing/         # core 의 테스트용 구현 (FakeClock, FakePoller)
    bus/                  # ZeroMQ 토픽·RPC 인터페이스 (바이트만 나름)
    bus_testing/          # FakeBus
    protocol/             # 토픽·RPC 이름, 기본 엔드포인트 상수
    vehicle_api/          # IVehicleTelemetry·IVelocitySetpointSink·IFlightModeControl(Client)·IFollowControl(Client) 인터페이스 (core 에만 의존)
    vehicle_api_testing/  # vehicle_api 의 테스트용 구현
    vehicle_api_zmq/      # vehicle_api 의 ZeroMQ 어댑터 (vehicle_api·bus 링크, 다음 PR)
  apps/                   # 실행 파일. 프로세스 하나에 폴더 하나
    onboard/              # onboard 프로세스 (통신 담당)
    video_sender/         # 영상 송신부 (통신 담당)
    follow_controller/    # 추종 제어 (추적·추종 담당)
    tracker/              # UCMC 추적기 (추적·추종 담당)
  vision/                 # 탐지 프로세스. Python + C++ 어댑터 (탐지 담당)
  config/                 # 설정 파일 (mavlink-router, onboard, 센서 대응표)
```

- RPi 프로세스끼리만 사용하는 것은 `onboard/libs/`, onboard 와 통제기가 함께 사용하는 것(MAVLink, 영상 메타데이터 형식)은 `common/` 에 둔다.
- 모듈 규칙(폴더, 헤더 경로, namespace, 테스트 등록)은 CONTRIBUTING.md 0.4.
- 클래스와 인터페이스 설계는 `docs/design/uml/`.

의존성은 Ubuntu 24.04 apt 패키지를 사용한다 (RPi 와 CI 가 같음).

| OS | 설치 |
|---|---|
| Ubuntu 24.04 (RPi, CI) | `sudo apt install cmake ninja-build libzmq3-dev cppzmq-dev libmsgpack-cxx-dev libgtest-dev` |
| macOS (개발용) | `brew install cmake ninja zeromq cppzmq msgpack-cxx googletest` |

```bash
cmake -S onboard -B build/onboard -G Ninja
cmake --build build/onboard
ctest --test-dir build/onboard --output-on-failure
```

## 통제기 빌드 (`gcs/`)

Qt 6.2 이상(Widgets, Network, Test)이 필요하다. onboard와 별도 CMake 프로젝트다.

```bash
cmake -S gcs -B build/gcs -DCMAKE_PREFIX_PATH=/opt/homebrew   # macOS (brew install qt). Windows: C:/Qt/6.x/msvc2022_64
cmake --build build/gcs
ctest --test-dir build/gcs
./build/gcs/argus_gcs
```

## MAVLink 메시지 (`common/mavlink/`)

onboard와 통제기가 주고받는 메시지는 `common/mavlink/message_definitions/argus.xml`(argus dialect)에 정의하고, 여기서 생성한 C 헤더 `common/mavlink/generated/`를 저장소에 커밋한다.
표준 정의(`common.xml`·`minimal.xml`·`standard.xml`)는 기체 FC(KoaFC, PX4 v1.16.0)의 MAVLink 버전을 그대로 사용한다.

```bash
# 최초 실행: 로컬 가상환경에서 아래 명령 실행. pymavlink 버전은 tools/requirements-mavgen.txt 에 명시
python3 -m venv ~/.venvs/mavgen && ~/.venvs/mavgen/bin/pip install -r tools/requirements-mavgen.txt

# 재생성: argus.xml 수정한 뒤 아래 명령 실행. 스크립트가 common/mavlink/generated/ 를 지우고 다시 생성한다.
MAVGEN_PYTHON=~/.venvs/mavgen/bin/python tools/generate_mavlink.sh
```

- 순서: `argus.xml` 수정 → 재생성 → `docs/spec/icd.md` 수정 → onboard·통제기 코드 수정. 한 PR 에 함께 넣는다.
- `common/mavlink/generated/`는 직접 수정하지 않는다. CI 의 `mavlink` 작업이 같은 버전으로 다시 생성해 커밋본과 비교하므로, 재생성을 하지 않으면 PR 이 실패한다.
- 통째로 다시 만드는 이유: `argus.xml` 에서 메시지를 지웠을 때 옛 헤더가 남지 않게 한다.
- pymavlink 버전을 올릴 때: `tools/requirements-mavgen.txt` 를 수정하고 재생성해 헤더 변경과 한 커밋으로 올린다.
- Windows: Git Bash 에서 같은 명령 실행. 가상환경의 python 경로는 `~/.venvs/mavgen/Scripts/python` 이다.
