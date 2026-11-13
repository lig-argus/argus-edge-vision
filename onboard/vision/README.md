# onboard/vision: 탐지 프로세스

열화상 RAW14 프레임을 전처리하고 Hailo-8 에서 YOLOX 로 추론해 탐지 결과를 내는 프로세스다.
카메라 수신, 인지 큐, `perception/observations` 발행은 C++ 어댑터가, 전처리, 추론, 미리보기
인코딩은 Python 작업자가 맡는다 (`docs/design/uml/class_vision.md`).

C++ 은 다른 onboard 모듈과 같이 `include/argus/vision/`, `src/`, `test/` 에, Python 은 `python/` 에 둔다 (#35). 지금은 Python 쪽(인터페이스와 구현)만 있고, C++ 어댑터와 카메라 수신은 다음 PR 에서 옮긴다.

## Python (`python/`)

| 경로 | 내용 |
|---|---|
| `argus_vision/types.py` | 값 타입: `RawFrame`, `Gray8Frame`, `PixelBox`, `Classification`, `Detection` |
| `argus_vision/preprocessor.py` | `IPreprocessor`: RAW14 → 원본 크기 8 비트 흑백 |
| `argus_vision/detector.py` | `IDetector`: 흑백 프레임 → 원본 영상 픽셀 좌표의 탐지 목록 |
| `argus_vision/preview_sink.py` | `IPreviewSink`: gray8, overlay, npu_input 미리보기를 C++ 어댑터로 |
| `argus_vision/double_plateau.py` | `DoublePlateauPreprocessor`: Double Plateau adaptive (Python 경로) |
| `argus_vision/plateau_native.py`, `native/` | 같은 전처리의 C++17 + OpenMP 라이브러리와 ctypes 호출 (출력이 Python 경로와 같음) |
| `argus_vision/model_input.py` | HEF 입력 만들기(1채널 같은 크기는 그대로, 아니면 레터박스)와 박스 좌표 되돌리기 |
| `argus_vision/hailo_detector.py` | `HailoDetector`: HailoRT 4.23 비동기 추론, HEF 안 NMS 출력 해석 |
| `argus_vision/zmq_preview_sink.py` | `ZmqPreviewSink`: ipc PUSH(SNDHWM 4), 별도 인코딩 스레드, 토픽별 FPS 상한 |
| `argus_vision/worker.py` | `PythonVisionWorker`: 전처리 → 추론 → 미리보기 순서 |
| `argus_vision/zmq_worker_server.py`, `__main__.py` | C++ 어댑터의 ipc REQ 를 받는 REP 서버(`perception_ipc.proto`)와 실행 진입점 `python -m argus_vision` |
| `argus_vision/messages.py` | 탐지 결과를 protobuf 메시지(`detection.proto`)로 바꾸기 |
| `argus_vision_testing/fakes.py` | 테스트용 구현: `FakePreprocessor`, `FakeDetector`, `FakePreviewSink` |
| `test/` | 표준 라이브러리 `unittest` |

- 인터페이스와 값 타입(`types.py`, `preprocessor.py`, `detector.py`, `preview_sink.py`)은 numpy, OpenCV, HailoRT, ZeroMQ 에 의존하지 않는다. 구현은 RPi 탐지 venv 의 numpy 1.26, OpenCV, pyzmq, protobuf, HailoRT Python 4.23 을 쓴다.
- 메시지는 protobuf 다. 정의는 `onboard/libs/protocol/proto/argus_msg/` 의 `vision_frame_gray8.proto`, `vision_overlay_jpeg.proto`, `vision_npu_input_jpeg.proto`, `perception_ipc.proto`(탐지 프로세스 안의 ipc), 함께 쓰는 탐지 결과 `detection.proto` 이고, Python 생성 코드는 `onboard/libs/protocol/python/argus_msg/` 에 있다 (`tools/generate_proto.sh`).
- 구현은 실기체 RPi `~/ARGUS` 의 탐지 코드(`src/argus/perception/`, `vendor/plateau_native/`)를 옮긴 것이다. 같은 녹화를 넣으면 흑백 영상과 탐지 결과가 같다.
- 이름은 CONTRIBUTING 0.2 "클래스 이름 짓기"를 따른다 (인터페이스 `I…`, 구현 "방식 + 이름", 테스트용 `Fake…`).

### 실행

C++ 어댑터가 `python -m argus_vision --endpoint … --preview-endpoint … --session-id … --source … --backend hailo --hef … --labels … --score-threshold 0.7` 로 띄운다 (`PYTHONPATH` 에 `onboard/vision/python` 과 `onboard/libs/protocol/python`). 클래스별 경계값은 `--class-score-thresholds 0.5,0.45,0.35`(라벨 순서), 클래스 사이 겹침 제거는 `--class-suppress tank:car:0.3` 으로 주며, 인자가 없으면 실기체와 같은 환경 변수를 읽는다. 미리보기는 `--publish-preview`(overlay), `--publish-npu-input`, `--publish-gray8` 로 켠다. `vision/frame/gray8` 은 기본 꺼짐이며, 지금 RPi 의 C++ 어댑터는 이 토픽을 받지 않으므로 어댑터를 옮긴 뒤 켠다.

| 환경 변수 | 뜻 |
|---|---|
| `ARGUS_NATIVE_PREPROCESS` | `1` 이면 C++ 전처리. 라이브러리를 못 불러오면 Python 경로로 하고 stderr 에 남긴다 |
| `ARGUS_NATIVE_THREADS` | C++ 전처리의 OpenMP 스레드 수 (기본 4). 다른 프로세스와 CPU 를 나눌 때 느려지지 않게 `OMP_WAIT_POLICY=passive` 를 같이 둔다 |
| `ARGUS_CLASS_SCORE_THRESHOLDS` | `--class-score-thresholds` 의 기본값. 비우면 모든 클래스에 `--score-threshold` |
| `ARGUS_CLASS_SUPPRESS` | `--class-suppress` 의 기본값. 예: `tank:car:0.3` 은 탱크 박스와 IoU 0.3 이상 겹친 차 박스를 지운다. 비우면 끔 |
| `ARGUS_PLATEAU_NATIVE_LIB` | 라이브러리 경로 (기본 `python/native/build/libplateau_native.so`) |

C++ 전처리 라이브러리는 RPi 에서 만든다 (저장소 CI 는 만들지 않는다):

```bash
onboard/vision/python/native/build.sh
```

### 테스트

```bash
cd onboard/vision/python/test
PYTHONPATH=..:../../../libs/protocol/python python3 -m unittest discover
```

- 구현 테스트는 numpy, OpenCV, pyzmq, protobuf 가 있는 환경(RPi 탐지 venv)에서 돈다. HailoRT 가 없어도 되며 NPU 추론 자체는 시험하지 않는다 (NMS 해석과 좌표 변환은 시험한다).
- `test_double_plateau.py` 는 RPi 의 원래 식(np.percentile, np.median)과 Python 경로, C++ 경로의 출력을 바이트 단위로 비교한다. C++ 경로는 라이브러리가 있을 때만 돈다.

형식, 규칙, 타입 검사는 저장소 공통 규칙(CONTRIBUTING 0.2, 0.3)대로 커밋 hook 과 CI 의 python lint 작업이 한다. CI 환경에 없는 numpy, OpenCV, pyzmq, HailoRT 는 `pyproject.toml` 의 mypy 예외로 둔다.
