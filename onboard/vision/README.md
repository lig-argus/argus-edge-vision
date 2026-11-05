# onboard/vision: 탐지 프로세스

열화상 RAW14 프레임을 전처리하고 Hailo-8 에서 YOLOX 로 추론해 탐지 결과를 내는 프로세스다.
카메라 수신, 인지 큐, `perception/observations` 발행은 C++ 어댑터가, 전처리, 추론, 미리보기
인코딩은 Python 작업자가 맡는다 (`docs/design/uml/class_vision.md`).

C++ 은 다른 onboard 모듈과 같이 `include/argus/vision/`, `src/`, `test/` 에, Python 은 `python/` 에 둔다 (#35). 지금은 Python 쪽 인터페이스만 있다.

## Python (`python/`)

| 경로 | 내용 |
|---|---|
| `argus_vision/types.py` | 값 타입: `RawFrame`, `Gray8Frame`, `PixelBox`, `Classification`, `Detection` |
| `argus_vision/preprocessor.py` | `IPreprocessor`: RAW14 → 원본 크기 8 비트 흑백 |
| `argus_vision/detector.py` | `IDetector`: 흑백 프레임 → 원본 영상 픽셀 좌표의 탐지 목록 |
| `argus_vision/preview_sink.py` | `IPreviewSink`: gray8, overlay, npu_input 미리보기를 C++ 어댑터로 |
| `argus_vision_testing/fakes.py` | 테스트용 구현: `FakePreprocessor`, `FakeDetector`, `FakePreviewSink` |
| `test/` | 값 타입과 Fake 의 테스트 (표준 라이브러리 `unittest`) |

- 인터페이스와 값 타입은 numpy, OpenCV, HailoRT, ZeroMQ 에 의존하지 않는다. 구현(`DoublePlateauPreprocessor`, `HailoDetector`, `ZmqPreviewSink`)은 다음 이슈에서 RPi 의 현재 코드를 옮겨 온다.
- 이름은 CONTRIBUTING 0.2 "클래스 이름 짓기"를 따른다 (인터페이스 `I…`, 구현 "방식 + 이름", 테스트용 `Fake…`).

### 테스트

```bash
cd onboard/vision/python
python3 -m unittest discover -s test
```

형식, 규칙, 타입 검사는 저장소 공통 규칙(CONTRIBUTING 0.2, 0.3)대로 커밋 hook 과 CI 의 python lint 작업이 한다.
