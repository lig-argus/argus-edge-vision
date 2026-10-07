# 기존 구현·인터페이스 파일 지도

C++는 contracts → ports → application → adapters 경계로 구성하며 실행 조립은 CompositionRoot가 담당한다.
아래는 기존 구현과 선언의 위치다. 링크의 파일명으로 역할을 구분한다.

| 흐름도 역할 | 현행 파일 |
|---|---|
| 환경 프로파일 | [EnvironmentProfile.hpp](../cpp/include/argus/application/EnvironmentProfile.hpp) / [EnvironmentProfile.cpp](../cpp/src/application/EnvironmentProfile.cpp) |
| CompositionRoot | [CompositionRoot.hpp](../cpp/include/argus/application/CompositionRoot.hpp) / [CompositionRoot.cpp](../cpp/src/application/CompositionRoot.cpp) |
| 실행 진입 | [OnboardMain.cpp](../cpp/apps/OnboardMain.cpp) |
| IClock | [IClock.hpp](../cpp/include/argus/ports/IClock.hpp) / [SystemClock.cpp](../cpp/src/contracts/SystemClock.cpp) |
| 프레임/시각 | [Frame.hpp](../cpp/include/argus/contracts/Frame.hpp) |
| IFrameSource | [IFrameSource.hpp](../cpp/include/argus/ports/IFrameSource.hpp) |
| SDK 입력 | [I3SdkFrameSource.hpp](../cpp/include/argus/adapters/I3SdkFrameSource.hpp) / [I3SdkFrameSource.cpp](../cpp/src/adapters/I3SdkFrameSource.cpp) |
| V4L2/Replay 입력 | [OpenCvFrameSource.hpp](../cpp/include/argus/adapters/OpenCvFrameSource.hpp) / [OpenCvFrameSource.cpp](../cpp/src/adapters/OpenCvFrameSource.cpp) |
| C++ 카메라 읽기 helper | [FrameSourceCapture.cpp](../cpp/apps/FrameSourceCapture.cpp) |
| 유계 큐 | [FrameQueue.hpp](../cpp/include/argus/application/FrameQueue.hpp) |
| 인지 실행 | [PerceptionWorker.hpp](../cpp/include/argus/application/PerceptionWorker.hpp) / [PerceptionWorker.cpp](../cpp/src/application/PerceptionWorker.cpp) |
| 전처리 | [Preprocessor.py](../python/argus_workers/perception/Preprocessor.py) |
| IDetector | [IDetector.hpp](../cpp/include/argus/ports/IDetector.hpp) |
| Python 연결 | [PythonVisionAdapter.hpp](../cpp/include/argus/adapters/PythonVisionAdapter.hpp) / [PythonVisionAdapter.cpp](../cpp/src/adapters/PythonVisionAdapter.cpp) |
| Python 영상 요청 처리 | [PythonVisionWorker.py](../python/argus_workers/perception/PythonVisionWorker.py) |
| YOLOX | [YOLOXDetector.py](../python/argus_workers/perception/YOLOXDetector.py) |
| BoxPostprocessor | [BoxPostprocessor.hpp](../cpp/include/argus/application/BoxPostprocessor.hpp) |
| Observation | [Observation.hpp](../cpp/include/argus/contracts/Observation.hpp) |
| Track/TTL | [Track.hpp](../cpp/include/argus/contracts/Track.hpp) |
| OverlayEncoder | [OverlayEncoder.py](../python/argus_workers/perception/OverlayEncoder.py) |
| Pub/Sub | [MessageBus.hpp](../cpp/include/argus/adapters/MessageBus.hpp) / [MessageBus.cpp](../cpp/src/adapters/MessageBus.cpp) |
| 버스 중계 | [BusProxy.py](../python/argus_workers/transport/BusProxy.py) |
| Recorder | [Recorder.hpp](../cpp/include/argus/application/Recorder.hpp) / [Recorder.cpp](../cpp/src/application/Recorder.cpp) |
| CommandGateway 자료형 | [CommandRequest.hpp](../cpp/include/argus/contracts/CommandRequest.hpp) |
| Tracker 계약만 | [ITracker.hpp](../cpp/include/argus/ports/ITracker.hpp) |
| CameraMotion 계약만 | [ICameraMotion.hpp](../cpp/include/argus/ports/ICameraMotion.hpp) |
| VehiclePort 계약만 | [IVehiclePort.hpp](../cpp/include/argus/ports/IVehiclePort.hpp) |
| 미리보기 수신 | [PreviewSubscriber.py](../python/argus_workers/tools/PreviewSubscriber.py) |

ITracker/ICameraMotion/IVehiclePort와 Track/CommandRequest는 계약·자료형이며 해당 실행 기능은 없다.
현재 구현 상태는 [FLOW_ALIGNMENT](FLOW_ALIGNMENT.md)에서 확인한다.

주석뿐인 **30개 역할/54개 파일**의 위치·설계 노드·조건은
[UNIMPLEMENTED_FILES](UNIMPLEMENTED_FILES.md)에서만 관리한다.
이 파일들을 인터페이스 선언이나 빌드·팩토리에 등록된 구현으로 간주하지 않는다.

기존 PC C++ 시험 모듈 위치는 `/home/user/project/코드/px4_follow_sitl/cpp`다.
이 패키지의 실제 IR 경로와 PC의 단일 박스/AlphaBeta 시험 경로는 별도 구현이다.
