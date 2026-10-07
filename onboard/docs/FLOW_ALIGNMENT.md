# 전체 흐름도 준수

원본: design/whole-flowchart.mmd와 design/flow.mmd. 원본 설계는 변경하지 않았다.
사용자 추가 결정: C++ 실행 뼈대 + Python 영상 작업자, Pi에서 영상/bbox 합성, hull ROI 폐기.

| 흐름도 노드 | 현재 상태 |
|---|---|
| CompositionRoot / 환경 프로파일 | C++에서 검증·조립 |
| IClock | C++ CLOCK_MONOTONIC/CLOCK_REALTIME ns, Python 동일 host clock |
| IFrameSource / 유계 큐 | C++ SDK/V4L2/Replay, 기존 capacity 2 FIFO/drop-oldest |
| 전처리 / IDetector / YOLOX | C++ PythonVisionAdapter 경계 뒤 Python 구현 |
| BoxPostprocessor / Observation | C++ full_box 검증·관측 구성, 공개 v2 발행 |
| OverlayEncoder | Pi Python 별도 작업자, 같은 frame_id 영상과 박스, C++ 영상 발행 |
| Pub/Sub | C++ 노드 발행·구독 접점 / Python BusProxy |
| 비동기 Recorder | C++ 별도 기록 스레드 / 유계 로그 대기 공간 |
| ITracker / CameraMotion | C++ 인터페이스·팩토리 경계만. 실제 알고리즘/CMC 미구현 |
| tracking/tracks | Track/TTL 계약만. 발행 미구현 |
| GazeboPoseAdapter / Gazebo Camera | 이 온보드 패키지에 미구현. 기존 PC 시험 코드는 별도 |
| LatestStateStore / ControlContext / TargetSelector / Geometry | 이 온보드 경로에 실제 제어 구현 없음 |
| MissionStateMachine / Follower / CommandGuard / Watchdog | 이 온보드 경로에 실제 구현 없음 |
| CommandGateway | 요청/결과 자료형만. 이벤트 처리·중복 제거 미구현 |
| VehiclePort / FC telemetry | C++ 포트 계약만. 기체 연결·송신 시작하지 않음 |

원래 흐름도의 TRACK→Overlay 입력은 실제 Tracker를 붙이는 다음 단계다.
현재는 YOLOX 검출 박스를 표시한다. class_id를 track_id로 바꾸지 않는다.

상세 flow는 ByteTrack / OC-SORT / Hybrid-SORT HMIoU OFF/ON 네 개를 검증한다.
네 이름을 허용하되 실제 구현 요청에 임의 Mock을 넣지 않는다. 모두 ReID 없음, 같은 FC 회전 CMC 조건을 따른다.
Overview의 단순 비교 설명보다 상세 flow의 네 조건을 우선한다.

인지 코드에 비행 명령을 넣지 않는다. 향후 제어는 독립 주기, 최신 원자적 ControlContext,
명시적 대상 선택, 마지막 실제 검출 기준 TTL, epoch 중재, 최종 Guard와 단일 VehiclePort를 따라야 한다.
실제 노출 시각/FC 동기화가 없는 현재 상태에서 정밀 CMC나 전체 비행 흐름 준수를 완료했다고 주장하지 않는다.

## 파일 존재와 구현 상태 구분

사용자 승인으로 미구현 역할의 표시 파일을 배치했다. [목록](UNIMPLEMENTED_FILES.md)은 30개 역할/54개 파일이다.
아래 세 상태를 구분하며 앞 표의 미구현 기능이 완료된 것으로 바꾸지 않는다.

| 상태 | 의미 | 예시 |
|---|---|---|
| 구현 있음 | 실제 동작이 빌드·실행 경로에 있음 | PerceptionWorker, FrameQueue, MessageBus |
| 인터페이스/자료형만 있음 | 사용 가능한 선언은 있으나 해당 실행 구현은 없음 | ITracker, ICameraMotion, IVehiclePort, Track, CommandRequest |
| 주석만 있는 미구현 표시 파일 | 타입 선언·함수·동작·실행 연결이 없음 | Watchdog, MissionStateMachine, IFollower |

Recorder의 인지 CSV는 구현돼 있지만 전체 평가/FC/제어 지표는 미구현이다.
새 Evaluator 파일은 그 역할의 표시뿐이다. 현재 Observation은 IMAGE2D이며 POSE3D도 여전히 미구현이다.
CommandGateway는 overview의 GCS 영역 API에 해당한다. 계획된 Pi 파일은 역할 표시만 하며 실행 배치나 Windows 서비스 구현을 확정하지 않는다.
