# 미구현 파일 지도 — 사용자 승인, 2026-09-26

승인한 것은 **30개 역할의 주석 전용 파일 54개 배치**다. 기능 구현 승인이 아니다.
헤더만 6개, 헤더/소스 쌍 24개이며 모두 `/*미구현 임의 생성 금지*/`와
`UNIMPLEMENTED_PLACEHOLDER` 설명뿐이다. 새 헤더도 타입·함수를 선언하지 않는다.

사용자 명시적 승인 없이 알고리즘·API·필드·수치·기본 반환·Mock을 만들거나
CMake·팩토리·실행 스크립트·버스 경로에 연결하지 않는다.
기존 구현/인터페이스/자료형과 .py.org 원본을 유지한다.
일반 기존 버그 수정과 미구현 기능 작성을 구분하며 [AGENTS](../AGENTS.md)를 따른다.

## 읽는 방법

전체 설계는 [whole-flowchart.mmd](design/whole-flowchart.mmd),
세부 조건은 [flow.mmd](design/flow.mmd)가 기준이다.
아래 전체/상세 노드는 원본 설계의 ID다. 각 파일의 역할별 설계 조건은 모두 유지했다.

표의 폴더는 `cpp/include/argus/`와 `cpp/src/` 아래 상대 경로다.
파일명에 대응하는 hpp/cpp 링크를 사용한다. 헤더만 있는 항목은 소스도 API도 없다.
미구현 표시 파일은 확장자 앞 파일명 끝에 `_TBD`를 붙인다(예: `Watchdog_TBD.hpp`).
기존 구현·포트의 위치는 [FILE_MAP](FILE_MAP.md)에 있다.

## ports

| 역할·파일 | 설계 노드 | 조건 |
|---|---|---|
| **IFollower** ([hpp](../cpp/include/argus/ports/IFollower_TBD.hpp))<br>유효한 TargetState를 사용하는 추종 포트 | 전체: `FOLLOW`<br>상세: `GUIDANCE, FOLLOW_INTENT, PRED_INTENT, SCAN_INTENT` | 실제 포트 선언과 호출 계약은 아직 없다. 유효 TargetState와 동일 ControlContext를 사용해야 한다.<br>인지 프레임 루프에서 명령을 생성하지 않으며 최종 CommandGuard와 단일 IVehiclePort를 거쳐야 한다. |

## contracts

| 역할·파일 | 설계 노드 | 조건 |
|---|---|---|
| **TargetState** ([hpp](../cpp/include/argus/contracts/TargetState_TBD.hpp))<br>운용자가 승인한 선택 ID와 대상 유효성의 계약 | 전체: `STORE, TARGET, FOLLOW`<br>상세: `PREP, PREP_CHECK, COMMIT, SNAP, FGATE, TRACK, TTL, CONF` | 준비 단계의 후보 ID는 pending이다. 승인된 시작 사건에서만 선택 ID와 임무 상태를 원자적으로 확정한다.<br>최신 실제 검출과 선택 ID의 연결을 확인하고 겹침·ID 불확실성을 임의로 승인하지 않는다.<br>Predicted 결과나 새 배치 발행으로 마지막 실제 검출 시각과 TTL을 갱신하지 않는다. 타입·필드 선언은 미구현이다. |
| **ControlContext** ([hpp](../cpp/include/argus/contracts/ControlContext_TBD.hpp))<br>한 제어 주기에서 함께 사용하는 원자적 상태 스냅샷 계약 | 전체: `LOOP, GUARD, STORE`<br>상세: `SNAP, MSTATE, ARBITER, GUARD` | 임무 상태·선택 ID·재개차단·관측·FC 상태·epoch를 같은 원자적 스냅샷으로 다룬다.<br>제어 판단과 최종 Guard가 같은 ControlContext를 사용한다. 구체적인 필드와 동기화 구현은 미정이다. |
| **VehicleState** ([hpp](../cpp/include/argus/contracts/VehicleState_TBD.hpp))<br>FC 자세·위치·속도·실제 모드·시각·유효성 계약 | 전체: `MAV, STORE, HEALTH, STATUS`<br>상세: `FC, FHEALTH, FEXIT, SCHECK, WCHECK` | 실제 텔레메트리의 시각과 유효성을 구분한다. 모드 요청 성공을 실제 모드 확인으로 간주하지 않는다.<br>노출 시각을 FC 자세와 정렬해야 한다. 호스트 수신 시각을 실제 노출 시각으로 대신하지 않는다. |
| **GuidanceIntent** ([hpp](../cpp/include/argus/contracts/GuidanceIntent_TBD.hpp))<br>중재 전 가이던스 의도 계약 | 전체: `FOLLOW, GUARD`<br>상세: `FOLLOW_INTENT, PRED_INTENT, SCAN_INTENT, ARBITER, GUARD` | 상세 flow의 mode·reason·네 축 setpoint 의도와 epoch/만료 연결은 구현 승인 시 확정한다.<br>기존 IVehiclePort.hpp의 ControlIntent를 재정의하거나 덮어쓰지 않는다.<br>Predicted는 수평·yaw 0, Scan은 전진·횡 0이다. 가이던스 의도가 VehiclePort로 직접 송신되면 안 된다. |
| **SystemStatus** ([hpp](../cpp/include/argus/contracts/SystemStatus_TBD.hpp))<br>임무·대상·기체·진단을 전송할 상태 계약 | 전체: `STATUS, HEALTH, TARGET, FSM, MAV`<br>상세: `SNAP, MSTATE, FAULT, MANUAL, PX4_FS` | 임무 상태·TargetState·기체 상태·진단의 출처와 시각을 보존해야 한다.<br>공개 system/status schema와 필드는 미구현이다. 기존 perception/observations v2나 진단 schema를 임의 변경하지 않는다. |

## application/perception

| 역할·파일 | 설계 노드 | 조건 |
|---|---|---|
| **FrameTransform** ([hpp](../cpp/include/argus/application/perception/FrameTransform_TBD.hpp) / [cpp](../cpp/src/application/perception/FrameTransform_TBD.cpp))<br>카메라 장착·보정과 환경별 좌표계 변환 | 전체: `TIME, GEO, POSE`<br>상세: `RAY, CMC, METRIC` | 원 영상 픽셀 광선, 장착 보정, Gazebo world/PX4 local 좌표와 시각을 구분한다.<br>보정값·축 방향·단위·지면 모델을 임의 확정하지 않는다. 실제 노출 시각이 없는 정밀 정렬을 완료됐다고 표시하지 않는다. |
| **CameraMotion** ([hpp](../cpp/include/argus/application/perception/CameraMotion_TBD.hpp) / [cpp](../cpp/src/application/perception/CameraMotion_TBD.cpp))<br>FC 자세를 프레임 시각에 정렬하는 회전 CMC | 전체: `MOTION, TIME, TRACK`<br>상세: `CMC, FRAME, FC` | 이전·현재 프레임의 실제 노출 시각에 FC 자세를 정렬한다. 병진 시차는 보상하지 않는 설계다.<br>ByteTrack, OC-SORT, Hybrid-SORT HMIoU OFF/ON 모두 같은 FC 회전 CMC 조건을 사용한다.<br>기존 ICameraMotion 포트와 ITracker.hpp의 argus::CameraMotion 자료형을 유지한다. 이 파일은 동명 클래스 선언이 아니다. |
| **TrackingPublisher** ([hpp](../cpp/include/argus/application/perception/TrackingPublisher_TBD.hpp) / [cpp](../cpp/src/application/perception/TrackingPublisher_TBD.cpp))<br>실제 추적 결과의 tracking/tracks 발행 | 전체: `DATA, TRACK, TARGET`<br>상세: `OBS, TRACK, TTL, CONF` | Confirmed/Predicted/Lost, full_box, 선택 후보 ID, 마지막 실제 검출 시각·TTL·좌표계·출처를 보존해야 한다.<br>실제 Tracker가 없으므로 class_id를 track_id로 쓰거나 가짜 트랙을 발행하지 않는다. 기존 Track.hpp 계약은 유지한다. |

## adapters/tracking

| 역할·파일 | 설계 노드 | 조건 |
|---|---|---|
| **ByteTrackTracker** ([hpp](../cpp/include/argus/adapters/tracking/ByteTrackTracker_TBD.hpp) / [cpp](../cpp/src/adapters/tracking/ByteTrackTracker_TBD.cpp))<br>ByteTrack 시험 조건의 ITracker 어댑터 | 전체: `TRACK`<br>상세: `CHOICE, BYTE, CMC, OBS` | 시험 설정 1: ByteTrack, ReID 없음, 다른 세 조건과 같은 FC 회전 CMC 사용.<br>실제 알고리즘·연결 임계값은 미구현이며 예측으로 last_real_detection_time/TTL을 갱신하면 안 된다. |
| **OCSortTracker** ([hpp](../cpp/include/argus/adapters/tracking/OCSortTracker_TBD.hpp) / [cpp](../cpp/src/adapters/tracking/OCSortTracker_TBD.cpp))<br>OC-SORT 시험 조건의 ITracker 어댑터 | 전체: `TRACK`<br>상세: `CHOICE, OC, CMC, OBS` | 시험 설정 2: OC-SORT, ReID 없음, 다른 세 조건과 같은 FC 회전 CMC 사용.<br>overview의 CMC 없음 설명보다 상세 flow의 동일 CMC 비교 조건을 따른다. 알고리즘과 임계값은 미구현이다. |
| **HybridSortTracker** ([hpp](../cpp/include/argus/adapters/tracking/HybridSortTracker_TBD.hpp) / [cpp](../cpp/src/adapters/tracking/HybridSortTracker_TBD.cpp))<br>Hybrid-SORT HMIoU OFF/ON 두 시험 조건 | 전체: `TRACK`<br>상세: `CHOICE, HOFF, HON, CMC, OBS` | 시험 설정 3은 HMIoU OFF/일반 IoU, 시험 설정 4는 HMIoU ON이다. 같은 알고리즘 파일의 두 설정으로 표시한다.<br>HMIoU 외 TCM·ROCM·연결 단계와 설정은 같아야 한다. 둘 다 ReID 없음, 같은 FC 회전 CMC를 사용한다.<br>두 조건의 알고리즘·설정·임계값은 미구현이다. 승인 없이 구현 복제나 비교 조건 변경을 하지 않는다. |

## application/control

| 역할·파일 | 설계 노드 | 조건 |
|---|---|---|
| **LatestStateStore** ([hpp](../cpp/include/argus/application/control/LatestStateStore_TBD.hpp) / [cpp](../cpp/src/application/control/LatestStateStore_TBD.cpp))<br>최신 트랙·대상·기체 상태 저장소 | 전체: `STORE, DATA, MAV`<br>상세: `OBS, SNAP, FC, COMMIT, HOLD_REQ, SAFETY` | 제어는 최신 유효 상태를 원자적으로 읽으며 epoch와 임무/차단/선택 ID를 함께 다룬다.<br>기존 인지 FrameQueue의 용량 2 FIFO/drop-oldest는 별도 정책이다. 이 저장소를 이유로 인지 큐를 변경하지 않는다.<br>Predicted 또는 새 배치 발행으로 트랙 TTL을 연장하지 않는다. 저장·동기화 로직은 미구현이다. |
| **ControlLoop** ([hpp](../cpp/include/argus/application/control/ControlLoop_TBD.hpp) / [cpp](../cpp/src/application/control/ControlLoop_TBD.cpp))<br>영상 도착과 독립된 제어 주기 | 전체: `LOOP, STORE, TARGET, GEO, FSM, GUARD, API`<br>상세: `TICK, SNAP, MSTATE, ARBITER, DROP` | 주기 Timer에서 이벤트를 수집하고 하나의 ControlContext로 선택·기하·임무·Guard를 판단한다.<br>Watchdog은 별도 실행이어야 한다. 미정 제어 주기나 임계값을 임의 확정하지 않는다.<br>오래된 epoch 의도는 폐기 후 상태를 재평가한다. 현재 인지 실행부에 이 루프를 자동 연결하지 않는다. |
| **TargetSelector** ([hpp](../cpp/include/argus/application/control/TargetSelector_TBD.hpp) / [cpp](../cpp/src/application/control/TargetSelector_TBD.cpp))<br>운용자 지정 단일 ID 선택과 동일성 확인 | 전체: `TARGET, STORE, DATA`<br>상세: `PREP, PREP_CHECK, FINAL_DET, COMMIT, FGATE, CONF, SCAN_CAND, WAIT_CAND` | 운용자가 확인한 1ID를 고정하며 겹침·ID 불확실성에서 새 ID를 임의로 승인하지 않는다.<br>pending 후보와 실제 활성 선택을 구분한다. Hold/Scan의 새 후보는 표시·기록만 하며 재추종 명령이 아니다. |
| **Geometry** ([hpp](../cpp/include/argus/application/control/Geometry_TBD.hpp) / [cpp](../cpp/src/application/control/Geometry_TBD.cpp))<br>원 영상 전체 박스에서 방향·거리 품질 계산 | 전체: `GEO, TIME`<br>상세: `RAY, DIR, FOV, RISK, RANGE, METRIC, DIST, NO_RANGE` | 전체 박스(full_box)와 카메라/FC 정렬을 사용한다. hull ROI는 사용하지 않는다.<br>방향과 거리 품질을 분리하고 METRIC/SCALE_ONLY/INVALID를 구분해야 한다.<br>미터 거리는 접점/특징점·카메라 높이·지면 모델·기하/시각 유효성 확인이 필요하다. bbox 크기만으로 미터 거리를 임의 확정하지 않는다.<br>시야 판정 기준과 카메라 보정/높이 값은 미정이며 실제 구현은 없다. |
| **MissionStateMachine** ([hpp](../cpp/include/argus/application/control/MissionStateMachine_TBD.hpp) / [cpp](../cpp/src/application/control/MissionStateMachine_TBD.cpp))<br>시작·추종·Hold·Scan·Manual·Fault 임무 전환 | 전체: `FSM, LOOP, FOLLOW, GUARD, API`<br>상세: `STARTUP, FOLLOWING, HOLD_TRANSITION, WAITING, SCANNING, FAULTS, IDLE, PREP, READY, ENTER, ACK, STARTING, FINAL_DET, COMMIT, HOLD_REQ, HCLASS, HZERO, HSTOP, HACK, HFAILED, BLOCK_WAIT, WAIT, SCAN_ENTER, SCAN_ACK, SCAN_COMMIT, MANUAL, FAULT, NO_RETRY` | 최초 시작/명시적 재시작은 후보 확인, 준비 검사, 정지 setpoint 사전 송신, 실제 Offboard 확인, 최신 실제 검출 재확인 뒤 원자적 COMMIT 순서다.<br>Hold 전환은 재개차단 ON, 의도 폐기, epoch 증가, 거리 적분 리셋이며 Hold 요청은 사건당 1회다. hold_attempted와 실제 모드를 확인한다.<br>HOLD_WAIT는 후보 표시만 한다. Scan은 운용자 명령과 조건 확인 후 시작하며 후보 발견은 자동 재추종이 아니다.<br>Manual은 운용자 모드를 존중하고 Offboard 명령을 중지한다. Fault/전환 실패는 Hold 자동 재요청을 하지 않는다.<br>기체 상태에 관계없이 Hover가 보장된다고 주장하지 않는다. 상태 전환 코드와 타이머는 미구현이다. |
| **Follower** ([hpp](../cpp/include/argus/application/control/Follower_TBD.hpp) / [cpp](../cpp/src/application/control/Follower_TBD.cpp))<br>Confirmed·Predicted·Scan 가이던스 의도 생성 | 전체: `FOLLOW, GEO, FSM, GUARD`<br>상세: `GUIDANCE, CONF, DIR, FOV, RISK, RECOVER, RANGE, NO_RANGE, METRIC, DIST, YAW_OK, TURN, APPROACH, FUTURE_RISK, BRAKE, SETTLE, FOLLOW_PI, FOLLOW_INTENT, PRED_INTENT, SCAN_INTENT` | Confirmed는 방향과 거리 품질을 분리한다. SCALE_ONLY/INVALID는 전진 0이며 METRIC만 미터 거리 기반 추종에 사용할 수 있다.<br>시야 위험·회전 우선·접근 위험은 전진 0이다. 원거리 접근, 중거리 감속, 목표거리 추종 조건을 구분한다.<br>Predicted는 전진·횡·yaw 0, 독립 고도 유지, 거리 적분 정지다. Scan은 제한 yaw 의도와 전진·횡 0이다.<br>yaw P/거리 P 또는 PI/고도 P의 이득·속도·가속·저크·적분 한도를 임의 확정하지 않는다. VehiclePort로 직접 송신하지 않는다. |
| **CommandArbiter** ([hpp](../cpp/include/argus/application/control/CommandArbiter_TBD.hpp) / [cpp](../cpp/src/application/control/CommandArbiter_TBD.cpp))<br>동일 epoch·임무와 안전 이벤트를 확인하는 의도 중재 | 전체: `LOOP, GUARD, PORT`<br>상세: `ARBITER, SAFETY, FOLLOW_INTENT, PRED_INTENT, SCAN_INTENT, DROP` | 같은 epoch·임무 상태인지, 우선 안전 이벤트가 없는지, FC와 실제 Offboard가 유효한지 확인해야 한다.<br>Watchdog 안전 이벤트를 우선하고 오래된 의도는 폐기하여 상태를 재평가한다. 최종 송신 전에 CommandGuard를 거친다. |
| **CommandGuard** ([hpp](../cpp/include/argus/application/control/CommandGuard_TBD.hpp) / [cpp](../cpp/src/application/control/CommandGuard_TBD.cpp))<br>송신 직전의 최종 명령 유효성·제한 검사 | 전체: `GUARD, LOOP, PORT, MAV`<br>상세: `GUARD, ARBITER, SEND, PRED_INTENT, SCAN_INTENT, NO_RANGE, RISK` | 같은 ControlContext에서 제어권·RC 개입·상태·epoch·한도·만료·stale·변화율을 최종 확인한다.<br>METRIC 무효면 잔류 전진 0, Predicted는 수평·yaw 0, Scan은 전진·횡 0, 시야 위험이면 전진 0이어야 한다.<br>유효 명령만 단일 IVehiclePort에 직접 전달한다. 한도·TTL·변화율 값은 미정이며 임의 초기값이나 허용 응답을 만들지 않는다. |
| **Watchdog** ([hpp](../cpp/include/argus/application/control/Watchdog_TBD.hpp) / [cpp](../cpp/src/application/control/Watchdog_TBD.cpp))<br>영상·제어 루프와 독립된 상태 감시 | 전체: `CONTROL, HEALTH, GUARD`<br>상세: `WATCHDOG, WTICK, WSTATE, WF, WS, WW, WP, WTERMINAL, WOK, SAFETY, FEXIT, SMODE` | 영상 프레임 도착과 독립된 Timer에서 임무 상태별 프레임·TTL·FC·실제 모드·RC·전환 결과를 감시한다.<br>안전 이벤트가 우선하며 명령 epoch를 무효화한다. Fault/Manual은 명령 생성 없이 관측한다.<br>상세 flow의 20~50 Hz는 미시험 후보이며 실행 주기로 확정하지 않는다. Hover 보장을 주장하거나 Hold 자동 재요청을 하지 않는다. |

## application/commands

| 역할·파일 | 설계 노드 | 조건 |
|---|---|---|
| **CommandGateway** ([hpp](../cpp/include/argus/application/commands/CommandGateway_TBD.hpp) / [cpp](../cpp/src/application/commands/CommandGateway_TBD.cpp))<br>운용자 요청 ID·중복 제거·결과 응답 | 전체: `API, UI, LOOP`<br>상세: `PREP, OP_SCAN, WEVENT, NO_RETRY` | overview에서 GCS 영역 API로 표시된 역할이다. 이 Pi 파일은 계획된 역할의 표시용이며 실행 위치나 Windows 서비스 구현을 확정하지 않는다.<br>운용자 선택/시작/중단/재탐색 요청의 ID·중복 제거·결과 응답을 제어 이벤트와 연결해야 한다.<br>기존 CommandRequest.hpp 자료형은 유지한다. 자동 재시작·자동 재추종·Hold 재시도를 만들어서는 안 된다. |

## application/diagnostics

| 역할·파일 | 설계 노드 | 조건 |
|---|---|---|
| **HealthMonitor** ([hpp](../cpp/include/argus/application/diagnostics/HealthMonitor_TBD.hpp) / [cpp](../cpp/src/application/diagnostics/HealthMonitor_TBD.cpp))<br>입력·NPU·프레임·FC·배터리·GCS 진단 집계 | 전체: `HEALTH, SOURCE, DET, MAV`<br>상세: `FHEALTH, SCHECK, WCHECK, WP, PX4_FS` | 카메라·NPU·프레임 연령·FC·배터리·GCS 링크 진단과 출처/시각을 구분한다.<br>기존 인지 fault/진단 발행은 구현돼 있지만 종합 건강 상태 집계는 미구현이다. 미정 이상 판정값을 임의 확정하지 않는다. |
| **SystemStatusPublisher** ([hpp](../cpp/include/argus/application/diagnostics/SystemStatusPublisher_TBD.hpp) / [cpp](../cpp/src/application/diagnostics/SystemStatusPublisher_TBD.cpp))<br>임무·대상·기체·진단의 system/status 발행 | 전체: `STATUS, LOOP, HEALTH, MAV, TARGET, FSM, UI`<br>상세: `MSTATE, SNAP, FAULT, MANUAL, PX4_FS` | 같은 상태 스냅샷의 임무·TargetState·기체·진단을 운영 UI에 전달해야 한다.<br>현재 system/status 발행은 미구현이다. 기존 관측/진단/JPEG 발행을 전체 임무 상태 발행으로 표시하지 않는다. |

## application/evaluation

| 역할·파일 | 설계 노드 | 조건 |
|---|---|---|
| **Evaluator** ([hpp](../cpp/include/argus/application/evaluation/Evaluator_TBD.hpp) / [cpp](../cpp/src/application/evaluation/Evaluator_TBD.cpp))<br>로그와 정답을 사용하는 추적·제어 성능 평가 | 전체: `LOG, DATA, STATUS, MOTION, GZ, MAV`<br>상세: `OBS, FC, FOLLOW_INTENT, SEND` | mode, t_frame, t_att, t_cmd, FPS, E2E, ID 유지율과 출처/정답 시각을 연결할 평가 역할이다.<br>기존 Recorder의 비동기 인지 CSV 기록은 구현돼 있다. 평가 지표·정답 정렬·제어 로그는 미구현이며 결과를 임의 생성하지 않는다. |

## adapters/gazebo

| 역할·파일 | 설계 노드 | 조건 |
|---|---|---|
| **GazeboCameraFrameSource** ([hpp](../cpp/include/argus/adapters/gazebo/GazeboCameraFrameSource_TBD.hpp) / [cpp](../cpp/src/adapters/gazebo/GazeboCameraFrameSource_TBD.cpp))<br>SITL_IMAGE Gazebo 영상 입력 | 전체: `GZ, SOURCE`<br>상세: `FRAME` | IFrameSource와 기존 프레임/시각 계약을 따르는 Gazebo Transport 영상 입력 역할이다.<br>Transport 구독과 실제 노출 시각은 미구현이다. 입력 생성 요청을 다른 입력이나 Mock으로 자동 대체하지 않는다. |
| **GazeboPoseAdapter** ([hpp](../cpp/include/argus/adapters/gazebo/GazeboPoseAdapter_TBD.hpp) / [cpp](../cpp/src/adapters/gazebo/GazeboPoseAdapter_TBD.cpp))<br>SITL_POSE 좌표·시간 정규화 | 전체: `GZ, POSE, OBS, TIME`<br>상세: `OBS, FC` | SITL_POSE 전용 Gazebo Transport pose를 좌표·시각·출처와 함께 정규화할 역할이다.<br>현재 온보드 Observation.hpp는 IMAGE2D 계약이다. POSE3D 확장은 미구현이며 기존 v2 공개 관측 schema를 임의 바꾸지 않는다.<br>기존 PC 단일 박스/AlphaBeta 시험 경로를 완성된 온보드 구현으로 복사·등록하지 않는다. |

## adapters/vehicle

| 역할·파일 | 설계 노드 | 조건 |
|---|---|---|
| **MavsdkVehicleAdapter** ([hpp](../cpp/include/argus/adapters/vehicle/MavsdkVehicleAdapter_TBD.hpp) / [cpp](../cpp/src/adapters/vehicle/MavsdkVehicleAdapter_TBD.cpp))<br>MAVSDK 단일 송신자·텔레메트리·만료 감시 | 전체: `MAV, PORT, PX4, STORE, MOTION`<br>상세: `SEND, FC, ENTER, ACK, HSTOP, HACK, SCAN_ENTER, SCAN_ACK, PX4_FS` | 유효한 Guard 결과만 직렬 송신하며 기존 IVehiclePort 계약을 보존한다. 텔레메트리는 시각/실제 모드와 함께 제공한다.<br>Offboard set_velocity_body, 진입 확인, 사건당 1회 Offboard::stop/Hold 요청, 명령 만료 감시는 모두 미구현이다.<br>요청 응답을 실제 모드 확인으로 대신하지 않으며 Hold 실패를 자동 재시도하지 않는다. 연결·송신·시험값을 임의 생성하지 않는다. |
| **ReplayVehicleAdapter** ([hpp](../cpp/include/argus/adapters/vehicle/ReplayVehicleAdapter_TBD.hpp) / [cpp](../cpp/src/adapters/vehicle/ReplayVehicleAdapter_TBD.cpp))<br>REPLAY 환경의 기체 로그/포트 대응 | 전체: `REPLAY, MAV, STORE, MOTION`<br>상세: `FC, SNAP, SEND` | REPLAY 프로파일의 기체 상태와 포트 대응은 미구현이다. 실제 FC에 연결하지 않는 역할이다.<br>실제 송신이 성공했다고 가정하거나 임의 FC 상태를 합성하지 않는다. 기존 유한 영상 Replay 구현과 구분한다. |

## adapters/replay

| 역할·파일 | 설계 노드 | 조건 |
|---|---|---|
| **ReplayFcLogSource** ([hpp](../cpp/include/argus/adapters/replay/ReplayFcLogSource_TBD.hpp) / [cpp](../cpp/src/adapters/replay/ReplayFcLogSource_TBD.cpp))<br>IR 영상과 함께 재생할 FC RPY 로그 입력 | 전체: `REPLAY, MOTION, TIME`<br>상세: `FC, CMC, FRAME, RAY` | IR 영상과 FC 자세 로그를 원래 프레임/노출 시각에 맞춰 재생하는 역할이다.<br>로그 포맷·동기화·시계 연결은 미구현이다. 기존 영상 Replay의 호스트 읽기 시각을 원래 노출 시각으로 간주하지 않는다. |

## 기존 구현과 구분

| 영역 | 구현된 부분 | 미구현 |
|---|---|---|
| Observation/Track | IMAGE2D 관측, Track/TTL 자료형 | POSE3D, 실제 추적, tracking/tracks 발행 |
| Recorder | 비동기 인지 CSV | FC/명령/정답 정렬, 유지율 평가 |
| Replay | 유한 영상 캡처 | 원래 노출 시각과 FC 로그의 동기 재생 |
| 진단/통신 | 인지 fault·관측·진단·옵션 JPEG | 종합 HealthMonitor, system/status |
| CommandRequest | 요청/결과 자료형 | Gateway 중복 제거·이벤트·결과 처리 |

Hybrid-SORT HMIoU OFF/ON은 한 파일의 두 설정으로 표시한다.
ByteTrack / OC-SORT / Hybrid-SORT OFF / ON 네 후보는 별도로 등록해야 하며,
모두 ReID 없이 같은 FC 회전 CMC 조건을 따른다.
Windows 통제기·PX4·Gazebo와 기존 PC 시험 모듈은 외부 시스템/별도 프로젝트다.
이 파일 배치로 외부 구현이 완료된 것은 아니다.

전체 파일·조건·해시는 [unimplemented-files.json](unimplemented-files.json),
배치 당시 검사와 백업은 [VALIDATION](VALIDATION.md)에 있다.
