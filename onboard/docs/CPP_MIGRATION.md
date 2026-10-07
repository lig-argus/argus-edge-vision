# C++ 전환 결정 — 2026-09-26

사용자는 계약·입력/큐·IPC·실행 조립·공개 발행의 C++ 전환을 승인했다.
전처리·YOLOX와 박스/JPEG의 전면 C++ 전환은 하지 않으며, 영상과 bbox는 Pi에서 합친다.
사용자의 이후 명시적 지시가 우선한다. 작업 규칙은 [AGENTS](../AGENTS.md)를 따른다.

## 책임 분리

| 영역 | C++ | Python |
|---|---|---|
| 계약 | Frame/Observation/Track/CommandRequest, ports | 영상 작업자 출력·MessagePack 변환 |
| 입력·큐 | SDK/V4L2/Replay 어댑터, FrameQueue | RAW14·letterbox 전처리 |
| IPC·생명주기 | OwnedProcess/PythonVisionAdapter, 준비·timeout·응답 ID 검사 | ready/infer/shutdown 요청 실행 |
| 조립·실행 | OnboardMain → CompositionRoot → PerceptionWorker, fault·정리 | YOLOX/Hailo 추론 |
| 공개 통신·미리보기 | 관측·진단·JPEG 발행 | Pi 박스/JPEG 별도 스레드, BusProxy, 점검·구독 도구 |

SDK는 기존 `/home/user/thessen_raw14_viewer/build/i3_raw14_capture`를 별도 프로세스로 재사용한다.
I3R4 40바이트 헤더와 RAW14 LE16을 읽으며 SDK의 버퍼 복사·14비트 검사·오래된 프레임
덮어쓰기·정지 정책을 유지한다. OpenCV 캡처도 별도 helper이며 상위 어댑터는 poll/timeout으로 종료에 응답한다.

## 보존한 계약과 범위

- 인지 큐: 기본 용량 2, FIFO/drop-oldest. 처리 중 Python 요청은 하나다.
- 기존 전처리·모델을 재사용하고 C++가 원본 픽셀 `full_box`를 검증한다.
- 내부 `session_id/frame_id`를 검사한다. 공개 Observation v2는 기존 23필드이며 `track_id=null`이다.
- 노출 시각과 수신 시각을 구분한다. 현재 `exposure_time_valid=false`이며 C++/Python은 같은 Linux 호스트의 CLOCK_MONOTONIC ns를 사용한다.
- CSV의 `queue_dwell_ms`는 호스트 수신→C++ 처리 시작이다. 예전 put→get 지연과 직접 비교하지 않는다. IPC 시간은 별도 기록한다.
- 현재 `perception_only`이며 Tracker/CMC/제어/FC 구현 승인은 포함하지 않는다.

상세 필드는 [BUS_CONTRACT](BUS_CONTRACT.md), 상태·TTL 조건은 [FLOW_ALIGNMENT](FLOW_ALIGNMENT.md)에 있다.

## 원본 보관과 후속 승인

Python 원본 31개는 `legacy/python/`의 `*.py.org`로 보관하며
[manifest.json](../legacy/python/manifest.json)의 SHA256으로 확인한다.
현행 Python은 `python/argus_workers/`다. 보관본은 실행·import·패키징하지 않으며,
C++ 실패 시 Python 전체 실행부로 fallback하지 않는다.
원본 복구는 사용자 요청이 있을 때 별도 작업 폴더에서 검토한다.

같은 날 승인한 후속 작업은 **30개 역할의 주석 전용 표시 파일 54개 배치**다.
헤더만 6개와 헤더/소스 쌍 24개이며 API·기능 구현 승인이 아니다.
CMake·팩토리·실행 경로에 연결하지 않았다. [미구현 파일 지도](UNIMPLEMENTED_FILES.md)를 따른다.

[설치·빌드 의존성 및 검증 명령](INSTALL_AND_RUN_RPI5_HAILO8.md)과
[당시 Pi 검증 결과·백업](VALIDATION.md)은 별도 문서에서 관리한다.
