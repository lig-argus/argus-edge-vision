# 시스템 UML

| 항목 | 내용 |
|---|---|
| 문서 버전 | 0.8 (팀 검토 전 초안) |
| 목적 | 규약 문서(`docs/spec/`)에 글과 표로 있는 구조와 동작을 UML 로 옮겨 빠진 인터페이스·상태를 찾는다. 규약과 다르면 규약이 맞고, 이 문서를 고친다 |
| 표기 | Mermaid. GitHub 에서 바로 렌더된다. 다이어그램 안 라벨은 한국어, 다이어그램 종류 이름은 영어 |
| 대상 독자 | 추적·추종 담당(onboard control), 탐지 담당(onboard vision), 통제기 담당(gcs) |

## 파일

| 파일 | 다이어그램 | 참조 문서 | 상태 |
|---|---|---|---|
| `deployment.md` | Deployment Diagram — 장비, 실행 환경(OS·런타임), 설치되는 artifact(실행 파일·설정·모델), 통신 경로 | `docs/spec/icd.md` 1장, `docs/spec/video.md` 1·8장 | 있음 |
| `component.md` | Component Diagram — 프로세스와 통제기를 컴포넌트로, ZeroMQ 토픽·REQ/REP·MAVLink·RTSP 를 provided·required interface 로 | `docs/spec/process_interfaces.md` 2·5장, `docs/spec/icd.md` 1장 | 있음 |
| `state_machine.md` | State Machine Diagram — `ARGUS_MISSION_STATE`, `ARGUS_TARGET_STATE` 의 상태와 transition | `docs/spec/icd.md` 3·4·6장, `common/mavlink/message_definitions/argus.xml` | 있음 |
| `class_onboard.md` | Class Diagram — onboard (공통 인터페이스 `IMavlinkConnection`·ZeroMQ·`IClock`·`EventLoop`, FC 통신, 통제기 통신) | `docs/spec/icd.md`, `docs/spec/process_interfaces.md` 3·5·6장 | 있음 |
| `class_follow.md` | Class Diagram — 추종 제어의 외부 인터페이스 (`IVehicleTelemetry`·`IVelocitySetpointSink`·`IFlightModeControlClient`·`IFollowControl` 과 ZeroMQ 어댑터) | `docs/spec/process_interfaces.md` 4장 | 있음 (내부는 추적·추종 담당) |
| `class_tracker.md` | Class Diagram — UCMC 추적기의 외부 인터페이스 (구독·발행) | `docs/spec/process_interfaces.md` 5.1 | 있음 (내부는 추적·추종 담당) |
| `class_vision.md` | Class Diagram — 탐지 프로세스의 외부 인터페이스 (C++ 과 Python 이 맡는 처리, 발행 토픽 4개)와 Python 쪽 내부 인터페이스(`IPreprocessor`, `IDetector`, `IPreviewSink`) | `docs/spec/process_interfaces.md` 5.1, `docs/spec/video.md` 8장 | 있음 |
| `class_video.md` | Class Diagram — 영상 송신부(`IFrameSource`, `FrameCollector`, ONVIF XML, RTSP), 통제기 영상 수신·동기화 | `docs/spec/video.md` 1~6장 | 있음 |
| `class_gcs.md` | Class Diagram — 통제기 (MAVLink 클라이언트, 상태 모델, 명령, 영상 표시) | `docs/spec/icd.md`, `docs/spec/video.md` 6·7장 | 있음 |
| `sequence.md` | Sequence Diagram — 박스 클릭 → 추종 시작, 추종 1주기, 영상 프레임 1장, FC telemetry 두절 | `docs/spec/icd.md` 4·6장, `docs/spec/process_interfaces.md` 5·6장, `docs/spec/video.md` 4~6장 | 있음 |

- `class_follow.md`·`class_tracker.md` 는 다른 프로세스와 주고받는 토픽·인터페이스·어댑터만 그렸다. `class_vision.md` 는 2절에 Python 쪽 내부 인터페이스가 있다. 내부 Class Diagram 은 각 담당이 같은 파일에 절을 추가한다.
- Class Diagram 마다 "SOLID 대응" 표를 두어 인터페이스를 둔 이유(교체·테스트·확장)를 적는다. 구체 클래스(`UdpMavlinkConnection`, `Zmq*`, `SteadyClock`, GStreamer 래퍼)는 조립 지점(`main`)에서만 보이고 나머지는 인터페이스에 의존한다.

## 표기 규칙

- Deployment Diagram: Mermaid 에 deployment 전용 문법이 없어 flowchart 로 그린다. 상자 = 장비와 OS, 네모 = artifact(둘째 줄은 실행 환경·런타임), 원통 = 파일·장치, 선 = 통신 경로.
- Component Diagram: Mermaid 에 component 전용 문법이 없어 flowchart 로 그린다. 컴포넌트는 네모, 인터페이스는 둥근 네모(이름 = 토픽·요청·프로토콜). provided(발행·응답·서비스하는 쪽)는 컴포넌트 → 인터페이스 실선, required(구독·요청·클라이언트 쪽)는 인터페이스 → 컴포넌트 점선. 미결 항목은 라벨 끝에 `[미결 n]` (번호는 참조 문서의 검토 항목).
- Class Diagram: Mermaid `classDiagram`. `<<interface>>` 는 순수 가상 클래스. `<|..` 구현, `<|--` 상속, `*--` 소유, `..>` 사용(호출·구독·발행, 라벨에 토픽·메시지 이름). 멤버는 책임을 보이는 것만 적는다.
- Sequence Diagram: Mermaid `sequenceDiagram`. 참여자 이름은 Class Diagram 의 클래스 이름(괄호 안 프로세스). `->>` 호출·송신, `-->>` 응답, `--)` 발행.
- State Machine Diagram: Mermaid `stateDiagram-v2`. transition 라벨은 UML 표기 `event [guard] / effect` 를 따른다. 그림에는 event 만 적고 guard·effect 와 출처는 그림 아래 표에 적는다. 규약에 event 가 없는 transition 은 `[미결]` 로 적고 규약 문서에 질문을 남긴다.

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| 0.1 | 2026-10-16 | 초안. README, Deployment Diagram, Component Diagram, State Machine Diagram (#31) |
| 0.2 | 2026-10-16 | Class Diagram 6개 추가: onboard, 추종 제어·추적기·탐지 경계, 영상, 통제기 (#31) |
| 0.3 | 2026-10-19 | Sequence Diagram 추가: 박스 클릭 → 추종 시작, 추종 1주기, 영상 프레임 1장, FC telemetry 두절 (#31) |
| 0.4 | 2026-10-19 | 탐지 담당 리뷰 반영(#33: deployment·component·class_vision·sequence), `vehicle_api` 타깃 분리(#35 리뷰, class_follow), 파일 제목의 "경계" → "외부 인터페이스" |
| 0.5 | 2026-10-19 | 추적·추종 담당 리뷰 반영(#33): state_machine(Offboard 확인·실패 FAULT·복구), sequence(단조 시각·실제 모드 확인), class_tracker(FC 표본 입력), class_onboard(송신권·독립 감시), class_follow(결과 형), "산출물" 을 UML 요소 이름 artifact 로 |
| 0.6 | 2026-10-25 | 코드(#39, #40)와 이름 맞춤: class_onboard 0.5(실행 루프, `IMavlinkConnection`, `FcCommandGate`, `MavlinkCommandReceiver`·`CommandResponder`), class_follow 0.5(구현용·호출용 인터페이스 분리), class_gcs 0.2, class_tracker 0.5, class_video(`FrameJoiner`), sequence 0.4 |
| 0.7 | 2026-10-26 | class_video 0.4(코드 #41 과 맞춤: `IFrameSource.subscribe`, `FrameCollector`·`CollectedFrame`, 직렬화는 구체 클래스), sequence 0.5 |
| 0.8 | 2026-10-29 | class_vision 0.4(Python 쪽 내부 인터페이스, 코드 `onboard/vision/python/` 과 맞춤) |
