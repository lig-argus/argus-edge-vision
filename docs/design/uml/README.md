# 시스템 UML

| 항목 | 내용 |
|---|---|
| 문서 버전 | 0.2 (팀 검토 전 초안) |
| 목적 | 규약 문서(`docs/spec/`)에 글과 표로 있는 구조와 동작을 UML 로 옮겨 빠진 인터페이스·상태를 찾는다. 규약과 다르면 규약이 맞고, 이 문서를 고친다 |
| 표기 | Mermaid. GitHub 에서 바로 렌더된다. 다이어그램 안 라벨은 한국어, 다이어그램 종류 이름은 영어 |
| 대상 독자 | 추적·추종 담당(onboard control), 탐지 담당(onboard vision), 통제기 담당(gcs) |

## 파일

| 파일 | 다이어그램 | 참조 문서 | 상태 |
|---|---|---|---|
| `deployment.md` | Deployment Diagram — 장비, 실행 환경(OS·런타임), 설치 산출물, 통신 경로 | `docs/spec/icd.md` 1장, `docs/spec/video.md` 1·8장 | 있음 |
| `component.md` | Component Diagram — 프로세스와 통제기를 컴포넌트로, ZeroMQ 토픽·REQ/REP·MAVLink·RTSP 를 provided·required interface 로 | `docs/spec/process_interfaces.md` 2·5장, `docs/spec/icd.md` 1장 | 있음 |
| `state_machine.md` | State Machine Diagram — `ARGUS_MISSION_STATE`, `ARGUS_TARGET_STATE` 의 상태와 transition | `docs/spec/icd.md` 3·4·6장, `common/mavlink/message_definitions/argus.xml` | 있음 |
| `class_onboard.md` | Class Diagram — onboard (공통 경계 `IMavlinkIo`·ZeroMQ·`IClock`, FC 통신, 통제기 통신) | `docs/spec/icd.md`, `docs/spec/process_interfaces.md` 3·5·6장 | 있음 |
| `class_follow.md` | Class Diagram — 추종 제어 경계 (`IVehicleState`·`IVelocitySetpoint`·`IModeControl`·`IFollowCommand` 와 ZeroMQ 어댑터) | `docs/spec/process_interfaces.md` 4장 | 있음 (내부는 추적·추종 담당) |
| `class_tracker.md` | Class Diagram — UCMC 추적기 경계 (구독·발행) | `docs/spec/process_interfaces.md` 5.1 | 있음 (내부는 추적·추종 담당) |
| `class_vision.md` | Class Diagram — 탐지 프로세스 경계 (카메라, 전처리, 발행) | `docs/spec/process_interfaces.md` 5.1, `docs/spec/video.md` 8장 | 있음 (내부는 탐지 담당) |
| `class_video.md` | Class Diagram — 영상 송신부, 통제기 영상 수신·동기화 | `docs/spec/video.md` 1~6장 | 있음 |
| `class_gcs.md` | Class Diagram — 통제기 (MAVLink 클라이언트, 상태 모델, 명령, 영상 표시) | `docs/spec/icd.md`, `docs/spec/video.md` 6·7장 | 있음 |
| `sequence.md` | Sequence Diagram — TRACK_START, 추종 1주기, FC 두절 | `docs/spec/icd.md` 4·6장 | 예정 (Class Diagram 뒤) |

- `class_follow.md`·`class_tracker.md`·`class_vision.md` 는 경계(토픽·인터페이스·어댑터)만 그렸다. 내부 Class Diagram 은 각 담당이 같은 파일에 절을 추가한다.
- Class Diagram 마다 "SOLID 대응" 표를 두어 인터페이스를 둔 이유(교체·테스트·확장)를 적는다. 구체 클래스(`MavlinkLink`, `Zmq*`, `SteadyClock`, GStreamer 래퍼)는 조립 지점(`main`)에서만 보이고 나머지는 인터페이스에 의존한다.

## 표기 규칙

- Deployment Diagram: Mermaid 에 deployment 전용 문법이 없어 flowchart 로 그린다. 상자 = 장비와 OS, 네모 = 산출물(둘째 줄은 실행 환경·런타임), 원통 = 파일·장치, 선 = 통신 경로.
- Component Diagram: Mermaid 에 component 전용 문법이 없어 flowchart 로 그린다. 컴포넌트는 네모, 인터페이스는 둥근 네모(이름 = 토픽·요청·프로토콜). provided(발행·응답·서비스하는 쪽)는 컴포넌트 → 인터페이스 실선, required(구독·요청·클라이언트 쪽)는 인터페이스 → 컴포넌트 점선. 미결 항목은 라벨 끝에 `[미결 n]` (번호는 참조 문서의 검토 항목).
- Class Diagram: Mermaid `classDiagram`. `<<interface>>` 는 순수 가상 클래스. `<|..` 구현, `<|--` 상속, `*--` 소유, `..>` 사용(호출·구독·발행, 라벨에 토픽·메시지 이름). 멤버는 책임을 보이는 것만 적는다.
- State Machine Diagram: Mermaid `stateDiagram-v2`. transition 라벨은 UML 표기 `event [guard] / effect` 를 따른다. 그림에는 event 만 적고 guard·effect 와 출처는 그림 아래 표에 적는다. 규약에 event 가 없는 transition 은 `[미결]` 로 적고 규약 문서에 질문을 남긴다.

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| 0.1 | 2026-10-16 | 초안. README, Deployment Diagram, Component Diagram, State Machine Diagram (#31) |
| 0.2 | 2026-10-16 | Class Diagram 6개 추가: onboard, 추종 제어·추적기·탐지 경계, 영상, 통제기 (#31) |
