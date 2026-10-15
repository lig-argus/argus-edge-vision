# 시스템 UML

| 항목 | 내용 |
|---|---|
| 문서 버전 | 0.1 (팀 검토 전 초안) |
| 목적 | 규약 문서(`docs/spec/`)에 글과 표로 있는 구조와 동작을 UML 로 옮겨 빠진 인터페이스·상태를 찾는다. 규약과 다르면 규약이 맞고, 이 문서를 고친다 |
| 표기 | Mermaid. GitHub 에서 바로 렌더된다. 다이어그램 안 라벨은 한국어, 다이어그램 종류 이름은 영어 |
| 대상 독자 | 추적·추종 담당(onboard control), 탐지 담당(onboard vision), 통제기 담당(gcs) |

## 파일

| 파일 | 다이어그램 | 참조 문서 | 상태 |
|---|---|---|---|
| `deployment.md` | deployment diagram — 장비, 실행 환경(OS·런타임), 설치 산출물, 통신 경로 | `docs/spec/icd.md` 1장, `docs/spec/video.md` 1·8장 | 있음 |
| `component.md` | component diagram — 프로세스와 통제기를 컴포넌트로, ZeroMQ 토픽·REQ/REP·MAVLink·RTSP 를 provided·required interface 로 | `docs/spec/process_interfaces.md` 2·5장, `docs/spec/icd.md` 1장 | 있음 |
| `state_machine.md` | state machine diagram — `ARGUS_MISSION_STATE`, `ARGUS_TARGET_STATE` | `docs/spec/icd.md` 3·4·6장, `common/mavlink/message_definitions/argus.xml` | 있음 |
| `class_onboard.md` | class diagram — onboard (FC 통신, 통제기 통신, ZeroMQ 어댑터) | `docs/spec/process_interfaces.md` 4장 | 예정 (#25 머지 뒤) |
| `class_follow.md` | class diagram — 추종 제어 경계 (`IFollowCommand` 구현, 상태 관리) | `docs/spec/process_interfaces.md` 4장 | 예정 |
| `class_video.md` | class diagram — 영상 송신부, 통제기 영상 수신 | `docs/spec/video.md` 5·6장 | 예정 (#20 머지 뒤) |
| `class_gcs.md` | class diagram — 통제기 (MAVLink 클라이언트, 운용 화면) | `docs/spec/icd.md` | 예정 |
| `sequence.md` | sequence diagram — TRACK_START, 추종 1주기, FC 두절 | `docs/spec/icd.md` 4·6장 | 예정 (class diagram 뒤) |

- 추적기(UCMC)·탐지 프로세스의 내부 class diagram 은 각 담당이 `class_tracker.md`, `class_vision.md` 로 추가한다. 여기서는 경계 인터페이스(토픽)만 그린다.

## 표기 규칙

- deployment diagram: Mermaid 에 deployment 전용 문법이 없어 flowchart 로 그린다. 상자 = 장비와 OS, 네모 = 산출물(둘째 줄은 실행 환경·런타임), 원통 = 파일·장치, 선 = 통신 경로.
- component diagram: Mermaid 에 component 전용 문법이 없어 flowchart 로 그린다. 컴포넌트는 네모, 인터페이스는 둥근 네모(이름 = 토픽·요청·프로토콜). provided(발행·응답·서비스하는 쪽)는 컴포넌트 → 인터페이스 실선, required(구독·요청·클라이언트 쪽)는 인터페이스 → 컴포넌트 점선. 미결 항목은 라벨 끝에 `[미결 n]` (번호는 참조 문서의 검토 항목).
- state machine diagram: Mermaid `stateDiagram-v2`. 화살표 라벨은 "사건 / 조건" 순서. 규약에 사건이 없는 화살표는 `[미결]` 로 적고 규약 문서에 질문을 남긴다.

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| 0.1 | 2026-10-16 | 초안. README, deployment diagram, component diagram, state machine diagram (#31) |
