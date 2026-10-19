# Class Diagram — onboard

| 항목 | 내용 |
|---|---|
| 참조 문서 | `docs/spec/icd.md` 1·2·3·4·5·6장, `docs/spec/process_interfaces.md` 3·5·6장 |
| 표기 | Mermaid `classDiagram`. `<<interface>>` 는 순수 가상 클래스. `<\|..` 구현, `*--` 소유, `..>` 사용. 멤버는 책임을 보이는 것만 적고 전체 목록이 아니다 |
| 범위 | onboard 프로세스 하나. 추종 제어 쪽 어댑터는 `class_follow.md`, 영상 송신부는 `class_video.md` |

## 1. 공통 인터페이스 (MAVLink, ZeroMQ, 시계)

```mermaid
classDiagram
    class IMavlinkIo {
        <<interface>>
        +send(msg, compid) bool
        +receive() optional~Message~
    }
    class MavlinkLink {
        -socket UDP 14540
        -seq per compid 191·195
    }
    class IPublisher {
        <<interface>>
        +publish(topic, payload)
    }
    class ISubscriber {
        <<interface>>
        +subscribe(topic, handler)
    }
    class IRequester {
        <<interface>>
        +request(payload, timeout) optional~Reply~
    }
    class IReplier {
        <<interface>>
        +serve(handler)
    }
    class ZmqPublisher
    class ZmqSubscriber
    class ZmqRequester
    class ZmqReplier
    class IClock {
        <<interface>>
        +now() monotonic_ns
    }
    class SteadyClock
    class FakeClock

    IMavlinkIo <|.. MavlinkLink
    IPublisher <|.. ZmqPublisher
    ISubscriber <|.. ZmqSubscriber
    IRequester <|.. ZmqRequester
    IReplier <|.. ZmqReplier
    IClock <|.. SteadyClock
    IClock <|.. FakeClock
```

- `IMavlinkIo` 는 UDP 를 숨긴다. 테스트는 `FakeMavlinkIo` 로 메시지를 넣고 꺼낸다.
- ZeroMQ 넷은 `process_interfaces.md` 3장의 PUB/SUB·REQ/REP 를 그대로 옮긴 것이다. 직렬화(MessagePack)는 구현 안에 둔다.
- `IClock` 은 두절·유효 시간 판정(2 s, 3 s, `valid_until_monotonic_ns`)을 테스트에서 기다리지 않고 검사하기 위한 것이다.

## 2. FC 통신 (compid 191·195)

```mermaid
classDiagram
    class FcTelemetry {
        +onMessage(ATTITUDE, LOCAL_POSITION_NED, GLOBAL_POSITION_INT, DISTANCE_SENSOR, HEARTBEAT)
        +fcConnected() bool
    }
    class TimeSync {
        +requestPeriodically()
        +onTimesync(TIMESYNC)
    }
    class SetpointRelay {
        +onSetpoint(control/setpoint)
        +tick() 20 Hz
    }
    class ModeRequestServer {
        +onRequest(control/mode_request) Reply
    }
    class HealthState {
        +fcConnected bool
        +gcsLink bool
    }
    FcTelemetry ..> IMavlinkIo : 수신
    FcTelemetry ..> IPublisher : fc/attitude, fc/state
    FcTelemetry ..> IClock : 2 s 두절
    FcTelemetry --> HealthState : FC_CONNECTED
    TimeSync ..> IMavlinkIo : TIMESYNC compid 191
    SetpointRelay ..> ISubscriber : control/setpoint
    SetpointRelay ..> IMavlinkIo : SET_POSITION_TARGET_LOCAL_NED
    SetpointRelay ..> IClock : valid_until
    SetpointRelay ..> ModeRequestServer : 유효 시간 초과 시 Hold
    ModeRequestServer ..> IReplier : control/mode_request
    ModeRequestServer ..> IMavlinkIo : DO_SET_MODE, DO_REPOSITION
    ModeRequestServer ..> IPublisher : fc/mode_event
```

| 클래스 | 책임 | 출처 |
|---|---|---|
| `FcTelemetry` | FC 메시지를 받아 수신 시각(`received_monotonic_ns`)을 붙여 `fc/attitude`·`fc/state` 로 발행. HEARTBEAT 포함 telemetry 가 2 s 없으면 `FC_CONNECTED = 0` | icd 2장, 6.2, pi 5.1 |
| `TimeSync` | compid 191 로 TIMESYNC 요청·응답 대조, `fc_time_monotonic_ns` 변환값 제공 | icd 1.1, 2장 |
| `SetpointRelay` | `control/setpoint` 를 SET_POSITION_TARGET_LOCAL_NED 로 20 Hz 송신. `valid_until_monotonic_ns` 가 지나면 속도 0 을 보내고 Hold 요청 | icd 2장, pi 6장 |
| `ModeRequestServer` | `control/mode_request`(OFFBOARD·HOLD·heading) 를 DO_SET_MODE·DO_REPOSITION 으로 보내고 수락·거부 응답. 실제 전환·Offboard 이탈 원인은 `fc/mode_event` 로 발행 | pi 5.2, icd 4.3 |
| `HealthState` | `ARGUS_STATUS.health` 비트의 현재 값 | icd 3.1 |

## 3. 통제기 통신 (compid 195)

```mermaid
classDiagram
    class ICommandHandler {
        <<interface>>
        +command() MAV_CMD
        +handle(COMMAND_LONG) AckResult
    }
    class CommandDispatcher {
        -handlers map~MAV_CMD, ICommandHandler~
        +onCommandLong(msg)
    }
    class TrackStartHandler
    class TrackStopHandler
    class ScanStartHandler
    class ControlAuthority {
        +isController(sysid) bool
    }
    class LatestTracks {
        +has(sensor, generation, class_id, track_id) bool
    }
    class FollowCommandClient {
        +trackStart(...) Result
        +stop(reason) Result
        +search(range, rate, timeout) Result
    }
    class ArgusStatusReporter {
        +onFollowStatus(follow/status)
        +tick() 1 Hz
    }
    class SensorStatusReporter {
        +onTrackingStatus(tracking/status)
        +tick() 1 Hz
    }
    class SensorMap {
        +toEnum(source) ARGUS_SENSOR
    }
    class GcsLinkMonitor {
        +onHeartbeat(sysid 250)
        +tick()
    }

    ICommandHandler <|.. TrackStartHandler
    ICommandHandler <|.. TrackStopHandler
    ICommandHandler <|.. ScanStartHandler
    CommandDispatcher *-- ICommandHandler
    CommandDispatcher ..> IMavlinkIo : COMMAND_LONG 수신, COMMAND_ACK·STATUSTEXT 송신
    TrackStartHandler ..> ControlAuthority : 조건 ①
    TrackStartHandler ..> LatestTracks : 조건 ④
    TrackStartHandler ..> FollowCommandClient
    TrackStopHandler ..> FollowCommandClient
    ScanStartHandler ..> FollowCommandClient
    FollowCommandClient ..> IRequester : follow/command
    LatestTracks ..> ISubscriber : tracking/tracks
    ArgusStatusReporter ..> ISubscriber : follow/status
    ArgusStatusReporter ..> IMavlinkIo : ARGUS_STATUS
    ArgusStatusReporter ..> SensorMap
    ArgusStatusReporter ..> HealthState
    SensorStatusReporter ..> ISubscriber : tracking/status
    SensorStatusReporter ..> IMavlinkIo : ARGUS_SENSOR_STATUS
    GcsLinkMonitor ..> IMavlinkIo : HEARTBEAT 수신
    GcsLinkMonitor ..> IClock : 3 s
    GcsLinkMonitor ..> FollowCommandClient : 두절 시 stop
    GcsLinkMonitor --> HealthState : GCS_LINK
```

| 클래스 | 책임 | 출처 |
|---|---|---|
| `CommandDispatcher` | `target_component = 195` 인 COMMAND_LONG 을 `MAV_CMD` 로 핸들러에 넘기고 결과를 COMMAND_ACK 로, DENIED·FAILED 이유를 STATUSTEXT 로 보낸다. 모르는 명령은 UNSUPPORTED | icd 4.1 |
| `TrackStartHandler` | 조건 ①(제어권) ③(FOLLOWABLE) ④(최신 추적 결과) ⑤(`mission_state`) 와 매개변수 범위 검사 → `FollowCommandClient.trackStart` → IN_PROGRESS, 이후 `command_results` 로 ACCEPTED·FAILED | icd 4.2 |
| `TrackStopHandler`, `ScanStartHandler` | 같은 방식. Scan 은 NaN 을 설정 기본값으로 바꾸고 범위 검사 | icd 4.2 |
| `ControlAuthority` | 제어권 sysid(250) 판정, `control_sysid` 값 | icd 5장 |
| `LatestTracks` | 센서별 최신 `tracking/tracks` 보관 (최신 기준 시간은 아키텍처 문서 2.3) | icd 4.2 ④ |
| `FollowCommandClient` | `follow/command` REQ, `command_id` 발급, 응답 시간 초과 → failed | pi 5.2, 6장 |
| `ArgusStatusReporter` | `follow/status` 의 `mission_state`·`target_state`·`selected` 를 ARGUS_STATUS 로 1 Hz + 변경 시 1회. `follow/status` 3 s 두절 → FAULT 보고와 Hold 요청 | icd 3.1, 6.4 |
| `SensorStatusReporter` | `tracking/status` 를 ARGUS_SENSOR_STATUS 로 센서마다 1 Hz + flags 변경 시 | icd 3.1 |
| `SensorMap` | `source` 문자열 ↔ `ARGUS_SENSOR` 대응표 (onboard 설정) | pi 3장 |
| `GcsLinkMonitor` | 통제기 HEARTBEAT 3 s 두절 → `GCS_LINK = 0`, 추종·Scan 중이면 TRACK_STOP 과 같은 절차 | icd 6.1 |

## 4. SOLID 대응

| 원칙 | 어디에 | 어떻게 |
|---|---|---|
| S 단일 책임 | 2·3절 모든 클래스 | 규약의 한 항목(메시지 하나·판정 하나)에 클래스 하나. ARGUS_STATUS 와 ARGUS_SENSOR_STATUS 도 분리 |
| O 개방-폐쇄 | `CommandDispatcher` + `ICommandHandler` | 명령 추가 = 핸들러 클래스 추가·등록. 디스패처 수정 없음 |
| L 치환 | `IMavlinkIo`, `IClock`, ZeroMQ 넷 | `Fake*` 가 같은 계약(순서·시간 제한)을 지켜 테스트에서 교체 |
| I 인터페이스 분리 | `IPublisher`·`ISubscriber`·`IRequester`·`IReplier` | 발행만 하는 클래스가 구독 API 를 보지 않음 |
| D 의존 역전 | 2·3절 전부 | 구체 `MavlinkLink`·`Zmq*`·`SteadyClock` 은 조립 지점(main)에서만 보이고, 나머지는 인터페이스에만 의존 |

## 5. 검토 필요 사항

1. `FollowCommandClient` 의 응답 시간 초과 값 (pi 미결 1).
2. `LatestTracks` 의 "최신" 기준 시간 (아키텍처 문서 2.3 값).
3. `onboard/` 폴더 구조가 정해지면 1·2·3절을 모듈(폴더)로 나눈다.

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| 0.1 | 2026-10-16 | 초안 (#31) |
| 0.2 | 2026-10-19 | 1절 제목 "공통 경계" → "공통 인터페이스" |
