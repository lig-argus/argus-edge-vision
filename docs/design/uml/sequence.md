# Sequence Diagram — 주요 시나리오

| 항목 | 내용 |
|---|---|
| 참조 문서 | `docs/spec/icd.md` 4·6장, `docs/spec/process_interfaces.md` 5·6장, `docs/spec/video.md` 4~6장 |
| 표기 | Mermaid `sequenceDiagram`. 참여자 이름은 Class Diagram 의 클래스 이름이다(괄호 안은 프로세스). `->>` 호출·송신, `-->>` 응답, `--)` 발행(PUB). 시간은 위에서 아래로 |
| 범위 | 시나리오 4개: 1. 박스 클릭 → 추종 시작, 2. 추종 1주기, 3. 영상 프레임 1장 송신·수신, 4. FC telemetry 두절 |

## 1. 박스 클릭 → 추종 시작 (TRACK_START)

```mermaid
sequenceDiagram
    actor op as 운용자
    participant view as VideoView (gcs)
    participant cmd as CommandSender (gcs)
    participant disp as CommandDispatcher (onboard)
    participant ts as TrackStartHandler (onboard)
    participant fcc as FollowCommandClient (onboard)
    participant srv as FollowCommandServer (추종 제어)
    participant ctl as FollowController (추종 제어)
    participant mode as ModeRequestServer (onboard)
    participant fc as FC

    op->>view: 박스 클릭
    view->>view: 클릭 시점 프레임의 메타데이터에서 박스 판정
    view->>cmd: trackStart(sensor, generation, class_id, track_id)
    cmd->>disp: COMMAND_LONG TRACK_START (compid 195)
    disp->>ts: handle()
    ts->>ts: 제어권·FOLLOWABLE·최신 추적 결과·mission_state 검사
    ts->>fcc: trackStart(...)
    fcc->>srv: REQ follow/command (command_id)
    srv->>ctl: trackStart()
    ctl-->>srv: 수락
    srv-->>fcc: REP follow/result outcome=IN_PROGRESS
    fcc-->>ts: IN_PROGRESS
    ts-->>disp: IN_PROGRESS
    disp-->>cmd: COMMAND_ACK IN_PROGRESS
    ctl->>mode: REQ control/mode_request OFFBOARD
    mode->>fc: DO_SET_MODE Offboard
    fc-->>mode: COMMAND_ACK ACCEPTED
    fc-->>mode: HEARTBEAT 실제 모드 Offboard
    mode--)ctl: PUB fc/mode_event Offboard (ACK + 실제 모드 확인)
    ctl--)fcc: PUB follow/status command_results outcome=SUCCEEDED, mission_state=FOLLOW_ACTIVE
    fcc-->>disp: SUCCEEDED
    disp-->>cmd: COMMAND_ACK ACCEPTED
```

- 첫 결과는 REP 응답(`follow/result`)에서 받는다. 사전 검사(형식·세션·기한)에서 거부된 요청은 `command_results` 에 남지 않을 수 있기 때문이다 (#32).
- 이후 진행은 `follow/status.command_results` 에서 같은 command_id 를 찾는다. `outcome` 과 ACK 의 대응: IN_PROGRESS → IN_PROGRESS, SUCCEEDED → ACCEPTED, REJECTED → DENIED, FAILED → FAILED.
- 통제기가 ACK 를 1 s 안에 받지 못해 다시 보내면(`icd.md` 4.1), onboard 는 같은 command_id 로 요청한다. 추종 제어는 재실행하지 않고 저장된 결과를 돌려준다.
- onboard 검사(조건 ①③④⑤)에서 거부되면 추종 제어로 보내지 않고 바로 `COMMAND_ACK DENIED` + `STATUSTEXT` 로 답한다.
- Offboard 진입 전부터 setpoint 가 2 Hz 이상 와야 PX4 가 Offboard 에 들어간다 (`icd.md` 2장). 그 흐름은 2절.

## 2. 추종 1주기

```mermaid
sequenceDiagram
    participant vis as PerceptionWorker (탐지)
    participant trk as UcmcTracker (추적기)
    participant vss as VehicleStateSubscriber (추적기)
    participant ctl as FollowController (추종 제어)
    participant fct as FcTelemetry (onboard)
    participant spr as SetpointRelay (onboard)
    participant fc as FC

    loop fc/attitude·fc/state 메시지마다 (30 Hz 이상)
        fc->>fct: ATTITUDE, LOCAL_POSITION_NED
        fct--)vss: PUB fc/attitude, fc/state (수신 시각 포함)
        fct--)ctl: PUB fc/attitude, fc/state
    end
    vis--)trk: PUB perception/observations (frame_id, unix_ns)
    trk->>vss: attitudeAt(프레임 received_monotonic_ns)
    vss-->>trk: 시각 차 50 ms 이내, 수신 후 경과 시간 200 ms 이내 메시지 (없으면 fc_fresh=false)
    trk--)ctl: PUB tracking/tracks
    ctl->>ctl: 표적 상태 갱신, 속도 계산
    ctl--)spr: PUB control/setpoint (valid_until)
    loop 20 Hz
        spr->>fc: SET_POSITION_TARGET_LOCAL_NED
    end
```

- 추적기와 추종 제어는 FC 에 직접 보내지 않는다. FC 로 가는 것은 모두 onboard 를 거친다 (`process_interfaces.md` 2.1).
- `SetpointRelay` 는 받은 최신 setpoint 를 20 Hz 로 보낸다. `valid_until` 이 지나면 4절과 같이 속도 0 과 Hold 요청.

## 3. 영상 프레임 1장 송신·수신

```mermaid
sequenceDiagram
    participant fp as PythonVisionAdapter (탐지)
    participant op as PerceptionWorker (탐지)
    participant tp as TracksPublisher (추적기)
    participant mj as MessageJoiner (영상 송신부)
    participant vs as VideoSender (영상 송신부)
    participant rs as RtspServer (영상 송신부)
    participant rr as RtspReceiver (gcs)
    participant fs as FrameSynchronizer (gcs)
    participant view as VideoView (gcs)

    fp--)mj: PUB vision/frame/gray8 (source, frame_id, unix_ns)
    op--)mj: PUB perception/observations (같은 frame_id)
    tp--)mj: PUB tracking/tracks (같은 frame_id)
    mj->>vs: JoinedFrame
    vs->>rs: pushVideo (RTP ts = unix_ns 변환)
    vs->>rs: pushMetadata (XML, UtcTime = unix_ns)
    rs->>rr: RTP/JPEG 영상 트랙
    rs->>rr: RTP 메타데이터 트랙
    rs->>rr: RTCP SR (RTP ts ↔ NTP 시각)
    rr->>fs: DecodedFrame (UTC)
    rr->>fs: FrameMetadata (UtcTime)
    fs->>fs: UTC 차 ±16 ms 이내끼리 묶음
    fs->>view: VideoFrame
```

- 세 토픽은 도착 순서가 보장되지 않는다. `MessageJoiner` 는 대기 시간 안에 짝이 모두 오지 않으면 버린다. 영상 프레임이 없는 탐지·추적 결과는 보내지 않는다 (`process_interfaces.md` 5.3).
- 영상 패킷을 먼저 보내고 메타데이터를 보낸다 (`video.md` 5장).
- 메타데이터가 늦게 오면 통제기는 프레임을 박스 없이 표시하고, 늦게 온 메타데이터는 버린다 (`video.md` 6장).

## 4. FC telemetry 두절 (추종 중)

```mermaid
sequenceDiagram
    participant fc as FC
    participant fct as FcTelemetry (onboard)
    participant spr as SetpointRelay (onboard)
    participant ctl as FollowController (추종 제어)
    participant asr as ArgusStatusReporter (onboard)
    participant gcs as TelemetryModel·ArgusStatusModel (gcs)

    Note over fc,fct: FC telemetry 가 오지 않음
    fct->>fct: 2 s 경과 → FC_CONNECTED = 0
    fct--)ctl: fc/attitude·fc/state 중단
    spr->>spr: FC 연결 없음 → setpoint 송신 중단
    Note over fc: COM_OF_LOSS_T 1 s 뒤 Offboard 이탈, COM_OBL_RC_ACT 동작 (Hold, 미결 D4)
    ctl->>ctl: mission_state = FAULT
    ctl--)asr: PUB follow/status mission_state=FAULT
    asr->>gcs: ARGUS_STATUS FAULT, health FC_CONNECTED=0
    Note over gcs: 통제기는 FC HEARTBEAT 3 s 없음으로 "연결 끊김" 을 따로 표시
```

- 링크가 다시 연결되어도 추종을 자동으로 재개하지 않는다. FAULT 에서는 운용자의 명시적 TRACK_STOP 을 받고 최신 FC 상태·FC 실제 Hold·RC 비개입을 확인한 뒤에만 HOLD_WAIT 로 돌아간다 (`icd.md` 6장, `state_machine.md`).
- onboard 는 추종 제어의 보고와 관계없이 FC 상태를 스스로 확인해 위험한 송신을 막는다 (`icd.md` 6.6).
- 경로가 FC → mavlink-router → onboard 이므로 mavlink-router 가 멈춰도 같은 흐름이 된다. 이때는 통제기 링크도 함께 끊긴다.

## 검토 필요 사항

1. 해소 (0.3): 1절의 Offboard 진입 실패(요청 실패, ACK 실패, 실제 모드 확인 시간 초과)는 outcome=FAILED → ACK FAILED, `mission_state` FAULT (`icd.md` 4.2).
2. 해소 (0.3): TRACK_START 는 FC 가 실제 Hold 이거나 유효한 Position 인계를 거쳤을 때만 수락 (`icd.md` 4.2 ⑥).
3. 2절: `ctl` 이 `fc/attitude` 를 프레임 시각에 맞춰 쓰는지, 최신 메시지를 쓰는지는 추적·추종 담당의 내부 설계.
4. 4절: FC telemetry 두절 때 `mission_state` 를 FAULT 로 바꾸는 주체. ICD 6.2 는 onboard 의 health 와 setpoint 중단만 정하고, `mission_state` 는 추종 제어가 정한다(`state_machine.md`). 그림은 추종 제어가 `fc/*` 중단을 보고 FAULT 로 바꾸는 것으로 가정했다. 판단 시간(2 s)을 추종 제어도 같은 값으로 쓰는지 확인 필요.

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| 0.1 | 2026-10-19 | 초안 (#31) |
| 0.2 | 2026-10-19 | 탐지 쪽 참여자를 실제 클래스 이름으로 (`PerceptionWorker`, `PythonVisionAdapter`, #33 리뷰) |
| 0.3 | 2026-10-19 | 추적·추종 담당 리뷰 반영(#33): Offboard 진입 확인은 ACK + 실제 모드(1절), FC 표본은 프레임의 호스트 단조 시각으로 고르고 시각 차 50 ms·수신 후 경과 시간 200 ms(2절), FAULT 복구 조건과 onboard 독립 감시(4절), "FC 표본" 을 "fc/attitude·fc/state 메시지" 로 |
