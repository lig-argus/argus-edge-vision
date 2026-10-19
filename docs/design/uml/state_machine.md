# State Machine Diagram — 임무 상태와 표적 상태의 transition

| 항목 | 내용 |
|---|---|
| 참조 문서 | `common/mavlink/message_definitions/argus.xml` (`ARGUS_MISSION_STATE`, `ARGUS_TARGET_STATE`), `docs/spec/icd.md` 4장 (명령), 6장 (두절), `docs/spec/process_interfaces.md` 6장 |
| 주체 | 두 상태 모두 추종 제어가 정하고 `follow/status` 로 onboard 에 주며, onboard 가 `ARGUS_STATUS` 로 통제기에 그대로 옮긴다 (`icd.md` 3.1) |
| 표기 | transition 라벨은 UML `event [guard] / effect`. 그림에는 event 만, guard·effect·출처는 표에. 규약에 event 가 없는 transition 은 `[미결]` |

## 1. `ARGUS_MISSION_STATE`

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> PREPARING : TRACK_START 수락
    HOLD_WAIT --> PREPARING : TRACK_START 수락
    PREPARING --> FOLLOW_ACTIVE : Offboard 진입 확인
    PREPARING --> FAULT : Offboard 진입 실패
    PREPARING --> HOLD_TRANSITION : TRACK_STOP
    IDLE --> SCAN_ACTIVE : SCAN_START 수락
    HOLD_WAIT --> SCAN_ACTIVE : SCAN_START 수락
    SCAN_ACTIVE --> HOLD_WAIT : Scan 종료
    FOLLOW_ACTIVE --> HOLD_TRANSITION : 정지 요구
    SCAN_ACTIVE --> HOLD_TRANSITION : 정지 요구
    HOLD_TRANSITION --> HOLD_WAIT : FC Hold 확인
    FOLLOW_ACTIVE --> MANUAL : Offboard 이탈, 수동
    SCAN_ACTIVE --> MANUAL : Offboard 이탈, 수동
    PREPARING --> FAULT : 두절
    FOLLOW_ACTIVE --> FAULT : 두절, failsafe
    SCAN_ACTIVE --> FAULT : 두절, failsafe
    MANUAL --> HOLD_WAIT : TRACK_STOP
    FAULT --> HOLD_WAIT : TRACK_STOP
```

| 상태 | 뜻 (`argus.xml`) | FC 모드 |
|---|---|---|
| IDLE | 추종·Scan 없음. 표적 선택 대기 | 임의 |
| PREPARING | TRACK_START 수락 뒤 Offboard 진입·확인 중 | → Offboard |
| FOLLOW_ACTIVE | 추종 중, setpoint 20 Hz | Offboard |
| HOLD_TRANSITION | 추종·Scan 을 끝내고 Hold 로 전환 중 | → Hold |
| HOLD_WAIT | FC 가 Hold 인 것을 확인하고 대기. 자동으로 추종을 재개하지 않는다 | Hold |
| SCAN_ACTIVE | yaw 스캔 중 | Hold 또는 Offboard [미결 E3, `icd.md`] |
| MANUAL | 조종사(RC) 또는 통제기가 모드를 바꿔 추종이 끝남 | 조종사·통제기가 정한 모드 |
| FAULT | FC 두절, 추종 제어 두절, Offboard 진입 실패 등 고장으로 멈춤 | FC failsafe 에 따름 |

transition 의 event [guard] / effect:

| transition | event | guard | effect | 출처 |
|---|---|---|---|---|
| IDLE·HOLD_WAIT → PREPARING | TRACK_START 수신 | `icd.md` 4.2 조건 ①~⑥ 충족 (⑥ FC 실제 Hold 또는 유효한 Position 인계) | ACK IN_PROGRESS | `icd.md` 4.2 |
| PREPARING → FOLLOW_ACTIVE | Offboard 진입 확인 | DO_SET_MODE 의 ACK 와 HEARTBEAT 로 확인한 실제 모드가 모두 Offboard | ACK ACCEPTED | `icd.md` 4.2 |
| PREPARING → FAULT | Offboard 진입 실패 | 모드 요청 실패, ACK 실패, 실제 모드 확인 시간 초과 중 하나 | ACK FAILED | `icd.md` 4.2 |
| PREPARING → HOLD_TRANSITION | TRACK_STOP 수신 | — | Hold 요청 | #33 리뷰 (추적·추종 담당) |
| IDLE·HOLD_WAIT → SCAN_ACTIVE | SCAN_START 수신 | 조건 충족, FC Hold | ACK IN_PROGRESS | `icd.md` 4.2 |
| SCAN_ACTIVE → HOLD_WAIT | yaw 회전량 도달, timeout, 추적 후보 발견 | — | 정지, ACK ACCEPTED | `icd.md` 4.2 |
| FOLLOW_ACTIVE → HOLD_TRANSITION | TRACK_STOP 수신, 통제기 HEARTBEAT 3 s 없음, target_state → LOST | — | Hold 요청 | `icd.md` 4.2, 6.1, 아키텍처 문서 2.7·2.11 |
| SCAN_ACTIVE → HOLD_TRANSITION | TRACK_STOP 수신, 통제기 HEARTBEAT 3 s 없음 | — | Hold 요청 | `icd.md` 4.2, 6.1 |
| HOLD_TRANSITION → HOLD_WAIT | FC Hold 확인 | — | ACK ACCEPTED | `icd.md` 4.2 Sequence Diagram |
| FOLLOW_ACTIVE·SCAN_ACTIVE → MANUAL | Offboard 이탈 | 원인이 조종사 RC 또는 통제기 모드 변경 | — | `icd.md` 4.3, 아키텍처 문서 2.11·2.20 |
| PREPARING·FOLLOW_ACTIVE·SCAN_ACTIVE → FAULT | FC telemetry 2 s 없음, `follow/status` 3 s 없음, setpoint 유효 시간 초과, Offboard 이탈 | Offboard 이탈은 원인이 failsafe 일 때 (PREPARING 제외) | 속도 0, Hold 요청 | `icd.md` 6.2, 6.4, 4.3, `process_interfaces.md` 6장 |
| MANUAL·FAULT → HOLD_WAIT | 운용자의 명시적 TRACK_STOP 수신 | 최신 FC 상태, FC 실제 Hold, RC 비개입을 모두 확인 | — (추종은 다시 시작하지 않는다. 다시 추종하려면 표적을 새로 고르고 TRACK_START) | `icd.md` 6장 |

## 2. `ARGUS_TARGET_STATE`

```mermaid
stateDiagram-v2
    direction LR
    [*] --> NONE
    NONE --> CONFIRMED : TRACK_START 수락
    CONFIRMED --> PREDICTED : 검출 없음
    PREDICTED --> CONFIRMED : 검출 다시 연결
    PREDICTED --> LOST : 예측 유지 한도 초과
    CONFIRMED --> NONE : TRACK_STOP
    PREDICTED --> NONE : TRACK_STOP
    LOST --> NONE : Hold 전환 완료
```

| transition | event | guard | effect | 출처 |
|---|---|---|---|---|
| NONE → CONFIRMED | TRACK_START 수락 | 표적이 최신 검출과 연결됨 | — | `icd.md` 4.2 |
| CONFIRMED → PREDICTED | 검출 없음 | — | 예측으로 유지 | `argus.xml` |
| PREDICTED → CONFIRMED | 검출이 다시 연결됨 | — | — | `argus.xml` |
| PREDICTED → LOST | 예측 유지 한도 초과 | 한도는 `system_architecture.md` 2.7, 값은 `icd.md` 미결 E4 | 추종 제어가 Hold 요청 | `argus.xml`, `icd.md` 부록 B |
| CONFIRMED·PREDICTED → NONE | TRACK_STOP 수신 | — | Hold 전환 완료 뒤 | `icd.md` 4.2 Sequence Diagram |
| LOST → NONE | Hold 전환 완료 | — | — | `argus.xml` |

- LOST 가 되면 추종 제어가 Hold 로 전환한다. 그래서 `mission_state` 의 FOLLOW_ACTIVE → HOLD_TRANSITION 조건에 target_state LOST 가 들어간다.

## 3. 검토 필요 사항

- 해소 (0.2): 앞의 미결 A(PREPARING 에서 Offboard 진입 실패 뒤 상태)는 FAULT, 미결 B(MANUAL·FAULT 의 복구)는 명시적 TRACK_STOP 과 안전 조건 확인 뒤 HOLD_WAIT 로 정했다 (`icd.md` 4.2·6장, #33·#36 리뷰).
- **[미결 C]** 표적 상태가 NONE 이 되는 시점: TRACK_STOP 을 받은 때인지, Hold 전환이 끝난 때(HOLD_WAIT)인지. 그림은 CONFIRMED·PREDICTED → NONE 을 TRACK_STOP 으로 그렸다. — 추적·추종
- **[미결 D]** PREDICTED 를 유지하는 조건: 예측 유지 한도 외에 어떤 조건(FC 표본의 유효성, 프레임 시각과 FC 표본 시각의 차 등)이 맞아야 PREDICTED 로 남는지. — 추적·추종
- `follow/status` 의 `target_phase` 는 이 두 상태와 별개 필드다 (`process_interfaces.md` 5.1.2). 값이 확정되면 이 문서에 추가한다.

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| 0.1 | 2026-10-16 | 초안 (#31) |
| 0.2 | 2026-10-19 | 추적·추종 담당 리뷰 반영(#33): Offboard 진입 확인은 ACK + 실제 모드, 실패는 FAULT, PREPARING 중 TRACK_STOP, MANUAL·FAULT 복구는 TRACK_STOP + 안전 조건 → HOLD_WAIT. 미결 A·B 해소, C·D 추가 |
