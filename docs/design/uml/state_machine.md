# State Machine Diagram — 임무 상태와 표적 상태의 transition

| 항목 | 내용 |
|---|---|
| 참조 문서 | `common/mavlink/message_definitions/argus.xml` (`ARGUS_MISSION_STATE`, `ARGUS_TARGET_STATE`), `docs/spec/icd.md` 4장 (명령), 6장 (두절), `docs/spec/process_interfaces.md` 6장 |
| 주체 | 두 상태 모두 추종 제어가 정하고 `follow/status` 로 mavlink_gateway 에 주며, mavlink_gateway 가 `ARGUS_STATUS` 로 통제기에 그대로 옮긴다 (`icd.md` 3.1) |
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
    HOLD_TRANSITION --> HOLD_WAIT : 정지 상태 확인
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
| HOLD_TRANSITION | 추종·Scan 을 끝내고 정지하는 중 | → Hold 또는 Offboard 제자리 유지 |
| HOLD_WAIT | 기체가 정지 상태(`icd.md` 4.2)인 것을 확인하고 대기. 자동으로 추종을 재개하지 않는다 | Hold 또는 Offboard 제자리 유지 |
| SCAN_ACTIVE | yaw 스캔 중 | Offboard (`icd.md` 4.2) |
| MANUAL | 조종사(RC) 또는 통제기가 모드를 바꿔 추종이 끝남 | 조종사·통제기가 정한 모드 |
| FAULT | FC 두절, 추종 제어 두절, Offboard 진입 실패 등 고장으로 멈춤 | FC failsafe 에 따름 |

transition 의 event [guard] / effect:

| transition | event | guard | effect | 출처 |
|---|---|---|---|---|
| IDLE·HOLD_WAIT → PREPARING | TRACK_START 수신 | `icd.md` 4.2 조건 ①~⑥ 충족 (⑥ 기체의 정지 상태 또는 유효한 Position 인계) | ACK IN_PROGRESS | `icd.md` 4.2 |
| PREPARING → FOLLOW_ACTIVE | Offboard 진입 확인 | DO_SET_MODE 의 ACK 와 HEARTBEAT 로 확인한 실제 모드가 모두 Offboard | ACK ACCEPTED | `icd.md` 4.2 |
| PREPARING → FAULT | Offboard 진입 실패 | 모드 요청 실패, ACK 실패, 실제 모드 확인 시간 초과 중 하나 | ACK FAILED | `icd.md` 4.2 |
| PREPARING → HOLD_TRANSITION | TRACK_STOP 수신 | — | 진행 중이던 TRACK_START 의 ACK CANCELLED, 정지 요청 | #33 리뷰 (추적·추종 담당), `icd.md` 4.1 |
| IDLE·HOLD_WAIT → SCAN_ACTIVE | SCAN_START 수신 | 조건 충족, 기체의 정지 상태 | ACK IN_PROGRESS | `icd.md` 4.2 |
| SCAN_ACTIVE → HOLD_WAIT | yaw 회전량 도달, timeout, 추적 후보 발견 | — | 정지, ACK ACCEPTED | `icd.md` 4.2 |
| FOLLOW_ACTIVE → HOLD_TRANSITION | TRACK_STOP 수신, 통제기 HEARTBEAT 3 s 없음, target_state → LOST | — | 정지 요청 | `icd.md` 4.2, 6.1, 아키텍처 문서 2.7·2.11 |
| SCAN_ACTIVE → HOLD_TRANSITION | TRACK_STOP 수신, 통제기 HEARTBEAT 3 s 없음 | — | TRACK_STOP 이면 진행 중이던 SCAN_START 의 ACK CANCELLED, 정지 요청 | `icd.md` 4.1, 4.2, 6.1 |
| HOLD_TRANSITION → HOLD_WAIT | 정지 상태 확인 | — | ACK ACCEPTED | `icd.md` 4.2 Sequence Diagram |
| FOLLOW_ACTIVE·SCAN_ACTIVE → MANUAL | Offboard 이탈 | 원인이 조종사 RC 또는 통제기 모드 변경 | — | `icd.md` 4.3, 아키텍처 문서 2.11·2.20 |
| PREPARING·FOLLOW_ACTIVE·SCAN_ACTIVE → FAULT | FC 연결 두절(ATTITUDE 0.5 s 없음) 또는 HEARTBEAT 3 s 없음, `follow/status` 3 s 없음, setpoint 유효 시간 초과, Offboard 이탈 | Offboard 이탈은 원인이 failsafe 일 때 (PREPARING 제외) | 정지 요청 | `icd.md` 6.2, 6.4, 4.3, `process_interfaces.md` 6장 |
| MANUAL·FAULT → HOLD_WAIT | 운용자의 명시적 TRACK_STOP 수신 | 최신 FC 상태, 기체의 정지 상태, RC 비개입을 모두 확인 | — (추종은 다시 시작하지 않는다. 다시 추종하려면 표적을 새로 고르고 TRACK_START) | `icd.md` 6장 |

## 2. `ARGUS_TARGET_STATE`

```mermaid
stateDiagram-v2
    direction LR
    [*] --> NONE
    NONE --> CONFIRMED : TRACK_START 수락
    CONFIRMED --> PREDICTED : 검출 없음
    PREDICTED --> CONFIRMED : 같은 표적 다시 검출
    PREDICTED --> LOST : 예측 유지 기한 만료·표적 무효
    CONFIRMED --> NONE : TRACK_STOP
    PREDICTED --> NONE : TRACK_STOP
    LOST --> NONE : TRACK_STOP
```

| transition | event | guard | effect | 출처 |
|---|---|---|---|---|
| NONE → CONFIRMED | TRACK_START 수락 | 표적이 최신 검출과 연결됨 | — | `icd.md` 4.2 |
| CONFIRMED → PREDICTED | 검출 없음 | 선택된 표적이 있고 그 표적의 마지막 상태 기록을 보관 중이며, 추종 제어 내부 단계가 GAP_HOLD·REACQUIRE·CONFIRM(검출이 끊긴 뒤 같은 표적을 다시 찾는 동안의 단계) 중 하나 | 예측으로 유지. 수평·yaw 출력 0, 고도는 기존 정책 | `argus.xml`, #33 리뷰 (추적·추종 담당) |
| PREDICTED → CONFIRMED | 같은 표적 다시 검출 | 다시 검출한 관측이 연속 확인 조건을 충족 | — | `argus.xml`, #33 리뷰 |
| PREDICTED → LOST | 예측 유지 기한 만료, 또는 선택 표적과 그 표적 기록이 무효가 됨 (무효가 되는 조건은 추종 제어 내부 구현) | 기한은 마지막으로 검증된 실제 관측 기준이며, 반복 조회나 빈 프레임 수신으로 늘리지 않는다. 기한 값은 `icd.md` 미결 E4 | 추종 제어가 Hold 요청 | `argus.xml`, `icd.md` 부록 B, #33 리뷰 |
| CONFIRMED·PREDICTED·LOST → NONE | TRACK_STOP 을 수락(ICD 4.2 조건 통과)해 표적 선택에 적용함 | — | 표적 선택 해제. 정지 완료를 기다리지 않는다 | `icd.md` 4.2, #33 리뷰 |

- TRACK_STOP 뒤에는 `target_state` NONE 과 `mission_state` HOLD_TRANSITION 이 함께 보고될 수 있다. 표적 해제는 TRACK_STOP 을 적용한 때, 기체 정지 완료는 기체의 정지 상태를 확인해 `mission_state` 가 HOLD_WAIT 가 된 때다.
- PREDICTED 보고와 제어 허가는 별개다. PREDICTED 여도 `fc/attitude`·`fc/state` 메시지 조건(프레임 시각과의 차 50 ms 이내, 수신 후 200 ms 이내), RC 개입, FC 제어 명령 송신권, 명령 유효 기한 검사는 그대로 적용한다.
- LOST 가 되면 추종 제어가 정지를 요청한다. 그래서 `mission_state` 의 FOLLOW_ACTIVE → HOLD_TRANSITION 조건에 target_state LOST 가 들어간다.

## 3. FC 명령 송신 상태 (`FcCommandState`)

mavlink_gateway 가 FC 로 보내는 Offboard 진입 명령과 setpoint 의 상태다. `mission_state`(1절)는 추종 제어가 정하고, 이 상태는 mavlink_gateway 의 `FcControl` 이 가진다 (`class_mavlink_gateway.md` 2절). 규약은 `docs/spec/icd.md` 4.2, 5장 (5.3 착륙 잠금, 5.4 통제기로의 복귀), 6.2, 6.6.

```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle --> Entering: 진입 요청 수락 / DO_SET_MODE(Offboard) 송신
    Idle --> Hovering: FC 가 Offboard (mavlink_gateway 재시작, 외부의 변경)
    Entering --> Following: FC 의 응답과 Offboard 를 모두 확인
    Entering --> CancellingEntry: 정지, FC 의 거부, 확인 시간 초과, 비행 상태를 받지 못함, 유효한 setpoint 없음, fc_epoch 변경, 송신 실패
    Entering --> Hovering: 확인 시간 초과 (응답 없이 Offboard)
    Following --> Hovering: 정지, setpoint 유효 시간 초과, 비행 상태를 받지 못함, fc_epoch 변경
    Following --> Locked: FC 가 Offboard 를 벗어남
    CancellingEntry --> Hovering: FC 가 Offboard
    CancellingEntry --> Idle: 진입 기한 후의 비행 상태가 Offboard 아님
    Hovering --> Following: 진입 요청 수락 (진입 명령 없음)
    Hovering --> Locked: FC 가 Offboard 를 벗어남
    Locked --> Idle: 운용자의 해제 (비행 상태가 최신일 때)
    Entering --> Landing: 착륙 명령 전달
    Following --> Landing: 착륙 명령 전달
    CancellingEntry --> Landing: 착륙 명령 전달
    Hovering --> Landing: 착륙 명령 전달
    Landing --> Idle: FC 가 Offboard 가 아님 (진입 기한이 있으면 그 후의 비행 상태)
    Idle --> HoverEntering: 복귀 명령 (45011, FC 모드 Position)
    Locked --> HoverEntering: 복귀 명령 (45011, FC 모드 Position)
    HoverEntering --> Hovering: 위치 setpoint 1 s 뒤 DO_SET_MODE(Offboard), FC 의 응답과 Offboard 를 모두 확인
    HoverEntering --> Idle: 진입 명령 전의 FC 모드 변경, 조종사 개입, 송신 실패, 정지, 비행 상태를 받지 못함
    HoverEntering --> CancellingEntry: 진입 명령 뒤의 FC 거부, 확인 시간 초과, 정지, fc_epoch 변경
    HoverEntering --> Landing: 착륙 명령 전달
```

| 상태 | 주기(20 Hz)마다 FC 로 보내는 setpoint | FC 제어 명령 송신권 |
|---|---|---|
| Idle | FC 가 Offboard 가 아님을 알고 추종 제어의 유효한 setpoint 가 있으면 그 값 (진입 전의 미리 송신). 그 밖에는 없음 | 없음 |
| Entering | 추종 제어의 setpoint | 진입을 요청한 세션 |
| Following | 추종 제어의 setpoint. 제자리 유지 중에 수락된 경우 첫 setpoint 를 받을 때까지는 제자리 유지 | 진입을 요청한 세션 |
| CancellingEntry | 속도 0 | 없음 |
| Hovering | 정한 위치와 yaw 를 목표로 하는 위치 setpoint. 자세와 local 위치가 최신(0.2 s 이내)인 첫 주기에 목표를 정하며, 그 전에는 속도 0 | 없음 |
| Locked | 없음 | 없음 |
| Landing | 정한 위치와 yaw 를 목표로 하는 위치 setpoint (Hovering 과 같음. 시작 상태의 목표를 이어 감) | 없음 |
| HoverEntering | 복귀 명령을 받은 때의 위치와 yaw 를 목표로 하는 위치 setpoint | 없음 |

- FC 의 모드는 HEARTBEAT(1 Hz)로만 알 수 있으므로 mavlink_gateway 는 넷 중 하나로 판단한다: Offboard, Offboard 아님, 아직 모름(쓸 수 있는 비행 상태가 아직 없음. 정상), 받지 못함(FC 연결 두절이거나 비행 상태를 받은 지 3 s 초과). "아직 모름" 은 어떤 전환도 일으키지 않는다.
- setpoint 송신을 끝내는 전환은 CancellingEntry → Idle, Landing → Idle, Following 또는 Hovering → Locked 뿐이다. 모두 FC 가 Offboard 가 아님을 비행 상태로 확인한 경우다. HoverEntering → Idle 은 진입 명령을 보내기 전(FC 가 Offboard 가 아님)에만 일어난다.
- Landing → Idle 은 통제기의 착륙 명령에 따른 이탈이므로 Locked 로 가지 않고, FC 가 Offboard 였으면 `fc/mode_event` 의 원인을 PilotSwitch(통제기의 모드 변경)로 보낸다 (`icd.md` 5.3). Landing, HoverEntering 에서는 추종 제어의 Offboard 진입 요청을 거부한다.
- CancellingEntry 는 취소한 진입의 기한 후에 받은 비행 상태만으로 "Offboard 아님" 을 판단한다. 기한 전의 값은 FC 가 아직 전환하지 않은 때의 값일 수 있다.
- `mission_state` 와의 관계: `fc/state` 의 `offboard_hover` 는 Hovering 에서 위치 setpoint 를 보낸 주기에만 true 다. Locked 가 되면 `fc/mode_event` 로 Offboard 이탈과 원인이 전달되고 추종 제어가 MANUAL 또는 FAULT 를 보고한다.

## 4. 검토 필요 사항

- 해소 (0.2): 앞의 미결 A(PREPARING 에서 Offboard 진입 실패 뒤 상태)는 FAULT, 미결 B(MANUAL·FAULT 의 복구)는 명시적 TRACK_STOP 과 안전 조건 확인 뒤 HOLD_WAIT 로 정했다 (`icd.md` 4.2·6장, #33·#36 리뷰).
- 해소 (0.2): 미결 C(표적 상태가 NONE 이 되는 시점)는 TRACK_STOP 을 적용한 때, 미결 D(PREDICTED 유지 조건)는 선택 표적과 그 표적 기록 보관 + 같은 표적을 다시 찾는 단계 + 예측 유지 기한 이내로 정했다 (2절, #33 리뷰).
- `follow/status` 의 `target_phase` 는 이 두 상태와 별개 필드다 (`process_interfaces.md` 5.1.2). 값이 확정되면 이 문서에 추가한다.

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| 0.1 | 2026-10-16 | 초안 (#31) |
| 0.2 | 2026-10-19 | 추적·추종 담당 리뷰 반영(#33): Offboard 진입 확인은 ACK + 실제 모드, 실패는 FAULT, PREPARING 중 TRACK_STOP, MANUAL·FAULT 복구는 TRACK_STOP + 안전 조건 → HOLD_WAIT. 미결 A·B 해소, C·D 추가 후 해소(표적 NONE 시점, PREDICTED 유지 조건), LOST → NONE 도 TRACK_STOP 적용 때로 |
| 0.3 | 2026-11-08 | PREPARING, SCAN_ACTIVE 에서 TRACK_STOP 수신 시 진행 중 명령의 ACK CANCELLED (#62 리뷰) |
