/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/src/application/control/MissionStateMachine_TBD.cpp
역할: 시작·추종·Hold·Scan·Manual·Fault 임무 전환
전체 흐름도: docs/design/whole-flowchart.mmd — FSM, LOOP, FOLLOW, GUARD, API
상세 흐름도: docs/design/flow.mmd — STARTUP, FOLLOWING, HOLD_TRANSITION, WAITING, SCANNING, FAULTS, IDLE, PREP, READY, ENTER, ACK, STARTING, FINAL_DET, COMMIT, HOLD_REQ, HCLASS, HZERO, HSTOP, HACK, HFAILED, BLOCK_WAIT, WAIT, SCAN_ENTER, SCAN_ACK, SCAN_COMMIT, MANUAL, FAULT, NO_RETRY

설계 조건 (실행 구현 아님):
- 최초 시작/명시적 재시작은 후보 확인, 준비 검사, 정지 setpoint 사전 송신, 실제 Offboard 확인, 최신 실제 검출 재확인 뒤 원자적 COMMIT 순서다.
- Hold 전환은 재개차단 ON, 의도 폐기, epoch 증가, 거리 적분 리셋이며 Hold 요청은 사건당 1회다. hold_attempted와 실제 모드를 확인한다.
- HOLD_WAIT는 후보 표시만 한다. Scan은 운용자 명령과 조건 확인 후 시작하며 후보 발견은 자동 재추종이 아니다.
- Manual은 운용자 모드를 존중하고 Offboard 명령을 중지한다. Fault/전환 실패는 Hold 자동 재요청을 하지 않는다.
- 기체 상태에 관계없이 Hover가 보장된다고 주장하지 않는다. 상태 전환 코드와 타이머는 미구현이다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
