/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/include/argus/contracts/SystemStatus_TBD.hpp
역할: 임무·대상·기체·진단을 전송할 상태 계약
전체 흐름도: docs/design/whole-flowchart.mmd — STATUS, HEALTH, TARGET, FSM, MAV
상세 흐름도: docs/design/flow.mmd — SNAP, MSTATE, FAULT, MANUAL, PX4_FS

설계 조건 (실행 구현 아님):
- 임무 상태·TargetState·기체 상태·진단의 출처와 시각을 보존해야 한다.
- 공개 system/status schema와 필드는 미구현이다. 기존 perception/observations v2나 진단 schema를 임의 변경하지 않는다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
