/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/include/argus/contracts/TargetState_TBD.hpp
역할: 운용자가 승인한 선택 ID와 대상 유효성의 계약
전체 흐름도: docs/design/whole-flowchart.mmd — STORE, TARGET, FOLLOW
상세 흐름도: docs/design/flow.mmd — PREP, PREP_CHECK, COMMIT, SNAP, FGATE, TRACK, TTL, CONF

설계 조건 (실행 구현 아님):
- 준비 단계의 후보 ID는 pending이다. 승인된 시작 사건에서만 선택 ID와 임무 상태를 원자적으로 확정한다.
- 최신 실제 검출과 선택 ID의 연결을 확인하고 겹침·ID 불확실성을 임의로 승인하지 않는다.
- Predicted 결과나 새 배치 발행으로 마지막 실제 검출 시각과 TTL을 갱신하지 않는다. 타입·필드 선언은 미구현이다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
