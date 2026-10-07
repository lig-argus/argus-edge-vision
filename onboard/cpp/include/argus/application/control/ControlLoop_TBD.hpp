/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/include/argus/application/control/ControlLoop_TBD.hpp
역할: 영상 도착과 독립된 제어 주기
전체 흐름도: docs/design/whole-flowchart.mmd — LOOP, STORE, TARGET, GEO, FSM, GUARD, API
상세 흐름도: docs/design/flow.mmd — TICK, SNAP, MSTATE, ARBITER, DROP

설계 조건 (실행 구현 아님):
- 주기 Timer에서 이벤트를 수집하고 하나의 ControlContext로 선택·기하·임무·Guard를 판단한다.
- Watchdog은 별도 실행이어야 한다. 미정 제어 주기나 임계값을 임의 확정하지 않는다.
- 오래된 epoch 의도는 폐기 후 상태를 재평가한다. 현재 인지 실행부에 이 루프를 자동 연결하지 않는다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
