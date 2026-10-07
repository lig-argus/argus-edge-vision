/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/include/argus/contracts/ControlContext_TBD.hpp
역할: 한 제어 주기에서 함께 사용하는 원자적 상태 스냅샷 계약
전체 흐름도: docs/design/whole-flowchart.mmd — LOOP, GUARD, STORE
상세 흐름도: docs/design/flow.mmd — SNAP, MSTATE, ARBITER, GUARD

설계 조건 (실행 구현 아님):
- 임무 상태·선택 ID·재개차단·관측·FC 상태·epoch를 같은 원자적 스냅샷으로 다룬다.
- 제어 판단과 최종 Guard가 같은 ControlContext를 사용한다. 구체적인 필드와 동기화 구현은 미정이다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
