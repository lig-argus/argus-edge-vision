/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/include/argus/application/control/LatestStateStore_TBD.hpp
역할: 최신 트랙·대상·기체 상태 저장소
전체 흐름도: docs/design/whole-flowchart.mmd — STORE, DATA, MAV
상세 흐름도: docs/design/flow.mmd — OBS, SNAP, FC, COMMIT, HOLD_REQ, SAFETY

설계 조건 (실행 구현 아님):
- 제어는 최신 유효 상태를 원자적으로 읽으며 epoch와 임무/차단/선택 ID를 함께 다룬다.
- 기존 인지 FrameQueue의 용량 2 FIFO/drop-oldest는 별도 정책이다. 이 저장소를 이유로 인지 큐를 변경하지 않는다.
- Predicted 또는 새 배치 발행으로 트랙 TTL을 연장하지 않는다. 저장·동기화 로직은 미구현이다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
