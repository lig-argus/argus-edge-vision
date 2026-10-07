/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/include/argus/contracts/GuidanceIntent_TBD.hpp
역할: 중재 전 가이던스 의도 계약
전체 흐름도: docs/design/whole-flowchart.mmd — FOLLOW, GUARD
상세 흐름도: docs/design/flow.mmd — FOLLOW_INTENT, PRED_INTENT, SCAN_INTENT, ARBITER, GUARD

설계 조건 (실행 구현 아님):
- 상세 flow의 mode·reason·네 축 setpoint 의도와 epoch/만료 연결은 구현 승인 시 확정한다.
- 기존 IVehiclePort.hpp의 ControlIntent를 재정의하거나 덮어쓰지 않는다.
- Predicted는 수평·yaw 0, Scan은 전진·횡 0이다. 가이던스 의도가 VehiclePort로 직접 송신되면 안 된다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
