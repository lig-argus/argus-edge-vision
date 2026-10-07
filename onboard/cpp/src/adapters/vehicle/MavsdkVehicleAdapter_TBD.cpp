/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/src/adapters/vehicle/MavsdkVehicleAdapter_TBD.cpp
역할: MAVSDK 단일 송신자·텔레메트리·만료 감시
전체 흐름도: docs/design/whole-flowchart.mmd — MAV, PORT, PX4, STORE, MOTION
상세 흐름도: docs/design/flow.mmd — SEND, FC, ENTER, ACK, HSTOP, HACK, SCAN_ENTER, SCAN_ACK, PX4_FS

설계 조건 (실행 구현 아님):
- 유효한 Guard 결과만 직렬 송신하며 기존 IVehiclePort 계약을 보존한다. 텔레메트리는 시각/실제 모드와 함께 제공한다.
- Offboard set_velocity_body, 진입 확인, 사건당 1회 Offboard::stop/Hold 요청, 명령 만료 감시는 모두 미구현이다.
- 요청 응답을 실제 모드 확인으로 대신하지 않으며 Hold 실패를 자동 재시도하지 않는다. 연결·송신·시험값을 임의 생성하지 않는다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
