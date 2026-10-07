/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/include/argus/contracts/VehicleState_TBD.hpp
역할: FC 자세·위치·속도·실제 모드·시각·유효성 계약
전체 흐름도: docs/design/whole-flowchart.mmd — MAV, STORE, HEALTH, STATUS
상세 흐름도: docs/design/flow.mmd — FC, FHEALTH, FEXIT, SCHECK, WCHECK

설계 조건 (실행 구현 아님):
- 실제 텔레메트리의 시각과 유효성을 구분한다. 모드 요청 성공을 실제 모드 확인으로 간주하지 않는다.
- 노출 시각을 FC 자세와 정렬해야 한다. 호스트 수신 시각을 실제 노출 시각으로 대신하지 않는다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
