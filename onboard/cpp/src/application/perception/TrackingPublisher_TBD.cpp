/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/src/application/perception/TrackingPublisher_TBD.cpp
역할: 실제 추적 결과의 tracking/tracks 발행
전체 흐름도: docs/design/whole-flowchart.mmd — DATA, TRACK, TARGET
상세 흐름도: docs/design/flow.mmd — OBS, TRACK, TTL, CONF

설계 조건 (실행 구현 아님):
- Confirmed/Predicted/Lost, full_box, 선택 후보 ID, 마지막 실제 검출 시각·TTL·좌표계·출처를 보존해야 한다.
- 실제 Tracker가 없으므로 class_id를 track_id로 쓰거나 가짜 트랙을 발행하지 않는다. 기존 Track.hpp 계약은 유지한다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
