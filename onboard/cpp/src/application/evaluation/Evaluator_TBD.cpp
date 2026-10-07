/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/src/application/evaluation/Evaluator_TBD.cpp
역할: 로그와 정답을 사용하는 추적·제어 성능 평가
전체 흐름도: docs/design/whole-flowchart.mmd — LOG, DATA, STATUS, MOTION, GZ, MAV
상세 흐름도: docs/design/flow.mmd — OBS, FC, FOLLOW_INTENT, SEND

설계 조건 (실행 구현 아님):
- mode, t_frame, t_att, t_cmd, FPS, E2E, ID 유지율과 출처/정답 시각을 연결할 평가 역할이다.
- 기존 Recorder의 비동기 인지 CSV 기록은 구현돼 있다. 평가 지표·정답 정렬·제어 로그는 미구현이며 결과를 임의 생성하지 않는다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
