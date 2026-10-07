/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/src/adapters/gazebo/GazeboPoseAdapter_TBD.cpp
역할: SITL_POSE 좌표·시간 정규화
전체 흐름도: docs/design/whole-flowchart.mmd — GZ, POSE, OBS, TIME
상세 흐름도: docs/design/flow.mmd — OBS, FC

설계 조건 (실행 구현 아님):
- SITL_POSE 전용 Gazebo Transport pose를 좌표·시각·출처와 함께 정규화할 역할이다.
- 현재 온보드 Observation.hpp는 IMAGE2D 계약이다. POSE3D 확장은 미구현이며 기존 v2 공개 관측 schema를 임의 바꾸지 않는다.
- 기존 PC 단일 박스/AlphaBeta 시험 경로를 완성된 온보드 구현으로 복사·등록하지 않는다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
