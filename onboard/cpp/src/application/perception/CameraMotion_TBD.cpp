/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/src/application/perception/CameraMotion_TBD.cpp
역할: FC 자세를 프레임 시각에 정렬하는 회전 CMC
전체 흐름도: docs/design/whole-flowchart.mmd — MOTION, TIME, TRACK
상세 흐름도: docs/design/flow.mmd — CMC, FRAME, FC

설계 조건 (실행 구현 아님):
- 이전·현재 프레임의 실제 노출 시각에 FC 자세를 정렬한다. 병진 시차는 보상하지 않는 설계다.
- ByteTrack, OC-SORT, Hybrid-SORT HMIoU OFF/ON 모두 같은 FC 회전 CMC 조건을 사용한다.
- 기존 ICameraMotion 포트와 ITracker.hpp의 argus::CameraMotion 자료형을 유지한다. 이 파일은 동명 클래스 선언이 아니다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
