/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/include/argus/application/commands/CommandGateway_TBD.hpp
역할: 운용자 요청 ID·중복 제거·결과 응답
전체 흐름도: docs/design/whole-flowchart.mmd — API, UI, LOOP
상세 흐름도: docs/design/flow.mmd — PREP, OP_SCAN, WEVENT, NO_RETRY

설계 조건 (실행 구현 아님):
- overview에서 GCS 영역 API로 표시된 역할이다. 이 Pi 파일은 계획된 역할의 표시용이며 실행 위치나 Windows 서비스 구현을 확정하지 않는다.
- 운용자 선택/시작/중단/재탐색 요청의 ID·중복 제거·결과 응답을 제어 이벤트와 연결해야 한다.
- 기존 CommandRequest.hpp 자료형은 유지한다. 자동 재시작·자동 재추종·Hold 재시도를 만들어서는 안 된다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
