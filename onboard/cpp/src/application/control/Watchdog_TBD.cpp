/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/src/application/control/Watchdog_TBD.cpp
역할: 영상·제어 루프와 독립된 상태 감시
전체 흐름도: docs/design/whole-flowchart.mmd — CONTROL, HEALTH, GUARD
상세 흐름도: docs/design/flow.mmd — WATCHDOG, WTICK, WSTATE, WF, WS, WW, WP, WTERMINAL, WOK, SAFETY, FEXIT, SMODE

설계 조건 (실행 구현 아님):
- 영상 프레임 도착과 독립된 Timer에서 임무 상태별 프레임·TTL·FC·실제 모드·RC·전환 결과를 감시한다.
- 안전 이벤트가 우선하며 명령 epoch를 무효화한다. Fault/Manual은 명령 생성 없이 관측한다.
- 상세 flow의 20~50 Hz는 미시험 후보이며 실행 주기로 확정하지 않는다. Hover 보장을 주장하거나 Hold 자동 재요청을 하지 않는다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
