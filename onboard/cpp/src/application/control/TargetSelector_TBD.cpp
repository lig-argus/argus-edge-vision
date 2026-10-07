/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/src/application/control/TargetSelector_TBD.cpp
역할: 운용자 지정 단일 ID 선택과 동일성 확인
전체 흐름도: docs/design/whole-flowchart.mmd — TARGET, STORE, DATA
상세 흐름도: docs/design/flow.mmd — PREP, PREP_CHECK, FINAL_DET, COMMIT, FGATE, CONF, SCAN_CAND, WAIT_CAND

설계 조건 (실행 구현 아님):
- 운용자가 확인한 1ID를 고정하며 겹침·ID 불확실성에서 새 ID를 임의로 승인하지 않는다.
- pending 후보와 실제 활성 선택을 구분한다. Hold/Scan의 새 후보는 표시·기록만 하며 재추종 명령이 아니다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
