/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/src/application/control/Follower_TBD.cpp
역할: Confirmed·Predicted·Scan 가이던스 의도 생성
전체 흐름도: docs/design/whole-flowchart.mmd — FOLLOW, GEO, FSM, GUARD
상세 흐름도: docs/design/flow.mmd — GUIDANCE, CONF, DIR, FOV, RISK, RECOVER, RANGE, NO_RANGE, METRIC, DIST, YAW_OK, TURN, APPROACH, FUTURE_RISK, BRAKE, SETTLE, FOLLOW_PI, FOLLOW_INTENT, PRED_INTENT, SCAN_INTENT

설계 조건 (실행 구현 아님):
- Confirmed는 방향과 거리 품질을 분리한다. SCALE_ONLY/INVALID는 전진 0이며 METRIC만 미터 거리 기반 추종에 사용할 수 있다.
- 시야 위험·회전 우선·접근 위험은 전진 0이다. 원거리 접근, 중거리 감속, 목표거리 추종 조건을 구분한다.
- Predicted는 전진·횡·yaw 0, 독립 고도 유지, 거리 적분 정지다. Scan은 제한 yaw 의도와 전진·횡 0이다.
- yaw P/거리 P 또는 PI/고도 P의 이득·속도·가속·저크·적분 한도를 임의 확정하지 않는다. VehiclePort로 직접 송신하지 않는다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
