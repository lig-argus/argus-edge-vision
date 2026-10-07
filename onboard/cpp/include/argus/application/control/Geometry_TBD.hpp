/*미구현 임의 생성 금지*/

/*
상태: UNIMPLEMENTED_PLACEHOLDER (주석만 있는 미구현 표시 파일)
파일: cpp/include/argus/application/control/Geometry_TBD.hpp
역할: 원 영상 전체 박스에서 방향·거리 품질 계산
전체 흐름도: docs/design/whole-flowchart.mmd — GEO, TIME
상세 흐름도: docs/design/flow.mmd — RAY, DIR, FOV, RISK, RANGE, METRIC, DIST, NO_RANGE

설계 조건 (실행 구현 아님):
- 전체 박스(full_box)와 카메라/FC 정렬을 사용한다. hull ROI는 사용하지 않는다.
- 방향과 거리 품질을 분리하고 METRIC/SCALE_ONLY/INVALID를 구분해야 한다.
- 미터 거리는 접점/특징점·카메라 높이·지면 모델·기하/시각 유효성 확인이 필요하다. bbox 크기만으로 미터 거리를 임의 확정하지 않는다.
- 시야 판정 기준과 카메라 보정/높이 값은 미정이며 실제 구현은 없다.

사용자 승인 범위: 파일 배치와 이 설명 주석만.
사용자 명시적 승인 없이 구현·Mock 대체·팩토리 등록·CMake 연결 금지.
미정 수치·알고리즘·API·동작을 임의로 확정하거나 생성하지 않는다.
함수·클래스·자료형 선언, 반환값, 연결 및 송신은 이 파일에 없다.
이후 사용자의 명시적 구현 지시가 있으면 그 지시가 우선한다.
*/
