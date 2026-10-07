# Ground Control System (GCS)

PC에서 실행하는 지상 통제 프로그램, 상태 표시, 관측 구독과 미리보기 도구를 배치합니다.
기존 `ground_control_station/` 디렉터리의 배치 위치를 `ground_control/`로 정리했습니다.
현재는 폴더 틀이며 GCS 애플리케이션이나 기체 명령 기능을 구현하지 않았습니다.

온보드의 공개 메시지 계약은 `onboard/docs/BUS_CONTRACT.md`를 따릅니다.
현재 패키지의 구독 CLI는 `onboard/python/argus_workers/tools/`에 있고,
Pi에서 합성한 JPEG를 구독하는 방식은 유지합니다.
