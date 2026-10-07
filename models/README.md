# 모델 학습·변환

PC/학습 환경의 YOLO 학습 설정, 평가, ONNX 내보내기, Hailo-8 HEF 변환과 모델 버전 정보를 배치합니다.
`datasets/`는 데이터셋 준비·관리 위치입니다. 현재 학습·변환 코드는 아직 이 폴더에 통합하지 않았습니다.

파이는 `onboard/`만 sparse-checkout하므로 이 최상위 폴더를 실행 경로에서 참조하지 않습니다.
검증한 배포 모델은 **HEF와 해당 라벨을 함께** `onboard/models/deployed/`에 배치합니다.
배치 방법은 `onboard/models/README.md`를 따릅니다.

실행용 Hailo-8 HEF, 입력·NMS 출력 형식, 라벨의 class ID 순서가 온보드 검출기와 일치해야 합니다.
학습 데이터, 가중치와 대형 변환 산출물은 Git 소스 이력에서 제외하고 별도 보관합니다.
