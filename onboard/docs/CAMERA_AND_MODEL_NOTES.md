# 입력·전처리·모델

## 입력과 전처리

| 입력 | 현행 경로 |
|---|---|
| `ir_sdk` | 기존 C++ i3_raw14_capture → I3SdkFrameSource → RAW14_LE16 Frame |
| `ir_v4l2` | C++ OpenCV helper → OpenCvFrameSource → BGR8 Frame |
| `replay` | C++ OpenCV helper의 유한 영상 읽기, 정상 EOF 구분 |
| Gazebo 영상/Pose | 이 패키지에서는 미구현, 생성 요청을 명확히 거부 |

SDK는 640×480·I3R4 헤더·sequence·14비트 범위를 검사하며 초과 값을 masking하지 않는다.
Python Preprocessor는 기존 minmax/fixed14 변환 후 BGR 3채널로 바꾼다.
letterbox는 비율 유지, INTER_CUBIC, pad 114, BGR→RGB, contiguous uint8이며
YOLOXDetector는 기존 PyHailoRT 4.23/Hailo NMS 결과를 원본 픽셀 `full_box`로 역변환한다.

현재 입력은 실제 센서 노출 시각을 제공하지 않는다. SDK 콜백/호스트 읽기 시각을 노출 시각으로
대체하거나 FC 회전 CMC가 정렬됐다고 표시하지 않는다.
카메라 실패는 C++의 fault로 끝난다. V4L2를 SDK로 자동 변경하지 않는다.

## 모델 점검

기존 HEF와 라벨은 전환 과정에서 유지했다. COCO80 시험 모델의 IR 대상 정확도는 별도 검증 대상이다.
아래는 사용자가 모델 교체를 지시했을 때의 점검 항목이며 자동 교체 지침이 아니다.

- HEF가 Hailo-8용인지 확인한다. Hailo-8L용과 혼용하지 않는다.
- 입력 1개·RGB 3채널, 입력 크기/형식과 현재 전처리의 호환성을 확인한다.
- 현재 검출기가 요구하는 Hailo NMS 출력 구조와 클래스 수를 확인한다.
- 라벨 수와 **class ID 순서**를 모델과 직접 대조한다. 기본 COCO80은 `config/coco80.txt`, 자체 모델은 해당 라벨 파일을 사용한다.
- runtime.env의 `ARGUS_HEF`, `ARGUS_LABELS`를 확인하고 합성/녹화 Replay 추론으로 박스 좌표와 출력을 검증한다.
- 새 모델의 지연·검출 품질은 다시 측정한다. 과거 Python/hull ROI 측정값을 현행 baseline으로 사용하지 않는다.

`hailortcli parse-hef <HEF경로>`로 메타데이터를 확인할 수 있다.
현재 `verify_rpi.sh`는 라벨 파일 존재를 확인하지만 라벨 수/순서와 HEF 클래스의 일치를 검사하지 않는다.
정적 검사의 성공을 실제 추론·정확도 검증으로 해석하지 않는다.

입력 설정, 빌드·실행·미리보기·검증 명령은
[설치·실행 안내](INSTALL_AND_RUN_RPI5_HAILO8.md)에 모았다.
예전 모델 체크리스트와 성능 기록은 [과거 문서](../legacy/README.md)에 보관한다.
