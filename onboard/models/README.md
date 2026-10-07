# 온보드 실행용 모델

학습·변환은 저장소 최상위 `models/`에서 수행하고, 검증한 실행용 HEF와 대응 라벨은 이 폴더에 함께 배치합니다.
현재 로컬 소스에는 실제 HEF가 없으므로 이 저장소에도 HEF를 포함하지 않았습니다.
`config/coco80.txt`와 `config/labels.txt`는 기존 라벨 자료이며, 배포 모델과 일치하는 파일을 선택해야 합니다.

## 각 Pi에 배치

`onboard/`에서 아래 경로의 예시를 실제 검증한 모델·라벨 파일 경로로 바꿔 실행합니다.

```bash
mkdir -p models/deployed
cp /path/to/validated_hailo8_model.hef models/deployed/model.hef
cp /path/to/matching_labels.txt models/deployed/labels.txt
```

`config/runtime.env`는 다음 두 경로를 사용합니다. 두 경로는 모두 `onboard/` 기준입니다.

```bash
ARGUS_HEF=models/deployed/model.hef
ARGUS_LABELS=models/deployed/labels.txt
```

두 Pi에 같은 모델·라벨 세트를 배치하고 아래 명령으로 두 파일의 SHA256을 비교합니다.
코드 태그, 모델 버전, 두 해시와 검증 결과를 함께 기록합니다.

```bash
sha256sum models/deployed/model.hef models/deployed/labels.txt
```

`models/deployed/`의 실제 HEF·라벨은 장치별 배포 파일이며 Git에서 제외합니다.
별도 배포 저장소를 사용할 경우 HEF와 라벨의 지정 버전 및 SHA256을 함께 고정하고
이 두 경로로 다운로드하도록 설치 절차를 구성합니다. 현재 자동 다운로드는 구현하지 않았습니다.

Hailo-8 호환성, 입력·출력 형식 및 라벨 순서는 `docs/CAMERA_AND_MODEL_NOTES.md`에 따라 점검합니다.
`scripts/verify_rpi.sh`는 정적 검사이며 실제 카메라 추론 및 라벨 순서 검증을 대신하지 않습니다.
