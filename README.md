# ARGUS Edge Vision

Raspberry Pi 5 + Hailo-8 온보드 인지, 시뮬레이션, 지상 통제(GCS), 모델 학습·변환을 관리하는 저장소입니다.

## 저장소 구조

```text
simulation/       시뮬레이션 환경과 실험
ground_control/   지상 통제 프로그램(GCS)
models/           YOLO 학습·평가·ONNX/HEF 변환과 데이터셋 관리
onboard/          Raspberry Pi에 설치할 C++·Python 실행 패키지
```

`onboard/` 안에 CMake/Python 패키지 정의, 코드, 설정 예시, 설치·실행 스크립트,
systemd 템플릿, 테스트, 설계·운용 문서와 실행용 모델 배치 위치가 있습니다.
설치·빌드·실행 시 다른 최상위 폴더를 참조하지 않습니다.
카메라 SDK, HailoRT/PyHailoRT와 OS 라이브러리는 각 Pi에 준비해야 합니다.

현재 `onboard/`는 **perception_only**입니다. 실제 추적·FC 연결·비행 제어는 미구현입니다.
`simulation/`, `ground_control/`, 최상위 `models/`는 역할과 배치 위치를 정한 틀이며
해당 프로그램이나 학습 파이프라인이 구현됐다는 의미는 아닙니다.

## 두 Raspberry Pi 운영

| 장치 | 브랜치 | 적용 방식 |
|---|---|---|
| 테스트용 Pi | `develop` | 개발 코드를 설치·빌드하고 카메라·추론·미리보기 검증 |
| 드론 장착용 Pi | `main` 또는 검증한 릴리즈 태그 | 검증한 코드·HEF·라벨 조합을 수동 적용 |

개발 브랜치의 실제 이름은 `develop`입니다. 변경은 기능 브랜치에서 작업하고
PR로 `develop`에 병합한 뒤 기존 릴리즈 절차에 따라 `main`에 반영합니다.
[기여 규칙](CONTRIBUTING.md)을 따릅니다.

## Pi에서 onboard만 받기

테스트용 Pi의 최초 설치:

```bash
git clone --filter=blob:none --sparse -b develop \
  https://github.com/s-jje/argus-edge-vision.git
cd argus-edge-vision
git sparse-checkout set --cone onboard
cd onboard
```

드론 장착용 Pi에서는 `-b develop`을 `-b main` 또는 적용할 릴리즈 태그로 바꿉니다.
cone 모드는 `onboard/` 전체와 루트 README 등 최상위 파일을 표시합니다.
루트 `package.json`은 개발자의 커밋 메시지 검사 도구용이며, Pi 실행에는 `npm install`이 필요하지 않습니다.

이후 [onboard 설치·실행 안내](onboard/docs/INSTALL_AND_RUN_RPI5_HAILO8.md)에 따라
의존성 준비, Python 환경/C++ 빌드, HEF·라벨 배치와 장치 설정을 진행합니다.
`git clone` 자체가 드라이버나 실행용 모델을 설치하지는 않습니다.

## 모델 배치

- 학습·평가·변환 작업은 최상위 `models/`에서 관리합니다.
- 검증한 실행용 HEF와 해당 class ID 순서의 라벨은 `onboard/models/deployed/`에 함께 배치합니다.
- `onboard/config/runtime.env`의 두 경로는 `onboard/` 기준입니다.

```text
onboard/models/deployed/model.hef
onboard/models/deployed/labels.txt
```

실제 HEF는 이 소스 패키지에 포함되지 않습니다. [실행용 모델 안내](onboard/models/README.md)를 참고하세요.
각 Pi의 실제 설정, 배포 모델 세트, `.venv-rpi/`, `build/`와 로그는 로컬에서 유지하고 Git에서 제외합니다.

## 코드 업데이트

선택한 브랜치를 추적하는 Pi에서 실행 프로세스를 종료한 뒤:

```bash
cd ~/argus-edge-vision
git pull --ff-only
cd onboard
bash scripts/setup_rpi.sh
bash scripts/verify_rpi.sh
bash scripts/run_rpi.sh
```

자동 실행 서비스를 사용 중인 경우 서비스 종료·재시작 방법은
[설치·실행 안내](onboard/docs/INSTALL_AND_RUN_RPI5_HAILO8.md)를 따릅니다.
코드 태그와 함께 HEF·라벨의 버전 및 SHA256, HailoRT·SDK 버전을 기록해 검증한 환경을 재현합니다.
