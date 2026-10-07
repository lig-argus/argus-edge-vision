# 설치·실행·검증

실행 대상은 이 저장소의 `onboard/` 패키지다. 아래 예시는 홈 디렉터리에 복제한
`~/argus-edge-vision/onboard`를 사용하며, 설치 위치가 달라도 스크립트는 자신의 위치에서 패키지 경로를 계산한다.
실제 runtime.env·HEF·배포 라벨·가상환경·빌드 산출물은 각 Pi에서 준비하고 Git에 포함하지 않는다.
C++ 실행 파일은 `build/bin/argus-onboard`이며 `argus-vision` CLI도 이를 실행한다.

## Pi에서 소스 받기

```bash
cd ~
git clone --filter=blob:none --sparse -b develop \
  https://github.com/s-jje/argus-edge-vision.git
cd argus-edge-vision
git sparse-checkout set --cone onboard
cd onboard
```

테스트 Pi는 `develop`을 사용한다. 드론 장착 Pi는 `main` 또는 검증한 릴리즈 태그를 선택한다.
cone 모드는 `onboard/`와 루트 파일들을 포함하며 다른 최상위 모듈은 필요하지 않다.
이후 명령은 모두 `onboard/`에서 실행한다.

## 준비와 빌드

기존 HailoRT/PyHailoRT, SDK, HEF와 Python 환경을 재사용한다.
C++17 컴파일러, CMake, Threads, ZeroMQ/cppzmq(`zmq.hpp`), MessagePack,
toml++, OpenCV 개발 파일이 필요하다. Debian/Ubuntu 계열의 개발 의존성 설치 예시:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake pkg-config python3-venv python3-dev \
  libzmq3-dev cppzmq-dev libmsgpack-cxx-dev libtomlplusplus-dev \
  libopencv-core-dev libopencv-imgproc-dev libopencv-imgcodecs-dev libopencv-videoio-dev
cd ~/argus-edge-vision/onboard
python3 -m venv --system-site-packages .venv-rpi
.venv-rpi/bin/python -m pip install -r requirements.txt
bash scripts/setup_rpi.sh
```

`setup_rpi.sh`는 필요하면 system-site-packages 가상환경을 만들고 Python 패키지를
editable로 설치한 뒤 C++ 빌드/CTest를 실행한다. Python 의존성을 자동 설치하지 않으므로
[pyproject.toml](../pyproject.toml)의 의존성과 기존 Hailo 환경이 준비돼 있어야 한다.
위 명령의 venv 생성은 기존 `.venv-rpi`가 없는 최초 설치 때 수행한다.
이미 준비된 Hailo Python 환경에서는 의존성을 확인한 뒤 기존 가상환경을 재사용한다.
기존 HailoRT/PyHailoRT와 SDK를 임의로 교체하지 않는다.

기존 `config/runtime.env`는 유지한다. 새 환경은
[runtime.env.example](../config/runtime.env.example)을 참고해 실제 HEF·라벨·입력 경로를 설정한다.
예시 설정 파일은 HEF·라벨을 모두 `models/deployed/`에서 읽는다.
검증한 모델 세트를 [실행용 모델 안내](../models/README.md)에 따라 배치한 뒤,
기존 설정이 없는 새 Pi에서만 다음을 실행한다.

```bash
cp config/runtime.env.example config/runtime.env
nano config/runtime.env
```

카메라·SDK 경로는 각 Pi의 실제 장치에 맞춘다. 상대 경로는 저장소 루트가 아닌 `onboard/` 기준이다.
`ARGUS_FRAME_SOURCE=auto`는 프로파일을 따르며 현재 REAL_IR은 V4L2다.
SDK 사용은 `ARGUS_FRAME_SOURCE=ir_sdk`를 명시한다.
환경 파일은 스크립트에서 읽으므로 입력 설정도 그 파일에서 확인한다.

## 점검과 실행

```bash
bash scripts/service_entrypoint.sh check
bash scripts/verify_rpi.sh
bash scripts/run_rpi.sh
```

- `check`: 프로파일·설정 검사. 스크립트는 HEF/라벨 파일 존재를 요구하지만 카메라·추론은 시작하지 않는다.
- `verify_rpi.sh`: Python 모듈, 장치/파일, HEF 입력 메타데이터의 정적 검사다. 실제 스트림·추론이나 라벨 수 일치까지 검증했다고 보지 않는다.
- `run_rpi.sh`: BusProxy와 C++ 실행부를 시작한다. Ctrl+C로 종료한다.

SDK 기본 경로는 `/home/user/thessen_raw14_viewer/build/i3_raw14_capture`다.
입력·모델 조건은 [CAMERA_AND_MODEL_NOTES](CAMERA_AND_MODEL_NOTES.md)를 따른다.

## Pi JPEG 미리보기

Pi에서 같은 프레임의 검출 박스를 그린 JPEG를 발행한다.

```bash
bash scripts/run_rpi.sh models/deployed/model.hef /dev/video0 models/deployed/labels.txt --preview
```

같은 Wi-Fi의 PC/Windows에서 구독 도구가 설치된 환경으로 실행한다.

```bash
argus-preview-sub --endpoint tcp://<Pi-IP>:5556
```

기존 설정을 유지하며 미리보기만 켜려면 runtime.env의 `ARGUS_PUBLISH_PREVIEW=1`을 사용한다.
`vision/npu_input/jpeg`는 별도 디버그 옵션이며 [버스 계약](BUS_CONTRACT.md)을 참고한다.

## 검증

```bash
bash scripts/build.sh
.venv-rpi/bin/python -m pytest tests/integration -q
```

C++ 계약 검사와 합성 Replay, 명시적 Mock Detector, SDK `--test-pattern`,
ZeroMQ 전달·JPEG/프레임 ID·worker 실패/지연·종료 정리를 검사한다.
SDK 검사에는 기존 캡처 실행 파일이 필요하다. 선택적 실제 Hailo 검사는
`ARGUS_HEF_SMOKE`에 HEF 경로를 지정하며 합성 Replay로 수행한다.
실제 카메라·FC·비행 검증을 대신하지 않는다. 이전 결과는 [VALIDATION](VALIDATION.md)에 있다.

자동 시작은 `scripts/install_autostart.sh`와 `deploy/systemd/`의 별도 운용 작업이다.
2026-09-26 전환 검증에서는 서비스 문법만 확인했고 설치·기동하지 않았다.
현재 실행부는 `perception_only`이며 기체 명령을 송신하지 않는다.

## 코드 업데이트

브랜치를 추적하는 Pi에서 실행 프로세스를 종료한 뒤 다음을 실행한다.
릴리즈 태그를 선택한 Pi는 새로 검증한 태그를 명시적으로 선택한다.

```bash
cd ~/argus-edge-vision
git pull --ff-only
cd onboard
bash scripts/setup_rpi.sh
bash scripts/verify_rpi.sh
```

수동 실행은 `bash scripts/run_rpi.sh`를 사용한다.
자동 실행 서비스를 이미 설치했다면 업데이트 전 `sudo systemctl stop argus-vision.service argus-busd.service`,
성공한 점검 후 `sudo systemctl start argus-busd.service argus-vision.service`를 사용한다.
기존 runtime.env와 배포 모델 세트는 유지한다. 새 모델은 대응 라벨과 함께 검증하고 적용한다.
