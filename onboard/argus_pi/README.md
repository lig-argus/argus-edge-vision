# ARGUS Pi 구현 (onboard/argus_pi)

2026-11-26 반입. Raspberry Pi 에서 실기에 쓰던 ARGUS 전체 구현(탐지, UCMC 추적, 추종 제어,
FC 게이트웨이, 영상 송신, 진단 기록)을 팀 메시지 규약에 맞춘 상태로 옮겼다.

## 팀 규약과의 관계
- 버스 메시지는 팀 protobuf `argus.msg` 와 `[topic][protobuf][binary]` 프레임, 팀 ZmqBus,
  포트 6370 부터 6375 를 따른다. 팀 proto 의 필드는 바꾸지 않았고 Pi 전용 값은 번호 100
  이상의 `pi_` 필드와 Pi 전용 proto 로 붙였다. 자세한 규칙은 `proto/README.md`.
- 팀 라이브러리 사본은 `third_party/argus_onboard` (팀 06d7f19 기준).
- 이 폴더는 `onboard/CMakeLists.txt` 에 포함하지 않는다. 빌드와 시험은 이 폴더의 스크립트와
  `cmake/` 를 쓴다 (예: `python3 build_runtime.py --host-check`).
- 내부 구조(`src/argus`, nlohmann, msgpack, cassert 시험)는 팀 모듈 규칙과 다르다. 팀 앱
  (tracker, follow_controller 등)으로 옮기는 일은 후속 작업이다.

## 팀 CI 와의 관계
- C++ 는 팀 `.clang-format` 으로 정리했다. `vendor/` 와 DDS 생성 코드는 원본 형식을 유지한다.
- Python 은 `ruff format` 으로 정리했다. 규칙 검사는 이 폴더의 `ruff.toml` (문법 오류와
  정의되지 않은 이름) 만 적용하고, mypy strict 는 파일마다 `# mypy: ignore-errors` 로 제외했다.
  팀 규칙 전체 적용은 후속 작업이다.

## 저장소에 넣지 않은 것
- IR 카메라 SDK `vendor/camera/sdk_2_12_2` (제조사 라이브러리, 공개 배포 가능 여부 미확인).
  Pi 에서는 같은 경로에 두고 빌드한다 (`cmake/camera_i3.cmake` 의 `I3_SDK_DIR` 로 바꿀 수 있다).
- 실행 기록(`run/`)과 진단 출력(`gui-guidance.json`).

## 검증 범위
WSL 과 Raspberry Pi 의 합성 입력 시험(빌드, protobuf, team_wire, gateway, follow_b, FC 없는
카메라 전용 기능)만 수행했다. 실제 카메라, NPU, FC, 비행 시험은 이 반입 상태로 하지 않았다.
