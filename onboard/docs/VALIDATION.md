# Pi 전환·표시 파일 배치 검증 — 2026-09-26

아래는 **당시 Pi에서 수행한 검증 기록**이다. 문서 정리 후 재실행한 결과가 아니다.
현행 Pi 경로는 `/home/user/npu-test/work/argus_vision_bus`다.
`teammate/`와 [legacy](../legacy/README.md)는 현행 구현의 근거가 아니다.

## C++ 전환

환경: Raspberry Pi 5 ARM64, g++ 13.3, HailoRT 4.23, Hailo-8.

| 대상 | C++ 빌드·CTest | Python 통합 검사 |
|---|---|---|
| 배포 후보 | C++17 빌드 성공, flow-contracts 1/1 | 14 passed (19.27s), 실제 Hailo smoke 포함 |
| Pi 실제 배포 경로 | 빌드 성공, flow-contracts 1/1 | 14 passed (19.29s), 실제 Hailo smoke 포함 |

- 기존 COCO80 YOLOX HEF와 합성 Replay로 실제 Hailo 추론, 기존 SDK 실행 파일의 `--test-pattern`으로 입력을 검사했다.
- FIFO/drop-oldest, 노출 시각 계약, ID별 마지막 실제 검출 TTL·prediction 불갱신, 공개 v2 23필드·full_box·null track_id를 확인했다.
- C++ Replay/SDK → Python 전처리/추론 → C++ 관측 발행 → BusProxy 구독 전달을 확인했다.
- Pi JPEG의 같은 frame_id와 실제 박스 픽셀을 확인했다.
- 잘못된 frame_id, 지연 worker, 잘린 SDK 스트림은 fault로 끝났으며 Python 전체 실행부 fallback이 없었다.
- SIGTERM 시 소유 자식 프로세스·임시 IPC와 프록시를 정리했다.
- Python 원본 31개의 SHA256을 보존했다. 패키지는 0.3.0이며 옛 argus_vision namespace는 import되지 않았다.
- 설치된 argus-vision CLI와 service_entrypoint.sh check는 C++ 진입점을 사용했다.
- systemd WorkingDirectory 인용부호 오류 수정 후 문법 검사는 exit 0이었다. 종료 제한은 75초로 조정했다.
- 서비스 템플릿 수정 후 원본 보관·현행 실행 경로 검사를 다시 수행했다(1 passed). 서비스는 설치·기동하지 않았으며 검사 프로세스는 종료했다.

## 미구현 표시 파일 배치

승인 범위는 **30개 역할의 주석 전용 파일 54개와 안내 문서**였다. 기능 구현은 추가하지 않았다.

- 헤더만 6개 + 헤더/소스 쌍 24개. 모든 파일은 `/*미구현 임의 생성 금지*/`와 설명 주석뿐이며 설계 노드 ID가 원본에 존재함을 확인했다.
- 기존 CMake·CompositionRoot·실행 스크립트·계약 등을 포함한 141개 파일의 SHA256이 같았다.
- C++ 빌드 성공, CTest **1/1**, 통합 검사 **13 passed, 1 skipped**. 선택적 실제 Hailo smoke는 이 작업에서 실행하지 않았다.
- argus-onboard와 argus-frame-source는 변경 전과 바이트까지 동일했다. 새 파일은 빌드·팩토리·실행 경로에 연결하지 않았다.
- check 결과는 계속 `perception_only`였으며 runtime.env·profiles.toml·HEF와 Python 보관 원본 31개의 해시를 보존했다.

## 백업과 원시 기록

Pi 백업 루트: `/home/user/npu-test/backups/`.

| 작업 | 백업 루트 아래 경로 | 이 사본의 원시 결과 |
|---|---|---|
| C++ 전환 | `argus-before-cpp-20260926-213502/before-source.tar.gz`, `retired/`, `validation-results.json` | [validation-results.json](validation-results.json) |
| 표시 파일 배치 | `argus-before-placeholders-20260926-215910/before-docs.tar.gz`, `validation-results.json` | [scaffold-validation-results.json](scaffold-validation-results.json) |

전환 전 src/tests는 위 전환 백업의 retired/에 .py.org로 보관했다.
[배포 메타데이터](deployment-metadata.json), [표시 파일 배치 메타데이터](scaffold-deployment-metadata.json),
[배포 당시 소스 해시](deployed-source-sha256.json), [전환 전 소스 해시](original-source-sha256.json),
[미구현 파일 목록·해시](unimplemented-files.json)는 당시 기록이다.
문서 정리로 이 JSON을 갱신하지 않았으며 현재 Markdown의 해시 목록으로 해석하지 않는다.

전환 당시 유지한 파일:

| 파일 | SHA256 |
|---|---|
| config/runtime.env | `c1d9113c881e80a2c8569ea038dbff06b2e82e78c74f6ed24cab573659bcabaa` |
| config/profiles.toml | `8f85cecac6e1f2e670f3d2626c13204fa23351fbf982ea770cde831916352015` |
| models/yolox_s_leaky_hailo8_zoo_v2.19.0.hef | `f51c6c2c6cd1bd73b9171858809177b39230ef98e6ff2de7306d313b013cf9be` |

## 검증하지 않은 범위

실제 USB/V4L2 카메라 스트림·실제 노출 시각·PX4/FC 연결·기체 명령 송신은 검증하지 않았다.
실제 MOT, FC 회전 CMC, 제어 상태 저장소/FSM/Guard/Watchdog/VehiclePort는 미구현이다.
합성 검사는 실제 장면의 정확도나 FPS 향상을 보장하지 않으며 YOLOX 시간은 기존 Hailo/Python 경로의 영향을 받는다.

재현 명령은 [설치·실행·검증](INSTALL_AND_RUN_RPI5_HAILO8.md)을 따른다.
