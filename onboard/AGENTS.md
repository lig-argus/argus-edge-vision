# AI 작업 규칙 — C++ 중심 구조 (사용자 승인, 2026-09-26)

작업 시작 전에 README.md, docs/CPP_MIGRATION.md, docs/FLOW_ALIGNMENT.md를 읽는다.
설계 원본은 docs/design/flow.mmd와 docs/design/whole-flowchart.mmd다.
사용자의 최신 명시적 지시가 이 문서보다 우선한다.

## 확정한 책임

- 실행 진입점은 build/bin/argus-onboard (C++ OnboardMain → CompositionRoot → PerceptionWorker)다.
- 카메라 I/O, 인지 큐, 공개 Observation 구성/검증/발행, 프로세스 생명주기는 C++가 소유한다.
- Python 유지 범위: RAW14/letterbox 전처리, YOLOX/Hailo, Pi에서 박스 그리기/JPEG, BusProxy, 점검·구독 도구.
- Wi-Fi 미리보기는 Pi에서 영상과 bbox를 합친 JPEG다. PC 합성 방식으로 바꾸지 않는다.
- PythonVisionWorker는 C++ 요청을 실행할 뿐이다. 카메라·큐·제어·공개 PUB를 새로 넣지 않는다.
- SDK 캡처는 기존 /home/user/thessen_raw14_viewer/build/i3_raw14_capture를 별도 프로세스로 재사용한다.
- SDK와 모델을 임의로 다시 작성하거나 교체하지 않는다.

## 자동 원복 금지

- legacy/python/**/*.py.org는 당시 Python 원본이다. 실행·import·패키징 대상이 아니다.
- *.py.org를 *.py로 되돌리거나 src/argus_vision/vision_node.py를 다시 만들지 않는다.
- C++ 빌드/실행 실패 시 Python 전체 실행부로 fallback하지 않는다. 실패 원인을 수정한다.
- 설치·서비스·CLI가 과거 argus_vision.vision_node로 돌아가지 않도록 한다.
- 과거 teammate, 출력 스냅샷, legacy 문서는 현행 구현의 근거로 사용하지 않는다.
- 사용자 명시적 요청 없이 C++ 중심 결정 자체를 원복하지 않는다. 일반 버그 수정은 진행한다.

## 흐름도 불변 조건

- 인지는 기체 명령을 생성하지 않는다. 프레임 루프와 제어/Watchdog은 독립이어야 한다.
- 기존 인지 큐: 기본 용량 2, FIFO, 가득 차면 oldest 폐기. latest-state 제어 저장소와 혼동하지 않는다.
- 전체 박스(full_box)만 사용한다. hull ROI를 다시 추가하지 않는다.
- 실제 센서 노출 시각이 없으면 유효하다고 주장하지 않는다. 수신 시각으로 대체하지 않는다.
- Prediction이나 발행으로 ID별 마지막 실제 검출 시각/TTL을 연장하지 않는다.
- 네 tracker 후보는 별도 구현으로 등록한다. ReID 없음, 상세 flow의 FC 회전 CMC 조건을 따른다.
- 미구현 Tracker/CMC/FC/control을 Mock 또는 시험용 AlphaBeta로 몰래 대체하지 않는다.
- 모든 기체 명령은 최종 Guard 및 단일 VehiclePort를 거쳐야 한다.
- 현재 이 패키지는 perception_only다. 미구현 제어 기능이 작동한다고 표시하지 않는다.

## 미구현 표시 파일 — 2026-09-26 사용자 승인

사용자는 전체 흐름도 역할별 빈 C++ 파일 배치를 승인했다. **기능 구현 승인은 아니다.**
목록과 설명은 [미구현 파일 지도](docs/UNIMPLEMENTED_FILES.md)에 있다.

- `UNIMPLEMENTED_PLACEHOLDER` 파일은 `/*미구현 임의 생성 금지*/`와 설명 주석만 갖는다.
- 헤더도 타입·함수를 선언하지 않는다. 파일 존재를 구현 완료나 사용 가능한 인터페이스로 간주하지 않는다.
- 이 표시 파일에 사용자 명시적 승인 없이 알고리즘·API·수치·기본 반환·Mock을 작성하지 않는다.
- CMake, CompositionRoot 팩토리, 실행 스크립트, 버스 발행·구독 경로에 자동 연결하지 않는다.
- 기존 구현/인터페이스/자료형을 빈 표시 파일로 덮어쓰지 않는다. `.py.org` 보관 정책도 유지한다.
- 일반적인 기존 구현 버그 수정은 가능하다. 미구현 기능 작성은 이후 사용자의 명시적 지시 범위에서 진행한다.

## 검증

bash scripts/build.sh 후 .venv-rpi/bin/python -m pytest tests/integration 를 실행한다.
실제 카메라/FC가 없는 조건은 합성 Replay 또는 명시적인 SDK --test-pattern으로 검증한다.
원본 보존은 legacy/python/manifest.json으로 확인한다. 실행 코드는 cpp/와 python/argus_workers/다.
