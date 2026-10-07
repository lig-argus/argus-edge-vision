# 과거 원본·문서 — 실행 지침 아님

현행 안내는 [README](../README.md), 작업 규칙은 [AGENTS](../AGENTS.md)를 따른다.
과거 자료를 근거로 Python 전체 실행부를 복원하거나 현재 기능·성능을 판단하지 않는다.

- `python/`: 당시 Python 원본 31개를 `*.py.org`로 보관. 실행·import·패키징 금지이며 [manifest](python/manifest.json)으로 SHA256을 확인한다.
- `launchers/`, `documentation/*.org`, `documentation/docs/*.org`: 이전 실행 설정·문서 원본. 자동 복원하지 않는다.
- `documentation/before-placeholders-20260926-215910.tar.gz`: 미구현 표시 파일 배치 전 문서/해시 기록.
- `documentation/history-20260926/`: 이번 정리에서 옮긴 과거 기록·Pi 상위 문서 사본. 원래 바이트를 보존했고 확장자만 `.md.org`로 바꿨다.

## 과거 기록 위치

| 기록 | 보관 원문 |
|---|---|
| Python 구현 진행 현황 | [PROGRESS_SUMMARY](documentation/history-20260926/docs/PROGRESS_SUMMARY.md.org) |
| 당시 성능 조사 | [PERF_NOTES](documentation/history-20260926/docs/PERF_NOTES.md.org) |
| C++ 전환 전 지연 측정 | [LATENCY_REPORT](documentation/history-20260926/docs/LATENCY_REPORT.md.org) |
| 이전 모델 교체 체크리스트 | [TRAINING_DAY_CHECKLIST](documentation/history-20260926/docs/TRAINING_DAY_CHECKLIST.md.org) |
| 이전 시스템 가이드 v1.0 | [Markdown 원문](documentation/history-20260926/docs/ARGUS_RPi5_Hailo8_YOLOX_시스템_이해_및_구현_가이드_v1.0.md.org) · [기존 DOCX 사본](../docs/ARGUS_RPi5_Hailo8_YOLOX_시스템_이해_및_구현_가이드_v1.0.docx) |
| Pi /home/user/README.md 사본 | [home-README](documentation/history-20260926/docs/pi-layout/home-README.md.org) |
| Pi /home/user/npu-test/README.md 사본 | [npu-README](documentation/history-20260926/docs/pi-layout/npu-README.md.org) |
| Pi /home/user/npu-test/AGENTS.md 사본 | [npu-AGENTS](documentation/history-20260926/docs/pi-layout/npu-AGENTS.md.org) |

원문 내부의 링크·경로·명령은 **기록 당시 위치 기준**이다. Pi 사본을 옮긴 것은 이 Windows 폴더 안의 정리이며 Pi 원격 파일을 변경한 것이 아니다.
Python 시대의 측정값은 현행 C++ 구조의 재측정값이 아니다. 기존 DOCX도 당시 가이드다.
현재 모델 점검은 [CAMERA_AND_MODEL_NOTES](../docs/CAMERA_AND_MODEL_NOTES.md),
실행은 [INSTALL_AND_RUN_RPI5_HAILO8](../docs/INSTALL_AND_RUN_RPI5_HAILO8.md),
전환·표시 파일 배치 검증은 [VALIDATION](../docs/VALIDATION.md)을 사용한다.
