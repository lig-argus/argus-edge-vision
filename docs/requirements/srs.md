# SRS — 소프트웨어 요구사항명세서

| 항목 | 내용 |
|---|---|
| 문서 버전 | 0.2 |
| 체계 | ARGUS 온보드 AI 드론 객체 탐지·추적 체계 |
| 작성 | ARGUS 팀 (LIG D&A The SSEN 임베디드 SW 스쿨 4기) |
| 상위 문서 | `docs/requirements/sss.md` (SSS), `docs/design/ssdd.md` (SSDD) |
| 관련 문서 | `docs/design/sdd.md` (SDD), `docs/spec/icd.md` (ICD), `docs/verification/std.md` (STD), `docs/verification/str.md` (STR) |

## 1. 범위

### 1.1 체계 개요

ARGUS는 소형 드론에 NPU 임베디드 보드를 탑재하여, 지휘부와 통신이 끊긴 상황에서도 온보드에서 지상 표적(전차)을 실시간 탐지·추적하고 대상을 자율 추종하는 체계이다. 프로젝트 계획서 대비 변경 사항은 7장에 기록한다. 본 문서의 상위 문서는 체계/부체계 규격서(SSS)와 체계/부체계 설계기술서(SSDD)이다.

### 1.2 CSCI 식별

| CSCI ID | 명칭 | 실행 위치 |
|---|---|---|
| CSCI-VIS | 탐지 (카메라 입력, 전처리, NPU 추론) | Raspberry Pi 5 + Hailo-8 |
| CSCI-TRK | UCMC 추적기 (객체 연결, 카메라 운동 보상) | Raspberry Pi 5 |
| CSCI-FOL | 추종 제어 (추종 판단, setpoint 계산, 임무·표적 상태) | Raspberry Pi 5 |
| CSCI-GW | mavlink_gateway (FC 통신, 통제기 통신, FC 명령 검사) | Raspberry Pi 5 |
| CSCI-VTX | 영상 송신부 (RTSP 영상·메타데이터 송신) | Raspberry Pi 5 |
| CSCI-GCS | 통제기 SW (영상·상태 표시, 운용자 명령) | Windows 노트북 |

CSCI는 SSDD 4.1절의 체계 구성요소 할당에 따라 식별하였다. 요구사항 표의 CSCI 열은 접두어 `CSCI-` 를 생략한다.

## 2. 참조문서

- 프로젝트 계획서
- 방위사업청, 「무기체계 소프트웨어 개발 및 관리 매뉴얼」 별책 「무기체계 소프트웨어 기술문서 작성가이드」, 2020
- `docs/requirements/sss.md`, `docs/design/ssdd.md` (상위 문서), `docs/design/sdd.md`, `docs/spec/icd.md`, `docs/spec/process_interfaces.md`, `docs/spec/video.md`, `docs/verification/std.md`, `docs/verification/str.md`
- PX4 Autopilot, MAVLink, HailoRT, GStreamer 공식 문서

## 3. 운용개념

- 운용자는 통제기에서 RTSP 영상과 추적 박스를 보고 추종 대상 1개를 지정한다.
- 탐지는 영상을 NPU로 추론하고, 추적기가 대상 ID를 유지하여 추적 결과를 발행한다.
- 추종 제어는 지정 대상의 영상 내 위치 오차로 추종 명령을 만들고, mavlink_gateway는 FC 명령 검사를 통과한 명령만 MAVLink로 FC(PX4)에 보낸다.
- 대상 소실·통신 단절·RC 오버라이드 시 추종을 멈추고 정지(Hold) 또는 조종자 제어로 넘긴다. 통제기 링크가 끊겨도 탐지·추적은 온보드에서 계속된다.
- 개발·시험은 환경 프로파일(SITL_POSE / SITL_IMAGE / REAL_IR / REPLAY)로 구분하며, 실기체 비행은 시뮬레이션·보드 통합 시험이 완료된 뒤 실내 저속·저고도에서만 수행한다.

## 4. 요구사항

검증 방법: T(시험) · D(입증) · A(분석) · I(검사).

### 4.1 기능 요구사항

| ID | 요구사항 | CSCI | 검증 |
|---|---|---|---|
| SRS-FUN-01 | SW는 영상 입력원(IR 카메라, USB 웹캠, Gazebo 카메라, 기록 재생)을 환경 프로파일 설정만으로 교체할 수 있어야 한다. | VIS | D |
| SRS-FUN-02 | SW는 추론 대기 프레임이 쌓이면 가장 오래된 프레임을 버려 최신 프레임을 처리해야 한다. | VIS | T |
| SRS-FUN-03 | SW는 온보드 NPU에서 전차 클래스를 탐지하고 바운딩박스·신뢰도·획득 시각을 출력해야 한다. | VIS | T |
| SRS-FUN-04 | SW는 탐지 결과에 추적기를 적용하여 트랙 ID와 상태(CONFIRMED / PREDICTED)를 유지해야 한다. | TRK | T |
| SRS-FUN-05 | SW는 드론 자세(FC RPY)를 이용해 카메라 운동을 보상한 뒤 추적해야 한다. | TRK | T |
| SRS-FUN-06 | SW는 추적 결과를 `tracking/tracks` 토픽으로 track_id·generation·상태·프레임 시각·source 와 함께 발행해야 한다 (`process_interfaces.md` 5.1.1). | TRK | I |
| SRS-FUN-07 | SW는 운용자가 지정한 대상 1개만 추종하며, 대상이 겹쳐도 다른 ID로 바꾸지 않아야 한다. | FOL, GW | T |
| SRS-FUN-08 | SW는 대상 박스 중심과 영상 중심의 오차를 기체 상대 기하로 변환하여 속도 setpoint를 생성해야 한다. | FOL | T |
| SRS-FUN-09 | SW는 임무 상태(`ARGUS_MISSION_STATE` 8개, `state_machine.md`)를 관리하고 FOLLOW_ACTIVE 상태에서만 추종 setpoint를 내야 한다. | FOL | T |
| SRS-FUN-10 | SW는 임무 상태·표적 상태를 `follow/status` 토픽으로 발행하고, mavlink_gateway는 이를 `ARGUS_STATUS` 로 통제기에 보고해야 한다. | FOL, GW | I |
| SRS-FUN-11 | 통제기는 RTSP 로 받은 영상과 메타데이터 트랙의 선택/비선택 박스, 임무 상태를 표시하고, 대상 선택·추종 시작·중단 명령(TRACK_START, TRACK_STOP)을 보낼 수 있어야 한다. | GCS, VTX | D |
| SRS-FUN-12 | SW는 프레임·자세·명령 시각(t_frame, t_att, t_cmd)과 트랙·상태를 비동기로 기록해야 한다. | 온보드 전체 | I |

### 4.2 성능 요구사항

| ID | 요구사항 | 검증 |
|---|---|---|
| SRS-PRF-01 | NPU 추론 처리율은 20 FPS 이상이어야 한다. | T |
| SRS-PRF-02 | 프레임 획득부터 FC 명령 송신까지 E2E 지연은 p95 기준 150 ms 이하여야 한다. | T |
| SRS-PRF-03 | 시험 시나리오에서 지정 대상 추적 유지율은 90% 이상이어야 한다. | T |
| SRS-PRF-04 | 전차 탐지 정확도는 시험 데이터셋에서 mAP@0.5 0.80 이상이어야 하며, 차량 모형을 전차로 오탐하는 비율은 5% 이하여야 한다. | T |
| SRS-PRF-05 | 온보드 보드(RPi5 + NPU) 소비전력은 12 W 이하여야 한다. | T |

### 4.3 인터페이스 요구사항

| ID | 요구사항 | 검증 |
|---|---|---|
| SRS-IF-01 | 보드와 FC는 USB로 연결하고, mavlink-router 를 경유하는 UDP로 MAVLink 텔레메트리를 받고 비행 명령을 보내야 한다. MAVLink 메시지는 생성 C 헤더로 인코드·디코드한다 (`icd.md` 1장). | T |
| SRS-IF-02 | NPU는 PCIe Gen3로 호스트와 연결되고 HailoRT로 HEF 모델을 실행해야 한다. | I |
| SRS-IF-03 | 온보드 프로세스는 ZeroMQ 토픽(발행·구독)과 요청-응답(`process_interfaces.md` 3장)으로만 데이터를 교환해야 한다. | I |
| SRS-IF-04 | 통제기 명령은 명령 번호(param7)를 가지며, 같은 번호의 재전송은 한 번만 처리하고 처리 결과(`COMMAND_ACK`)를 회신해야 한다 (`icd.md` 4.1). | T |

### 4.4 안전 요구사항

| ID | 요구사항 | 검증 |
|---|---|---|
| SRS-SAF-01 | 지정 대상이 소실되어 예측 유지 기한이 만료되면(표적 상태 LOST) 추종을 멈추고 정지(Hold)해야 하며, 재탐지 후에도 운용자 재지정 없이 다른 ID를 추종하지 않아야 한다. | T |
| SRS-SAF-02 | RC 조종기 오버라이드가 발생하면 SW 명령 송신을 즉시 중단해야 한다. | T |
| SRS-SAF-03 | 유효시간이 지난(stale) 트랙·명령은 FC로 보내지 않아야 한다. | T |
| SRS-SAF-04 | 추종 명령은 설정된 가속도·속도·고도 한도를 넘지 않아야 한다. | T |
| SRS-SAF-05 | FC 명령 송신자는 mavlink_gateway 하나여야 한다. | I |

### 4.5 환경·제약 요구사항

| ID | 요구사항 | 검증 |
|---|---|---|
| SRS-ENV-01 | 온보드는 Raspberry Pi 5 + Hailo-8(AI HAT+ 26 TOPS), Ubuntu 24.04 에서 동작해야 한다. | I |
| SRS-ENV-02 | 온보드 전원은 드론 메인 배터리(3S)와 분리해야 한다. | I |
| SRS-ENV-03 | 탑재물 중량은 500 g 이내여야 한다. | A |
| SRS-ENV-04 | FC는 PX4 펌웨어를 사용하며, 실내에서는 GPS 없이 옵티컬플로우·거리센서로 위치를 유지해야 한다. | T |
| SRS-ENV-05 | 온보드 코드는 C++로 작성하고, AI·영상 전처리만 Python을 허용한다. | I |

## 5. 자격요건

요구사항 31건의 검증 방법 분포는 시험(T) 19건, 검사(I) 9건, 입증(D) 2건, 분석(A) 1건이다. 시험·입증은 STD의 시험 케이스로, 검사·분석은 설계 검토와 환경 점검(TC-17)으로 수행한다.

## 6. 요구사항 추적표

상위 추적(SSS → SRS)은 SSS 5장을 따른다. 아래 표는 SRS → SDD → STD 하위 추적이다. CSC 식별자는 SDD 3장을 따른다.

| SRS ID | 구현 CSC (SDD) | 시험 케이스 (STD) |
|---|---|---|
| SRS-FUN-01 | CSC-VIS-01, CSC-COM-03 | TC-01 |
| SRS-FUN-02 | CSC-VIS-02 | TC-02 |
| SRS-FUN-03 | CSC-VIS-03, CSC-VIS-04, CSC-VIS-05 | TC-03 |
| SRS-FUN-04 | CSC-TRK-03 | TC-06 |
| SRS-FUN-05 | CSC-TRK-02, CSC-TRK-03 | TC-07 |
| SRS-FUN-06 | CSC-TRK-04 | TC-08 |
| SRS-FUN-07 | CSC-FOL-02, CSC-GW-05 | TC-06 |
| SRS-FUN-08 | CSC-FOL-02, CSC-FOL-03 | TC-15, TC-18 |
| SRS-FUN-09 | CSC-FOL-02 | TC-12 |
| SRS-FUN-10 | CSC-FOL-05, CSC-GW-06 | TC-08 |
| SRS-FUN-11 | CSC-GCS-02~05, CSC-VTX-01~03 | TC-11 |
| SRS-FUN-12 | CSC-COM-01, CSC-COM-02 | TC-09 |
| SRS-PRF-01 | CSC-VIS-04 | TC-04 |
| SRS-PRF-02 | 전체 파이프라인 | TC-09 |
| SRS-PRF-03 | CSC-TRK-03, CSC-FOL-02 | TC-06 |
| SRS-PRF-04 | CSC-VIS-04 | TC-05 |
| SRS-PRF-05 | (HW) | TC-16 |
| SRS-IF-01 | CSC-GW-01, CSC-GW-03 | TC-10 |
| SRS-IF-02 | CSC-VIS-04 | TC-03 |
| SRS-IF-03 | CSC-VIS-05, CSC-TRK-04, CSC-FOL-05 | TC-08 |
| SRS-IF-04 | CSC-GCS-01, CSC-GW-05 | TC-11 |
| SRS-SAF-01 | CSC-FOL-02 | TC-12 |
| SRS-SAF-02 | CSC-GW-02, CSC-GW-04 | TC-13, TC-18 |
| SRS-SAF-03 | CSC-GW-02, CSC-GW-03 | TC-14 |
| SRS-SAF-04 | CSC-FOL-02 | TC-14 |
| SRS-SAF-05 | CSC-GW-02 | TC-10 |
| SRS-ENV-01 | (플랫폼) | TC-17 |
| SRS-ENV-02 | (HW) | TC-16 |
| SRS-ENV-03 | (HW) | TC-16 |
| SRS-ENV-04 | (FC 설정) | TC-15 |
| SRS-ENV-05 | (개발 환경) | TC-17 |

## 7. 변경이력 (프로젝트 계획서 대비)

| 번호 | 항목 | 계획서 | 변경 내용 | 사유 | 영향 요구사항 |
|---|---|---|---|---|---|
| CHG-01 | NPU | DeepX DX-M1 | Hailo-8 (AI HAT+ 26 TOPS), HailoRT | 프로젝트 예산 부족, 멘토 보유 Hailo 보드 지원 | FUN-03, IF-02, ENV-01 |
| CHG-02 | 모델 변환 | ONNX → DeepX 컴파일 | ONNX → Hailo Dataflow Compiler → HEF | CHG-01 따름 | IF-02 |
| CHG-03 | 기체 | BP5 5인치 드론 | F450 픽스호크 풀셋 | 탑재 여유·교육용 기체 안정성 | ENV-03 |
| CHG-04 | FC | MicoAir H743 V2 | KoA FC (STM32H743), PX4 | 기체 풀셋 구성품 | ENV-04 |
| CHG-05 | FC 연결 | UART/USB | USB 단일 (mavlink-router 경유) | 탑재 보드 UART 핀 반복 손상 | IF-01 |
| CHG-06 | 영상 입력 | USB IIR 카메라 | 웹캠(통합 시험), 대형 IR 카메라(데이터셋) | 탑재용 IR 카메라 고장·수리 지연 | FUN-01 |
| CHG-07 | 프로세스 통신 | 미정 | ZeroMQ 토픽·요청-응답, protobuf 직렬화 | ROS 미사용 결정, 경량 구조 | IF-03 |
| CHG-08 | 구현 언어 | C/C++, Python, PyQt | C++ 중심, Python은 AI·전처리만 | 협업상 언어 통일 | ENV-05 |
| CHG-09 | 시험 환경 | SITL 후 실기체 | 환경 프로파일 4종, 실내 비행(GPS 미사용), 실외 비행 | 도심 GPS 불가, 기체 파손 위험 | ENV-04 |
| CHG-10 | 추적 | 미정 | UCMC | 드론 자체 운동 보상 필요 | FUN-04, FUN-05 |
| CHG-11 | 탐지 대상 | 미정 | RC 전차(M1A2) + 기만용 차량 모형 | 오탐 억제 입증 | PRF-04 |
| CHG-12 | 보드 OS | Linux | Ubuntu 24.04 native | HailoRT 호환 | ENV-01 |
| CHG-13 | 통제기 영상 | 미정 | RTSP (영상 트랙 + ONVIF 메타데이터 트랙) | 표준 프로토콜, 프레임과 박스 동기화 | FUN-11 |

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| 0.1 | 2026-11-22 | 초안 (docx, #84) |
| 0.2 | 2026-11-23 | Markdown 으로 전환 (#85 리뷰). CSCI를 기존 프로세스 구성으로 변경하고 추적표의 CSC 를 SDD 3장에 맞춤. SRS-IF-01 에서 MAVSDK 를 빼고 mavlink-router 경유 UDP·생성 C 헤더로, `system/status` 를 `follow/status`·`ARGUS_STATUS` 로, 임무 상태를 `ARGUS_MISSION_STATE` 로, 통제기 영상을 RTSP 로, 명령 중복 판단을 명령 번호(param7)로 수정. CHG-13 추가 |
