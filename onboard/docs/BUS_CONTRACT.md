# ARGUS 공개 버스와 내부 IPC

공개 버스는 기존 ZeroMQ XSUB/XPUB다. Python BusProxy가 중계하고 C++ 노드가 발행한다.
기본 publisher 입력은 tcp://127.0.0.1:5555, 구독 출력은 tcp://*:5556 이다.
PC/Windows는 tcp://<Pi-IP>:5556 에 연결한다.

```text
frame[0] = UTF-8 topic
frame[1] = MessagePack map header
frame[2] = optional JPEG bytes
```

| 토픽 | schema | 생산/상태 |
|---|---|---|
| perception/observations | argus.perception.observations.v2 | C++ / 구현 |
| diagnostics/events | argus.diagnostics.event.v1 | C++ / 구현 |
| vision/overlay/jpeg | argus.vision.overlay.v1 | Pi Python에서 bbox/JPEG 생성 → C++ 발행 / 옵션 |
| vision/npu_input/jpeg | argus.vision.npu_input.v1 | Python NPU 입력 JPEG → C++ 발행 / 디버그 옵션 |
| tracking/tracks | 기존 Track 계약 | 미구현 |
| command/request, command/result | argus.command.*.v1 | 자료형만 / 실행 미구현 |

## Observation v2 — 기존 23 필드 보존

schema, source, frame_id, seq, captured_unix_ns, captured_monotonic_ns,
published_monotonic_ns, image_width, image_height, model, inference_ms,
t_infer_us, t_publish_us, coordinate_frame, observation_kind, detections,
received_unix_ns, received_monotonic_ns, exposure_monotonic_ns,
source_receive_monotonic_ns, clock_domain, timestamp_source, exposure_time_valid.

seq == frame_id. 원본 camera_pixel 좌표, IMAGE2D다.
각 detection은 class_id,label,confidence,x1,y1,x2,y2,track_id의 8개 필드다.
track_id는 실제 Tracker가 없으므로 null이며 hull ROI 필드는 없다.
원본 전체 박스 좌표를 letterbox 좌표와 혼동하지 않는다.
C++ 내부 IPC의 session_id는 공개 관측에 추가하지 않았다.

received_* / captured_* 별칭은 호스트 수신 시각이다. 노출 시각이 아니다.
source_receive_monotonic_ns는 SDK/OpenCV helper의 호스트 콜백/읽기 완료 시각이다.
현재 source는 노출 시각을 제공하지 않으므로 exposure_time_valid=false, exposure_monotonic_ns=null이다.
호스트 단조 시각은 같은 Linux host에서만 비교한다. PC clock과 직접 빼지 않는다.

## JPEG

vision/overlay/jpeg는 원본 크기의 IR/BGR 표시 영상에 같은 frame_id 검출 박스를 Pi에서 그린 JPEG다.
기존 source/frame_id/unix_ns/width/height/encoding 필드와 schema를 유지한다.
seq, session_id, received_monotonic_ns는 보조 메타데이터다.
vision/npu_input/jpeg는 모델 입력의 letterbox 영상이며 원본 bbox 좌표를 그대로 그리면 맞지 않는다.
JPEG 생성은 별도 스레드, preview queue는 latest 1, 기본 overlay 상한 10fps다. 인지 queue capacity 2와 별개다.
PUB/SUB는 전달을 보장하지 않는다. 낮은 HWM만으로 최신 프레임 전달이 보장된다고 주장하지 않는다.

## 내부 C++ ↔ Python

외부 Pub/Sub와 별도 private ipc:// 소켓이다. 0700 임시 디렉터리는 C++가 생성/정리한다.
REQ/REP 요청에 schema argus.ipc.perception.v1, session_id, frame_id, 크기/stride/pixel_format,
수신/노출 시각 메타데이터와 raw bytes를 넣는다. 동시에 처리 중인 요청은 하나다.
Python은 ready/infer/shutdown을 처리한다. C++는 잘못된 응답 ID/좌표계·오류·timeout을 거부한다.
Python의 JPEG 작업자는 별도 PUSH로 C++ PULL 영상 발행 작업자에 전달한다.
Python이 공개 Observation을 발행하거나 기체 명령을 생성하지 않는다.

## Track TTL / 명령

Prediction은 last_real_detection_monotonic_ns와 ttl_ns를 갱신하지 않는다.
발행/배치 expiry로 ID TTL을 연장하지 않는다. ttl 기본 0은 invalid/expired다.
향후 명령은 Guard를 거쳐 단일 VehiclePort에서 직렬화한다. 현재 이 실행 파일은 명령을 송신하지 않는다.
