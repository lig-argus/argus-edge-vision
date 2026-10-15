# 영상 전송 규약 (RTSP + RTP/JPEG + 메타데이터 트랙)

| 항목 | 내용 |
|---|---|
| 문서 버전 | 0.3 (팀 검토 전 초안) |
| 범위 | RPi 영상 송신부 → 통제기: 영상과 프레임별 추적 결과(바운딩 박스) 전송 |
| 따르는 표준 | RTSP (RFC 2326), RTP·RTCP (RFC 3550), RTP/JPEG (RFC 2435), ONVIF Streaming·Analytics 명세의 메타데이터 스트림 형식. ONVIF Profile·SOAP 서비스는 구현하지 않는다 |
| 관련 문서 | `docs/spec/icd.md` (MAVLink: 추종 명령, 스트림 알림), `docs/spec/process_interfaces.md` (RPi 안 토픽, 송신부 매칭 규칙) |
| 상태 표기 | 값 뒤의 **[미결 n]** 은 9장의 검토 항목 번호 |

## 0. 범위

이 문서가 정하는 것
- RTSP 세션 구성, 트랙, SDP
- 영상 트랙의 코덱·패킷화·해상도·FPS
- 메타데이터 트랙의 XML 구조, 추적 결과 필드를 XML 요소로 바꾸는 규칙, 좌표 변환
- 영상 프레임과 메타데이터의 동기화 방법
- 송신부·통제기의 동작 규칙(빈 프레임, 끊김, 박스 클릭 → 표적 선택)
- MAVLink `VIDEO_STREAM_INFORMATION` 내용

이 문서가 정하지 않는 것
- RPi 안에서 영상 송신부가 받는 토픽과 매칭 규칙 → `docs/spec/process_interfaces.md` 5.3
- 추종 명령·상태 → `docs/spec/icd.md`
- 탐지·추적 알고리즘 → `docs/design/system_architecture.md`

## 1. 스트림 구성

```
RPi 영상 송신부 (RTSP 서버)                          통제기 (RTSP 클라이언트)
  rtsp://<RPi>:8554/argus/ir  ──DESCRIBE/SETUP/PLAY──▶
    ├─ 트랙 1: 영상        RTP/JPEG  (m=video)       ──RTP/UDP──▶  디코딩, 표시
    ├─ 트랙 2: 메타데이터  ONVIF XML (m=application) ──RTP/UDP──▶  박스 그리기, 클릭 판정
    └─ RTCP Sender Report (트랙마다)                 ──RTCP────▶  RTP 타임스탬프 ↔ UTC 변환
```

- 트랙: RTSP 세션 안의 개별 미디어 스트림. SDP 의 `m=` 줄 하나에 해당하며(아래 예), 통제기는 트랙마다 `SETUP` 을 보낸다. 영상 트랙·메타데이터 트랙 둘이다. "스트림" 은 이 문서에서 RTSP 세션 전체(`VIDEO_STREAM_INFORMATION` 의 stream)를 가리킨다.
- RTSP 세션 하나에 트랙 둘. 통제기는 둘 다 SETUP 한다. 영상만 보는 도구(VLC 등)는 영상 트랙만 받아도 재생된다.
- URL: `rtsp://<RPi 주소>:8554/argus/<센서>`. 센서 이름은 `ir`, `eo` (EO 카메라 장착 시). 포트·경로는 **[미결 5]**.
- 전송은 RTP/UDP 유니캐스트. 통제기가 RTSP `SETUP` 에서 수신 포트를 알린다. RTCP 는 RTP 포트 + 1.
- 구현 라이브러리: RPi 영상 송신부는 **GStreamer + gst-rtsp-server** (Ubuntu 24.04 apt, GStreamer 1.24), 통제기 수신부는 **GStreamer** (`rtspsrc`). 양쪽이 같은 도구라 송신 테스트 파이프라인을 수신 테스트에 그대로 쓸 수 있고, 코덱 변경이 파이프라인 요소 교체로 끝난다 (5·6장). Windows 배포 구성은 **[미결 1]**, Qt 연결 방식은 **[미결 2]**.

SDP(Session Description Protocol, RFC 8866)는 세션에 어떤 트랙이 있고 어떻게 받는지 적은 텍스트다. 통제기가 RTSP `DESCRIBE` 를 보내면 서버가 SDP 로 답하고, 통제기는 이것을 읽어 어떤 트랙을 `SETUP` 할지 정한다. RTSP 서버가 파이프라인 구성에서 자동으로 만든다.

SDP 예 (DESCRIBE 응답, 주소·페이로드 타입 107 은 예시):
```
v=0                                    SDP 버전 (항상 0)
o=- 0 0 IN IP4 192.168.4.1             세션을 만든 쪽 (서버 주소)
s=argus ir                             세션 이름
t=0 0                                  시간 제한 없음 (라이브)
m=video 0 RTP/AVP 26                   트랙 1: 종류 video, 전송 RTP, 페이로드 타입 26
a=rtpmap:26 JPEG/90000                 26 = JPEG, 클럭 90 kHz (RFC 3551 에 정해진 번호)
a=control:trackID=0                    이 트랙을 SETUP 할 때 쓰는 이름
m=application 0 RTP/AVP 107            트랙 2: 종류 application (영상·음성이 아닌 데이터), 페이로드 타입 107
a=rtpmap:107 vnd.onvif.metadata/90000  107 = ONVIF 메타데이터 XML (96~127 중 서버가 고른 동적 번호라 rtpmap 필수)
a=control:trackID=1
```

- 통제기는 영상 코덱이 JPEG 이라는 것을 SDP 의 `a=rtpmap:26 JPEG/90000` 줄에서 읽는다. MAVLink `VIDEO_STREAM_INFORMATION.encoding` 에는 JPEG 값이 없어 UNKNOWN(0) 을 보낸다 (7장).
- `m=application … vnd.onvif.metadata` 가 있어야 통제기가 메타데이터 트랙을 요청한다. 이 인코딩을 모르는 플레이어(VLC 등)는 영상 트랙만 받는다.

## 2. 영상 트랙 (RTP/JPEG)

| 항목 | 값 |
|---|---|
| 페이로드 | RFC 2435 RTP/JPEG, 정적 페이로드 타입 26, 90 kHz 클럭 |
| 입력 | 탐지 프로세스가 발행하는 박스 없는 **비압축 프레임**(`process_interfaces.md` 5.1 `vision/frame/raw` 가칭, 토픽 이름·픽셀 형식 **[미결 3]**). 전송 코덱 인코딩(JPEG)은 영상 송신부가 한다. 탐지 프로세스는 전송 방식을 모르고, 코덱·품질은 송신부 설정으로 정한다 (`process_interfaces.md` 5.3) |
| JPEG 조건 | baseline 부호화 (영상을 한 번에 순차 부호화. 여러 번 스캔하는 progressive 는 RFC 2435 가 지원하지 않음), 4:2:0 (RFC 2435 type 1), JPEG 규격의 표준 허프만 테이블, 재시작 마커(DRI) 없음. RFC 2435 는 허프만 테이블 을 전송하지 않고 수신기가 JPEG 규격의 표준 허프만 테이블 로 복원하므로, 인코더가 영상마다 만드는 최적화 허프만 테이블 을 쓰면 복원할 수 없다. 송신부의 JPEG 인코더(libjpeg-turbo 또는 GStreamer `jpegenc`)를 이 조건으로 설정한다 (progressive 끔, 허프만 테이블 최적화 끔). 흑백 입력이면 색차 성분 없이 인코딩한다 |
| 해상도 | IR 640×480. RFC 2435 는 8 px 단위, 최대 2040 px. 2040 px 를 넘는 EO 영상은 ONVIF JPEG RTP 헤더 확장(첫 패킷 0xFFD8, 이후 0xFFFF 식별자로 SOF·DQT·DHT 전달)으로 보낸다 **[미결 8]** |
| 품질 | 송신부 설정값. 초기값 75, 화질 측정 뒤 조정 **[미결 12]**. 양자화 테이블은 RFC 2435 의 Q 필드로 전달한다: Q 1~99 는 테이블 보내지 않고 수신기가 RFC 2435 부록 A 의 표준 양자화 테이블을 Q 값으로 배율 조정해 만든다. Q 128~255 는 인코더가 쓴 양자화 테이블을 첫 패킷의 Quantization Table 헤더에 담아 보낸다. 어느 쪽을 쓸지는 패킷화 구현(rtpjpegpay 등)이 정하므로, 통제기는 두 방식을 모두 지원해야 한다 |
| FPS | 최대 30. 고정 주기가 아니며 탐지 쪽 큐 드롭으로 프레임이 빠질 수 있다 |
| RTP 타임스탬프 | 프레임의 `unix_ns` 를 90 kHz 클럭으로 변환한 값. 영상 RTP 는 코덱과 관계없이 90000 Hz 로 정해져 있다 (RFC 3551. 1 틱 ≈ 11.1 µs, 30 FPS 에서 프레임당 3000 틱). 32 비트라 절대 시각이 아니며 RTCP SR 로 UTC 에 대응한다 (4장). 한 프레임의 모든 패킷이 같은 값, 마지막 패킷 marker = 1 |
| 비트 전송률 | 640×480 품질 75 는 프레임당 약 25~35 KB → 30 FPS 에서 약 6~8 Mbps (메타데이터 약 0.3 Mbps 별도). 7장 `bitrate` 에 이 값을 넣는다. 전송 경로는 RPi 와 통제기 사이 Wi-Fi(`docs/spec/icd.md` 1.2)이며 MAVLink 트래픽과 함께 쓴다. 실제 처리량과 손실률은 현장에서 측정한다 **[미결 11]** |

- 영상에는 박스를 그리지 않는다. 박스는 메타데이터 트랙으로만 보내고 통제기가 그린다 (`process_interfaces.md` 5.3).

## 3. 메타데이터 트랙 (ONVIF XML)

### 3.1 RTP 규칙

- SDP: `m=application`, 인코딩 이름 `vnd.onvif.metadata`, 클럭 90 kHz, 동적 페이로드 타입(예 107). 압축(`vnd.onvif.metadata.gzip`)은 쓰지 않는다.
- 페이로드: UTF-8 XML. 영상 프레임 하나마다 `tt:MetadataStream` 문서 하나. 문서가 MTU 를 넘으면 여러 패킷으로 나누고, 문서 끝 패킷에 marker = 1.
- RTP 타임스탬프: 패킷 생성 시각(ONVIF 권고). 영상 프레임과의 동기화에는 쓰지 않고, 동기화는 문서 안 `UtcTime` 으로 한다 (4장).
- 통제기는 marker 가 1 인 패킷까지 모아 XML 하나로 파싱한다. 패킷 손실로 문서가 깨지면 그 프레임의 메타데이터는 버린다.

### 3.2 XML 구조와 추적 결과 필드의 변환

```xml
<tt:MetadataStream xmlns:tt="http://www.onvif.org/ver10/schema" xmlns:argus="urn:argus:metadata:1">
  <tt:VideoAnalytics>
    <tt:Frame UtcTime="2026-10-15T07:15:27.321Z" Source="camera-ir">
      <tt:Transformation>
        <tt:Translate x="-1.0" y="-1.0"/>
        <tt:Scale x="0.003125" y="0.00416667"/>
      </tt:Transformation>
      <tt:Object ObjectId="12">
        <tt:Appearance>
          <tt:Shape>
            <tt:BoundingBox left="100.0" top="380.0" right="150.0" bottom="300.0"/>
            <tt:CenterOfGravity x="125.0" y="340.0"/>
          </tt:Shape>
          <tt:Class>
            <tt:Type Likelihood="0.90">Vehicle</tt:Type>
          </tt:Class>
        </tt:Appearance>
        <tt:Extension>
          <argus:ClassId>0</argus:ClassId>
          <argus:Label>tank</argus:Label>
          <argus:Generation>1</argus:Generation>
          <argus:State>CONFIRMED</argus:State>
        </tt:Extension>
      </tt:Object>
      <tt:Extension>
        <argus:FrameId>100</argus:FrameId>
        <argus:ImageWidth>640</argus:ImageWidth>
        <argus:ImageHeight>480</argus:ImageHeight>
      </tt:Extension>
    </tt:Frame>
  </tt:VideoAnalytics>
</tt:MetadataStream>
```

| `tracking/tracks` 필드 (`process_interfaces.md` 5.1.1) | 들어가는 XML 요소·속성 | 비고 |
|---|---|---|
| JPEG `unix_ns` (같은 frame_id 의 영상 프레임) | `tt:Frame@UtcTime` | ISO 8601, ms 단위, `Z`. 4장 |
| source | `tt:Frame@Source` | 카메라 식별 문자열. 통제기는 onboard 설정의 변환표(source 문자열 → `ARGUS_SENSOR`)로 바꾼다 |
| frame_id, image_width, image_height | `tt:Frame/tt:Extension/argus:*` | ONVIF 스키마에 같은 뜻의 요소가 없어 확장 요소(`tt:Extension`)에 둔다. 요소 이름은 **[미결 4]** |
| tracks[].track_id | `tt:Object@ObjectId` | 1~16,777,215 |
| tracks[].x1, y1, x2, y2 | `tt:BoundingBox` left·top·right·bottom | 3.3 좌표 변환 적용 |
| (x1+x2)/2, (y1+y2)/2 | `tt:CenterOfGravity` | 선택 |
| tracks[].class_id, label | `tt:Object/tt:Extension/argus:ClassId`, `argus:Label` | `tt:Class/tt:Type` 에는 ONVIF 분류명(Vehicle 등)과 confidence 를 넣는다 **[미결 4]** |
| tracks[].confidence | `tt:Type@Likelihood` | 0~1 |
| generation | `argus:Generation` | 표적 선택 식별자 |
| tracks[].state | `argus:State` | 현재 CONFIRMED 만 발행 |

- 트랙이 없는 프레임도 `tt:Frame` 을 보낸다(객체 없음). 통제기는 이것으로 "메타데이터는 살아 있고 객체만 없음" 을 구분한다. ONVIF 는 빈 프레임 생략을 허용하지만 우리는 생략하지 않는다.
- `tracking/tracks` 메시지 자체가 없는 프레임(입력 부재, 프레임 시각에 맞는 FC 표본 없음)도 영상은 보내고 메타데이터는 객체 없는 `tt:Frame` 을 보낸다.

### 3.3 좌표 변환

- 추적 결과 좌표는 영상 픽셀 좌표(원점 왼쪽 위, y 아래 방향, 640×480)다.
- ONVIF 프레임 좌표는 원점 왼쪽 아래, y 위 방향이며 `tt:Transformation` 으로 정규화 좌표(−1~1)로 변환한다. 그래서 **y 를 뒤집는다**:
  ```
  left   = x1
  right  = x2
  top    = image_height - y1    (위쪽 변. 값이 더 큼)
  bottom = image_height - y2
  Translate = (-1, -1), Scale = (2 / image_width, 2 / image_height)
  ```
- 통제기는 `tt:Transformation` 을 적용해 정규화 좌표를 구한 뒤 표시 크기에 맞춰 그린다. 영상 크기가 바뀌어도(EO) 같은 코드로 처리된다.

## 4. 동기화 (영상 프레임 ↔ 메타데이터)

- 송신부는 같은 `(source, frame_id)` 의 JPEG 과 추적 결과를 묶는다. 영상 RTP 타임스탬프와 메타데이터 `UtcTime` 은 **둘 다 그 프레임의 `unix_ns` 를 변환한 값**이다 (RTP 타임스탬프 = unix_ns × 90000 / 10^9 + 시작값, UtcTime = unix_ns 를 ISO 8601 로).
- 영상 트랙의 RTCP Sender Report 는 같은 순간의 RTP 타임스탬프 값과 NTP 시각 값을 함께 알린다. 송신부는 이 NTP 시각도 `unix_ns` 와 같은 RPi 시계(CLOCK_REALTIME)로 만든다.
- 통제기: 영상 프레임의 RTP 타임스탬프 → (RTCP SR) → UTC 시각 → 같은 `UtcTime` 의 메타데이터. 오차 허용은 ±반 프레임(30 FPS 에서 약 ±16 ms).
- `unix_ns` 는 RPi 시계의 호스트 수신 시각이며 노출 시각이 아니다. RPi 시계가 실제 UTC 와 달라도 영상·메타데이터 동기화에는 영향이 없다(둘 다 같은 시계). 녹화한 영상·메타데이터를 나중에 재생하면서 FC 로그 등 다른 장비의 기록과 시각을 맞출 때만 문제가 되며, 그때 RPi 시계 동기(NTP) 를 검토한다 **[미결 6]**.

## 5. 송신부 동작

- 구독·매칭 버퍼·재시작 처리는 `process_interfaces.md` 5.3 을 따른다.
- 프레임마다: 비압축 프레임 → JPEG 인코딩(640×480 흑백 약 1~2 ms) → RTP/JPEG 패킷화, 같은 프레임의 추적 결과 → XML → 메타데이터 패킷. 영상 패킷을 먼저 보낸다.
- 영상 프레임이 없는 추적 결과는 보내지 않는다. 추적 결과가 없는 프레임은 영상 + 빈 `tt:Frame` 을 보낸다.
- IR 카메라 NUC(셔터 보정) 동안 약 0.5 s 프레임이 없다. 송신부는 아무것도 보내지 않으며 RTSP 세션은 유지한다.
- RTSP 클라이언트가 없을 때도 서버는 떠 있고, 토픽 구독은 계속한다(연결 즉시 최신 프레임부터 송신).
- 구현: gst-rtsp-server 에 파이프라인 문자열을 등록한다 (`!` 는 GStreamer 에서 요소를 이어 주는 기호다). 영상 트랙 `appsrc name=video ! jpegenc ! rtpjpegpay name=pay0 pt=26`, 메타데이터 트랙 `appsrc name=meta ! rtppassthroughpay name=pay1 pt=107`. 메타데이터 RTP 패킷(헤더 12 바이트 + XML 조각, 마지막 조각 marker=1)은 송신부가 만들어 `meta` appsrc 에 넣는다 (ONVIF 전용 페이로더 `rtponvifmetadatapay` 는 Rust 플러그인이라 Ubuntu 에 패키지가 없음). 코덱을 바꾸면 `jpegenc ! rtpjpegpay` 를 `x264enc ! rtph264pay` 처럼 요소만 바꾼다.

## 6. 통제기 동작

- 구현: GStreamer `rtspsrc` 가 두 트랙을 받는다. 영상은 `rtpjpegdepay ! jpegdec ! appsink` (코덱이 바뀌면 `rtph264depay ! avdec_h264` 등으로 요소 교체. `decodebin` 으로 SDP 를 보고 자동 선택도 가능), 메타데이터는 `rtponvifmetadatadepay` 가 있으면 그것으로, 없으면 `appsink` 에서 RTP 헤더를 떼고 marker 까지 이어 붙여 XML 문서를 만든다 **[미결 1]**. `rtspsrc` 의 지터 버퍼(`latency`)와 RTCP 처리를 그대로 쓰고, UDP 가 막히면 TCP 로 자동 전환한다(`protocols`).
- 영상 프레임을 디코딩해 표시하고, 4장 방법으로 동기화한 메타데이터의 박스를 그 위에 그린다. 메타데이터가 아직 안 온 프레임은 박스 없이 표시하고, 늦게 온 메타데이터는 버린다.
- 박스 클릭: 클릭 좌표를 정규화 좌표로 바꿔 포함하는 박스(여럿이면 가장 작은 것)를 고른다. 그 객체의 (`Source` → sensor, `argus:Generation`, `argus:ClassId`, `ObjectId`)로 `MAV_CMD_ARGUS_TRACK_START` 를 보낸다 (`docs/spec/icd.md` 4.2). 판정은 **클릭 시점에 화면에 표시된 프레임**의 메타데이터로 한다.
- 선택된 표적(`ARGUS_STATUS.target_id`·`target_generation`)과 `ObjectId`·`argus:Generation` 이 같은 박스는 다른 색으로 강조한다.
- 영상 정지 표시: 새 영상 프레임이 일정 시간(NUC 공백 0.5 s 보다 길게, 예 1 s) 없으면 "영상 정지" 를 표시한다. 시간은 **[미결 7]**. 메타데이터만 끊긴 경우(추적기 미발행)는 박스가 사라지는 것으로 보이며 별도 표시는 `ARGUS_SENSOR_STATUS` 로 한다.

## 7. MAVLink 연계 (`VIDEO_STREAM_INFORMATION`, 269)

| 필드 | 값 |
|---|---|
| 송신 | onboard (compid 195). 통제기가 `MAV_CMD_REQUEST_MESSAGE`(param1 = 269) 로 요청하면 스트림마다 1회 |
| stream_id / count | 1 = IR (EO 추가 시 2) / 스트림 수 |
| type | `VIDEO_STREAM_TYPE_RTSP` (0) |
| flags | `VIDEO_STREAM_STATUS_FLAGS_RUNNING` (1). 송신부가 떠 있지 않으면 0 |
| framerate / resolution_h / resolution_v | 30 / 640 / 480 |
| bitrate | 약 7,000,000 (추정값) |
| rotation / hfov | 0 / 카메라 화각 측정 뒤 (`icd.md` A1). 모르면 0 |
| name | `IR` |
| uri | `rtsp://<RPi 주소>:8554/argus/ir` |
| encoding | `VIDEO_STREAM_ENCODING_UNKNOWN` (0). MAVLink 에 JPEG 값이 없다. 통제기는 코덱을 RTSP `DESCRIBE` 응답의 SDP(`a=rtpmap`)에서 읽는다 (1장) |
| camera_device_id | 0 |

onboard 가 송신부의 상태(실행 여부, 주소)를 아는 방법은 `process_interfaces.md` 에 추가한다 **[미결 9]**.

## 8. class_id 표

`tracking/tracks` 의 `class_id`·`label` 과 메타데이터 `argus:ClassId`·`argus:Label` 의 의미. 모델이 바뀌면 탐지 담당이 이 표와 `ARGUS_SENSOR_STATUS.model_ver` 을 함께 올린다.

| 센서 | model_ver | 모델 파일 | class_id | label | ONVIF `tt:Type` |
|---|---|---|---|---|---|
| IR | 1 | `yolox_s_leaky_v7.3.hef` | — | — | — |

탐지 담당이 채운다 **[미결 10]**. 통제기는 표에 없는 class_id 를 받으면 `argus:Label` 문자열을 그대로 표시한다.

## 9. 검토 필요 사항

1. 통제기 GStreamer Windows 배포 구성: 공식 MSVC 바이너리에 `rtponvifmetadatadepay`(gst-plugins-rs)가 포함되는지 `gst-inspect-1.0` 으로 확인, 필요한 플러그인만 묶었을 때 크기 측정, GStreamer 버전 고정 방법. — 통제기
2. 통제기에서 GStreamer 와 Qt 화면 연결 방식: `appsink` 로 프레임을 꺼내 `QImage` 로 그리기 vs `qml6glsink` 로 GPU 직접 표시. — 통제기
3. 비압축 프레임 토픽 이름·schema·픽셀 형식 (`process_interfaces.md` 7장 3). — 탐지
4. 표준에 없는 값을 넣을 XML 요소: `argus:` 네임스페이스와 요소 이름(FrameId, ImageWidth, ImageHeight, ClassId, Label, Generation, State), `tt:Class/tt:Type` 에 넣을 ONVIF 분류명. — 통신·탐지
5. 포트 8554, 경로 `/argus/<센서>` 확정. — 통신
6. RPi 시계 UTC 동기(NTP) 여부. 영상·메타데이터 동기화에는 불필요하고, 녹화 파일과 다른 장비 로그의 시각을 맞출 때 필요. — 통신
7. 통제기 "영상 정지" 판단 시간 (NUC 0.5 s 보다 길게). — 통제기
8. EO 카메라 해상도가 2040 px 를 넘을 때 ONVIF JPEG RTP 헤더 확장 적용 여부와 수신 지원. — 통신·탐지
9. onboard 가 송신부 상태를 받는 방법 (`VIDEO_STREAM_INFORMATION` flags 용). — 통신
10. IR 모델 v7.3 class_id 표. — 탐지
11. RPi–통제기 Wi-Fi 의 실제 처리량·손실률 측정 (영상 6~8 Mbps + 메타데이터 + MAVLink). 부족하면 품질·FPS 조정. — 통신
12. JPEG 품질값: 비압축 프레임 대 JPEG 의 화질(PSNR·육안)을 품질 75·90·95 에서 측정해 정함. 비트 전송률(11)과 함께 판단. — 통신·탐지

## 변경 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| 0.1 | 2026-10-15 | 팀 검토 전 초안. 스트림 구성, RTP/JPEG, ONVIF 메타데이터 XML 과 필드 변환, 좌표 변환, 동기화, 송신부·통제기 동작, VIDEO_STREAM_INFORMATION, class_id 표, 미결 1~11 |
| 0.3 | 2026-10-15 | 구현 라이브러리 결정: RPi gst-rtsp-server, 통제기 GStreamer (1·5·6장). 미결 1·2 를 배포 구성·Qt 연결로 교체 |
| 0.2 | 2026-10-15 | 입력을 비압축 프레임으로 바꾸고 JPEG 인코딩·품질을 송신부 책임으로 (2·5장, 미결 3·12) |
