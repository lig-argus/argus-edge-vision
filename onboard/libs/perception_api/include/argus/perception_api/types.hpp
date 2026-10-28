// 탐지·추적 결과 값 타입. 영상 송신부, onboard(LatestTracks), 추적기가 함께 쓴다.
// 필드와 단위는 docs/spec/process_interfaces.md 5.1 (vision/frame/gray8, perception/observations,
// tracking/tracks 5.1.1, tracking/status 5.1.3)과 같다. 각도는 rad, 거리는 m, 시각은 RPi monotonic
// clock 의 ns, 박스 좌표는 원본 영상 픽셀(원점 왼쪽 위, y 아래 방향)이다.
// 메시지(MessagePack) 안의 deg·ms 값은 어댑터가 받을 때 rad·s 로 바꾼다 (process_interfaces.md
// 3장).
#ifndef ARGUS_PERCEPTION_API_TYPES_HPP_
#define ARGUS_PERCEPTION_API_TYPES_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace argus::perception_api {

// 비압축 8 비트 흑백 프레임 (`vision/frame/gray8`). 박스를 그리지 않은 원본 픽셀 좌표계
struct Frame {
    // 카메라 식별 문자열. ARGUS_SENSOR 와의 대응은 onboard 설정 (SensorSourceMap)
    std::string source;

    // 탐지 프로세스 실행마다 바뀌는 값. 바뀌면 frame_id 가 처음부터 다시 시작할 수 있으므로
    // FrameCollector 는 보관 중인 메시지를 비운다
    std::string session_id;

    std::uint64_t frame_id = 0;

    // RPi 수신 시각 (wall clock). 영상, 메타데이터 동기화 기준 (video.md 4장)
    std::int64_t unix_ns = 0;

    std::int64_t received_monotonic_ns = 0;  // RPi 수신 시각 (monotonic clock). 노출 시각이 아니다
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t stride = 0;          // 한 줄의 바이트 수 (width 이상)
    std::vector<std::uint8_t> pixels;  // stride × height 바이트

    std::size_t expected_pixel_bytes() const {
        return static_cast<std::size_t>(stride) * static_cast<std::size_t>(height);
    }
};

// 원본 영상 픽셀 좌표의 박스. (x1, y1) 왼쪽 위, (x2, y2) 오른쪽 아래
struct PixelBox {
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;

    double width() const { return x2 - x1; }
    double height() const { return y2 - y1; }
};

// 분류 결과. class_id 와 label 의 대응은 모델 버전에 따른다 (video.md 8장)
struct Classification {
    std::int32_t class_id = 0;
    std::string label;
    double confidence = 0.0;  // 0~1
};

// 탐지 한 건 (`perception/observations` v2 의 detections[]). track_id 는 없다 (추적기가 붙인다)
struct Detection {
    Classification classification;
    PixelBox box;
};

// 한 프레임의 탐지 결과 (`perception/observations` v2). 객체가 없으면 detections 가 비어 있다.
// 큐에서 버린 프레임은 발행되지 않으므로 frame_id 는 건너뛸 수 있다
struct Observations {
    std::string source;
    std::uint64_t frame_id = 0;

    // 메시지의 received_monotonic_ns (RPi 수신 시각, 노출 시각이 아니다). 추적기가 FC 자세를
    // 맞추는 기준
    std::int64_t sample_monotonic_ns = 0;

    std::string model;  // 추론한 모델(HEF) 이름. class_id 와 label 의 대응은 모델마다 다르다
    std::uint32_t image_width = 0;
    std::uint32_t image_height = 0;
    std::vector<Detection> detections;
};

// 트랙 상태 (`tracking/tracks` tracks[].state). 지금은 Confirmed 만 발행된다 (process_interfaces.md
// 미결 6)
enum class TrackState : std::uint8_t {
    Confirmed,
    Predicted,
};

// 트랙의 지면 투영 (`tracking/tracks` tracks[].ground). FC 자세가 없으면 없다
struct GroundEstimate {
    double north_m = 0.0;
    double east_m = 0.0;
    double var_north_m2 = 0.0;
    double cov_north_east_m2 = 0.0;
    double var_east_m2 = 0.0;
    double alignment_s = 0.0;  // 투영에 쓴 FC 자세와 프레임의 시각 차
};

// 지면 투영에 쓴 박스 기준점 (`tracking/tracks` tracks[].ground_anchor)
struct GroundAnchor {
    double anchor_y_px = 0.0;  // 박스에서 지면에 닿는 것으로 본 y 좌표 (픽셀)
    double height_m = 0.0;     // 기준점의 지면 위 높이
};

// 프레임 시각의 기체 위치·자세 (`tracking/tracks` ground_pose). NED, 각도 rad
struct GroundPose {
    double north_m = 0.0;
    double east_m = 0.0;
    double down_m = 0.0;
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
};

// 트랙 하나 (`tracking/tracks` v5 tracks[])
struct Track {
    std::uint32_t track_id = 0;  // 1~16,777,215. 추적기 재시작 시 1 부터 (generation 으로 구분)
    Classification classification;
    PixelBox box;
    TrackState state = TrackState::Confirmed;
    bool real_detection = false;  // 이 프레임에 실제 탐지가 있었는지 (예측만이면 false)
    std::int64_t last_real_detection_monotonic_ns = 0;
    std::optional<GroundEstimate> ground;
    std::optional<GroundAnchor> ground_anchor;
};

// 한 프레임의 추적 결과 (`tracking/tracks` v5). source·frame_id 는 프레임·탐지 결과와 같은 값이며
// 세 메시지를 묶는 키다. 트랙이 없으면 tracks 가 비어 있고, 메시지 자체가 없을 수도 있다
struct Tracks {
    std::string source;
    std::uint64_t frame_id = 0;
    std::uint64_t fc_epoch = 0;
    std::uint32_t generation = 0;  // 1~16,777,215. 추적기 재시작·초기화마다 1 증가
    std::int64_t sample_monotonic_ns = 0;
    std::uint32_t image_width = 0;
    std::uint32_t image_height = 0;
    std::optional<GroundPose> ground_pose;
    std::vector<Track> tracks;
};

// 추적기 상태 (`tracking/status` v1, 1 Hz). ARGUS_SENSOR_STATUS 보고에 쓴다
struct TrackingStatus {
    std::string source;
    std::uint32_t generation = 0;
    bool fc_fresh = false;   // 프레임 시각에 맞는 FC 메시지가 있는지
    bool cmc_valid = false;  // 카메라 움직임 보정이 유효한지
    double fps = 0.0;
};

}  // namespace argus::perception_api

#endif  // ARGUS_PERCEPTION_API_TYPES_HPP_
