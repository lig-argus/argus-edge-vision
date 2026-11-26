// 추적 결과를 제어 프로세스와 분리해 전달하는 버전 고정 계약이다.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "argus/tracking/contracts/include/thermal_signature.hpp"

namespace follow {

inline constexpr char kTracksV1Schema[] = "argus.tracking.tracks.v1";
inline constexpr char kTracksV2Schema[] = "argus.tracking.tracks.v2";
// v3는 SITL 센서 시간의 운동 추정과 host 시간의 안전 만료를 분리한다.
inline constexpr char kTracksV3Schema[] = "argus.tracking.tracks.v3";
inline constexpr char kTracksV4Schema[] = "argus.tracking.tracks.v4";
inline constexpr char kTracksV5Schema[] = "argus.tracking.tracks.v5";
// v1 pixel tracks from static camera calibration are observation-only.
inline constexpr char kCameraOnlyTimestampSource[] = "ucmc_camera_only_static_calibration";
inline bool posed_tracks_schema(const std::string& s) {
    return s == kTracksV4Schema || s == kTracksV5Schema;
}
inline bool ground_tracks_schema(const std::string& s) {
    return s == kTracksV2Schema || s == kTracksV3Schema || posed_tracks_schema(s);
}

struct TrackRecordV1 {
    std::int64_t track_id{0};
    std::int32_t class_id{0};
    std::string label;
    double confidence{0};
    double x1{0}, y1{0}, x2{0}, y2{0};
    std::string state{"LOST"};
    bool real_detection{false};
    std::int64_t last_real_detection_monotonic_ns{0};
    // 시험용 v2: FC 센서 시각으로 정렬한 NED 지면 [N,E,varN,covNE,varE,alignment_ms]. GT는
    // 금지한다.
    std::optional<std::array<double, 6>> ground;
    std::optional<std::array<double, 2>>
        ground_anchor;  // v5만: 실제 관측 anchor_y(0/1), 지면 위 높이(m).
    // v5만, 선택: 같은 감지의 RAW14 열 외형 (2026-11-09). thermal_signature.hpp 참고.
    std::optional<ThermalSignature> thermal;
};

struct TracksV1 {
    std::string schema{kTracksV1Schema};
    std::string source;
    std::uint64_t frame_id{0};
    std::int64_t sample_monotonic_ns{0};
    std::string clock_domain;
    std::string timestamp_source;
    std::optional<std::int64_t> exposure_monotonic_ns;
    bool exposure_time_valid{false};
    std::uint32_t image_width{0};
    std::uint32_t image_height{0};
    std::string coordinate_frame{"camera_pixel"};
    std::uint64_t generation{0};
    // Pi v5: 영상 source를 보존하면서 FC 재시작 문맥을 별도로 검증한다.
    std::optional<std::uint64_t> fc_epoch;
    std::vector<TrackRecordV1> tracks;
    std::optional<std::int64_t>
        motion_sample_ns;  // v3만: FC boot에 정렬한 영상 센서 시간. host TTL과 별개.
    std::optional<std::array<double, 6>>
        ground_pose;                   // v4: 같은 ground 관측의 FC N,E,D(m),roll,pitch,yaw(deg).
    std::optional<std::uint64_t> seq;  // v5 공통 발행 순서. 구버전 생산자 읽기 호환을 위해 선택적.
    // 2026-11-19 v5 선택 확장(PAMPC #5): producer가 지면 투영에 실제로 쓴 평면 down(m, NED, EKF z와
    // 같은 좌표)과 투영 자세(roll, pitch, yaw deg, 자세 시각 보정과 ARGUS_CAMERA_PITCH_SCALE 배율
    // 뒤, 배포값 1.0). ground_pose 의미는 그대로다. 구버전 producer는 보내지 않는다. 자료형이
    // 맞으면 비유한 값도 그대로 두고 사용처가 거른다.
    std::optional<double> ground_plane_down_m;
    std::optional<std::array<double, 3>> projection_attitude_deg;
};

// 2026-11-19 PAMPC #5: 같은 프레임의 producer 지면 기준. 두 선택 필드가 모두 있을 때만 만든다.
struct ProducerGround {
    double ground_plane_down_m{0};
    std::array<double, 3> attitude_deg{};  // roll, pitch, yaw (deg)
    std::uint64_t fc_epoch{0};             // 같은 프레임의 FC epoch (없으면 0)
};
inline std::optional<ProducerGround> producer_ground_of(const TracksV1& message) {
    if (!message.ground_plane_down_m || !message.projection_attitude_deg) {
        return std::nullopt;
    }
    return ProducerGround{*message.ground_plane_down_m, *message.projection_attitude_deg,
                          message.fc_epoch.value_or(0)};
}

inline bool camera_only_tracks(const TracksV1& frame) {
    return frame.timestamp_source == kCameraOnlyTimestampSource;
}

// codec와 selector가 같은 조건을 검사한다. 경계별 크기·순서·중복 검사는 호출자가 맡는다.
inline bool valid_tracks_envelope(const TracksV1& message) {
    if (camera_only_tracks(message) &&
        (message.schema != kTracksV1Schema || message.fc_epoch || message.ground_pose ||
         message.motion_sample_ns || message.exposure_time_valid)) {
        return false;
    }
    const bool valid_source =
        (message.schema == kTracksV1Schema || ground_tracks_schema(message.schema)) &&
        !message.source.empty() && !message.clock_domain.empty() &&
        !message.timestamp_source.empty();
    const bool valid_image = message.coordinate_frame == "camera_pixel" &&
                             message.image_width != 0 && message.image_height != 0;
    const bool valid_time =
        message.sample_monotonic_ns >= 0 &&
        message.exposure_time_valid == message.exposure_monotonic_ns.has_value() &&
        (!message.exposure_time_valid ||
         *message.exposure_monotonic_ns == message.sample_monotonic_ns);
    const bool motion_schema =
        message.schema == kTracksV3Schema || posed_tracks_schema(message.schema);
    const bool valid_motion_time = motion_schema
                                       ? message.motion_sample_ns && *message.motion_sample_ns >= 0
                                       : !message.motion_sample_ns;
    bool valid_pose = posed_tracks_schema(message.schema) || !message.ground_pose;
    if (message.ground_pose) {
        for (double v : *message.ground_pose) {
            valid_pose = valid_pose && std::isfinite(v);
        }
    }
    if (posed_tracks_schema(message.schema) && !message.tracks.empty() && !message.ground_pose) {
        valid_pose = false;
    }
    const bool valid_fc_epoch = !message.fc_epoch || message.schema == kTracksV5Schema;
    // 2026-11-19 producer 지면 기준 확장은 v5에만 실린다 (값 범위 검사는 사용처가 한다).
    const bool valid_producer_ground =
        (!message.ground_plane_down_m && !message.projection_attitude_deg) ||
        message.schema == kTracksV5Schema;
    return valid_source && valid_image && valid_time && valid_motion_time && valid_pose &&
           valid_fc_epoch && (!message.seq || message.schema == kTracksV5Schema) &&
           valid_producer_ground;
}

inline bool valid_track_record(const TrackRecordV1& track) {
    const bool valid_identity = track.track_id > 0 && track.class_id >= 0 && !track.label.empty();
    const bool valid_confidence =
        std::isfinite(track.confidence) && track.confidence >= 0 && track.confidence <= 1;
    const bool finite_box = std::isfinite(track.x1) && std::isfinite(track.y1) &&
                            std::isfinite(track.x2) && std::isfinite(track.y2);
    const bool valid_extent =
        track.x1 >= 0 && track.y1 >= 0 && track.x2 > track.x1 && track.y2 > track.y1;
    const bool valid_state =
        (track.state == "CONFIRMED" || track.state == "PREDICTED" || track.state == "LOST") &&
        track.real_detection == (track.state == "CONFIRMED");
    bool valid_ground = true;
    if (track.ground) {
        const auto& g = *track.ground;
        for (double value : g) {
            valid_ground = valid_ground && std::isfinite(value);
        }
        valid_ground = valid_ground && g[2] > 0 && g[4] > 0 && g[2] <= 4 && g[4] <= 4 &&
                       g[2] * g[4] > g[3] * g[3] && g[5] >= 0 && g[5] <= 100;
    }
    if (track.ground_anchor) {
        const auto& a = *track.ground_anchor;
        valid_ground = valid_ground && track.ground && (a[0] == 0 || a[0] == 1) &&
                       std::isfinite(a[1]) && a[1] >= 0 && a[1] <= 3;
    }
    return valid_identity && valid_confidence && finite_box && valid_extent && valid_state &&
           valid_ground;
}

inline bool track_fits_frame(const TrackRecordV1& track, const TracksV1& message) {
    return track.x2 <= message.image_width && track.y2 <= message.image_height &&
           track.last_real_detection_monotonic_ns <= message.sample_monotonic_ns &&
           (track.state != "CONFIRMED" ||
            track.last_real_detection_monotonic_ns == message.sample_monotonic_ns);
}
}  // namespace follow
