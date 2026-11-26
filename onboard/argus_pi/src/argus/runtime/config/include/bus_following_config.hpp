// observations PUB/SUB 이후 공통 제어 경로에 필요한 설정만 선언한다.
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <istream>
#include <string>
#include <vector>

#include "argus/control/geometry/include/calibrated_ground_geometry.hpp"
#include "argus/control/runtime/include/flight_control_runtime.hpp"
#include "argus/control/target/include/target_selector.hpp"
#include "argus/perception/observations/include/observation_inbox.hpp"

namespace follow {
// Existing deployed configuration keys remain readable without an image-tracker dependency.
struct ObservationCompatibilityConfig {
    std::vector<std::int32_t> accepted_class_ids{2};
    float input_score_floor{0.1f};
    double lost_track_ttl_s{1.0}, max_prediction_gap_s{0.5};
    struct Thresholds {
        float high_score{0.3f}, new_track_score{0.4f};
    } thresholds;
};
struct RotationCompatibilityConfig {
    std::uint32_t calibration_width{0}, calibration_height{0};
    double fx{0}, fy{0}, cx{0}, cy{0};
    std::array<double, 9> camera_to_body{};
    double max_frame_interval_s{0.5}, max_rotation_deg{35};
    std::chrono::nanoseconds max_pose_bracket_distance{std::chrono::milliseconds(100)};
    std::chrono::milliseconds max_pose_receive_age{200};
};

struct BusFollowingConfig {
    std::string observations_sub_endpoint;
    std::string tracks_pub_endpoint;
    std::string tracks_sub_endpoint;
    std::string status_pub_endpoint;
    std::string command_rep_endpoint;
    std::string px4_connection_url;
    std::string expected_fc_uid{"000600000000353733323333511700280027"};
    std::uint8_t expected_fc_system_id{1}, expected_fc_component_id{1};
    ObservationInboxConfig inbox;
    ObservationCompatibilityConfig tracker;
    TargetSelectorConfig selector;
    CameraGroundCalibration geometry;
    RotationCompatibilityConfig rotation;
    FlightGuidanceConfig guidance;
    FlightControlRuntimeConfig runtime;
    bool enable_rotation_cmc{false};
    bool external_tracks{false};
    std::string diagnostics_csv;
    // 2026-11-12 정비 선택값과 같은 추종 모드 이름. 비어 있으면 예전 판별(높이 3.0 = 3 m)을 쓴다.
    std::string follow_controller_profile;
    std::chrono::milliseconds max_observation_age{250};
    std::chrono::milliseconds control_period{50};
    std::chrono::milliseconds watchdog_period{25};
};

// 2026-11-12 실외 모드: 45° 장착 4 m·3 m, 30° 장착 4 m·3 m (실내는 predictive만).
// 2026-11-18 실내 데모 모드(demo): 회전 추종만, 고도 1.5 m 유지, 45° 장착. 실내에서만 쓴다.
inline bool valid_follow_controller_profile(const std::string& name) {
    return name == "predictive" || name == "predictive_low" || name == "predictive_m30" ||
           name == "predictive_low_m30" || name == "demo" || name == "demo2";
}
// 2026-11-19 실내 데모2(demo2): 예측 이동 추종이지만 yaw를 거의 고정하고 옆이동으로 맞춘다. 실내
// 전용.
inline bool demo2_follow_controller_profile(const std::string& name) { return name == "demo2"; }
// 실내 데모 모드인지 (yaw 전용 설정 두 개로만 실행한다)
inline bool demo_follow_controller_profile(const std::string& name) { return name == "demo"; }
// 30° 장착 모드인지 (아니면 45° 장착)
inline bool mount30_follow_controller_profile(const std::string& name) {
    return name == "predictive_m30" || name == "predictive_low_m30";
}
// 데모 모드에서 이동 추종(moving) 전환을 거부할 때의 응답 문구
inline constexpr const char* kDemoMovingModeRejected = "데모 모드는 회전 추종만 가능합니다";
// 운용 "mode" 요청을 이 추종 모드 이름에서 거부해야 하면 그 이유, 아니면 빈 문자열
inline std::string follow_mode_change_rejection(const std::string& follow_controller_profile,
                                                const std::string& mode) {
    if (demo_follow_controller_profile(follow_controller_profile) && mode == "moving") {
        return kDemoMovingModeRejected;
    }
    return {};
}

// key=value 형식을 엄격하게 읽어 미설정·중복·미지원 키를 시작 전에 거부한다.
BusFollowingConfig parse_bus_following_config(std::istream& input);
BusFollowingConfig load_bus_following_config(const std::string& path);

}  // namespace follow
