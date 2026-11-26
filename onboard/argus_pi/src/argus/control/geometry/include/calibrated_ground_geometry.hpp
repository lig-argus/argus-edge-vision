// 검증된 카메라 보정·노출 시각 pose로 full_box 접점을 NED 지면에 투영한다.
#pragma once

#include <array>
#include <chrono>
#include <optional>

#include "argus/control/geometry/include/range_estimate.hpp"
#include "argus/tracking/contracts/include/tracks_v1.hpp"
#include "argus/vehicle/contracts/include/vehicle_state.hpp"

namespace follow {

struct CameraGroundCalibration {
    std::uint32_t image_width{0}, image_height{0};
    double fx{0}, fy{0}, cx{0}, cy{0};
    std::array<double, 4> distortion{};
    // optical camera xyz를 기체 FRD xyz로 바꾸는 보정 회전 행렬(행 우선)이다.
    std::array<double, 9> camera_to_body{};
    std::array<double, 3> camera_offset_body_m{};
    double box_anchor_x{0.5};
    double box_anchor_y{1.0};
    double target_anchor_height_above_ground_m{0};
    double minimum_down_ray{0.05};
    std::chrono::nanoseconds max_pose_bracket_distance{std::chrono::milliseconds(100)};
    std::chrono::milliseconds max_pose_receive_age{200};
};

struct GroundProjection {
    RangeEstimate range{};
    bool bearing_valid{false};
    double bearing_error_deg{0};
    double north_m{0}, east_m{0};
};

class CalibratedGroundGeometry final {
public:
    explicit CalibratedGroundGeometry(CameraGroundCalibration calibration);
    std::chrono::nanoseconds max_pose_bracket_distance() const noexcept {
        return calibration_.max_pose_bracket_distance;
    }
    GroundProjection project(const TracksV1& frame, const TrackRecordV1& target,
                             const Pose& pose_at_exposure, double ground_down_m,
                             Clock::time_point now = Clock::now(), bool bearing_only = false) const;
    // 자세 보정된 절대 NED 관측/목표를 가상 수평 카메라의 정규화 제어 좌표로 바꾼다.
    // 실제 관측이나 그 시각을 새로 생성하지 않는다.
    std::optional<std::array<double, 2>> control_projection(const Pose& pose,
                                                            std::array<double, 2> target_ned,
                                                            double ground_down_m,
                                                            bool require_in_image = true) const;
    // 실제 자세와 장착 offset을 포함한 영상 정규화 좌표. 시야 밖도 비용 계산에 사용한다.
    std::optional<std::array<double, 2>> image_projection(const Pose& pose,
                                                          std::array<double, 3> target_ned) const;
    // 동일 투영의 정규화 좌표와 광학 깊이(m). 예측 박스 축척용이며 새 관측이 아니다.
    std::optional<std::array<double, 3>> image_projection_with_depth(
        const Pose& pose, std::array<double, 3> target_ned) const;

private:
    CameraGroundCalibration calibration_;
};

}  // namespace follow
