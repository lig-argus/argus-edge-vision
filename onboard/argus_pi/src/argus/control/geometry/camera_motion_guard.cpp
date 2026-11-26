#include "argus/control/geometry/include/camera_motion_guard.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace follow {
CameraMotionGuard::CameraMotionGuard(CameraGroundCalibration camera, double margin,
                                     double velocity_p, bool contact, double maximum_speed,
                                     double backward_acceleration)
    : camera_(camera),
      margin_(margin),
      velocity_p_acc_(velocity_p),
      ground_contact_(contact),
      maximum_speed_(maximum_speed),
      backward_acceleration_(backward_acceleration) {
    (void)CalibratedGroundGeometry(camera);
    if (!std::isfinite(margin + velocity_p + maximum_speed + backward_acceleration) ||
        maximum_speed <= 0 || maximum_speed > 20 || margin <= 0 || margin >= .5 ||
        velocity_p <= 0 || velocity_p > 20 || backward_acceleration < 0 ||
        backward_acceleration > 5) {
        throw std::invalid_argument("invalid camera motion constraint");
    }
}
std::optional<CameraMotionLimit> CameraMotionGuard::limit(const Pose& pose,
                                                          const TrackRecordV1& box,
                                                          const FollowCommand& candidate,
                                                          double acceleration) const {
    if (!pose.have_velocity ||
        !std::isfinite(pose.velocity_north_m_s + pose.velocity_east_m_s + pose.velocity_down_m_s) ||
        !std::isfinite(acceleration) || acceleration <= 0) {
        return {};
    }
    const auto& c = camera_;
    const bool visible = box.x1 >= 2 && box.x2 <= c.image_width - 2 && box.y2 >= 2 &&
                         box.y2 <= c.image_height - 2 &&
                         (ground_contact_ ? box.y2 - box.y1 >= 8 : box.y1 >= 2);
    if (!visible) {
        return {};
    }
    const double yaw = pose.yaw_deg * std::acos(-1.) / 180, cy = std::cos(yaw), sy = std::sin(yaw);
    const double forward = cy * pose.velocity_north_m_s + sy * pose.velocity_east_m_s;
    const double right = -sy * pose.velocity_north_m_s + cy * pose.velocity_east_m_s;
    // 속도 setpoint의 slew와 구분해 실제 FC 속도에 대한 P 가속 요청을 제한한다.
    double dx = candidate.vx_m_s - forward, dy = candidate.vy_m_s - right;
    const double cap = acceleration / velocity_p_acc_;
    // 후진 속도의 제동은 양의 보정이다. 방향과 무관하게 기존 제동 예산을 사용한다.
    const bool reverse_braking = forward < 0 && dx > 0;
    const double cap_x = (dx < 0 || reverse_braking) && backward_acceleration_ > 0
                             ? std::max(acceleration, backward_acceleration_) / velocity_p_acc_
                             : cap;
    const double ratio = std::hypot(dx / cap_x, dy / cap);
    if (ratio > 1) {
        dx /= ratio;
        dy /= ratio;
    }
    if (backward_acceleration_ > 0 && ground_contact_ && dx < 0) {
        if (!std::isfinite(pose.pitch_deg)) {
            return {};
        }
        // 실제 하단 여유로 후퇴 기울기를 제한한다. 이미 가까운 정지차에도 작은 후퇴는 남긴다.
        const double margin_angle = std::atan2(c.image_height - 2 - box.y2, c.fy);
        const double up_angle =
            std::clamp(.8 * (pose.pitch_deg * std::acos(-1.) / 180 + margin_angle), 0., 1.);
        dx = std::max(dx, -9.81 * std::tan(up_angle) / velocity_p_acc_);
    }
    if (backward_acceleration_ > 0 && ground_contact_ && dx > 0) {
        if (!std::isfinite(pose.pitch_deg)) {
            return {};
        }
        // 후진 제동은 전체 상단을 보존한다. 전진/정지의 일부잘림 재접근만 하부 기준을 허용한다.
        const double visible_height = std::clamp((box.y2 - box.y1) * .5, 8., 32.);
        const double upper_edge = forward >= 0 && box.y1 < 2 ? box.y2 - visible_height : box.y1;
        const double margin_angle = std::atan2(upper_edge - 2, c.fy);
        const double down_angle =
            std::clamp(.8 * (-pose.pitch_deg * std::acos(-1.) / 180 + margin_angle), 0., 1.);
        dx = std::min(dx, 9.81 * std::tan(down_angle) / velocity_p_acc_);
    }
    FollowCommand command = candidate;
    command.vx_m_s = forward + dx;
    command.vy_m_s = right + dy;
    const double speed = std::hypot(command.vx_m_s, command.vy_m_s);
    if (speed > maximum_speed_) {
        command.vx_m_s *= maximum_speed_ / speed;
        command.vy_m_s *= maximum_speed_ / speed;
    }
    const bool limited =
        std::hypot(command.vx_m_s - candidate.vx_m_s, command.vy_m_s - candidate.vy_m_s) > 1e-9;
    const bool near_edge =
        box.x1 < c.image_width * margin_ || box.x2 > c.image_width * (1 - margin_) ||
        box.y2 < c.image_height * margin_ || box.y2 > c.image_height * (1 - margin_) ||
        (!ground_contact_ && box.y1 < c.image_height * margin_);
    return CameraMotionLimit{command, limited, near_edge};
}
}  // namespace follow
