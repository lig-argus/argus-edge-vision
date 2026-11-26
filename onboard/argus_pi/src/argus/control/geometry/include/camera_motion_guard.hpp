#pragma once
#include <optional>

#include "argus/control/contracts/include/control_intent.hpp"
#include "argus/control/geometry/include/calibrated_ground_geometry.hpp"
namespace follow {
struct CameraMotionLimit {
    FollowCommand command;
    bool limited{false}, recovery{false};
};
// 실제 관측 접점과 FC 속도를 검사하고 속도 보정량을 제한한다. 관측/TTL을 만들지 않는다.
class CameraMotionGuard final {
public:
    CameraMotionGuard(CameraGroundCalibration camera, double soft_margin, double velocity_p_acc,
                      bool ground_contact = false, double maximum_speed = 1.2,
                      double backward_acceleration = 0);
    std::optional<CameraMotionLimit> limit(const Pose&, const TrackRecordV1&,
                                           const FollowCommand& candidate,
                                           double acceleration_limit) const;

private:
    CameraGroundCalibration camera_;
    double margin_, velocity_p_acc_;
    bool ground_contact_;
    double maximum_speed_, backward_acceleration_;
};
}  // namespace follow
