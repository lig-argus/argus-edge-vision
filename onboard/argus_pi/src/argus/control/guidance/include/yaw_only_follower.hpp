// 보정된 body bearing(deg)을 yaw 속도(deg/s)로만 바꾼다. 대상 기억/FC 송신은 소유하지 않는다.
#pragma once
#include "argus/control/guidance/include/yaw_rate_smoother.hpp"
namespace follow {
class YawOnlyFollower final {
public:
    YawOnlyFollower(double kp, double maximum_rate, double bearing_deadband,
                    YawSmoothingConfig smoothing);
    double update(double bearing_deg, double dt_s);
    void reset();

private:
    double kp_, maximum_rate_, bearing_deadband_;
    YawRateSmoother smoother_;
};
}  // namespace follow
