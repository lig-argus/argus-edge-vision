// 같은 단위/축으로 보정된 camera ray와 exposure 시각 pose의 지면 교점을 확인한다.
#include "argus/control/geometry/include/calibrated_ground_geometry.hpp"

#include <cassert>
#include <cmath>
#include <string>

using namespace follow;

namespace {
CameraGroundCalibration calibration() {
    CameraGroundCalibration c;
    c.image_width = 640;
    c.image_height = 480;
    c.fx = c.fy = 500;
    c.cx = 320;
    c.cy = 240;
    // Nadir 카메라: optical right→body right, optical down→body aft, optical forward→body down.
    c.camera_to_body = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    c.box_anchor_x = 0.5;
    c.box_anchor_y = 0.5;
    return c;
}
TracksV1 observation() {
    TracksV1 f;
    f.source = "calibrated-camera";
    f.frame_id = 1;
    f.sample_monotonic_ns = 1500000000;
    f.clock_domain = "host_monotonic";
    f.timestamp_source = "camera_exposure";
    f.exposure_monotonic_ns = f.sample_monotonic_ns;
    f.exposure_time_valid = true;
    f.image_width = 640;
    f.image_height = 480;
    return f;
}
}  // namespace

int main() {
    const auto now = Clock::now();
    auto frame = observation();
    TrackRecordV1 target{
        4, 5, "rc_car", 0.9, 360, 230, 380, 250, "CONFIRMED", true, frame.sample_monotonic_ns};
    Pose pose;
    pose.north_m = 10;
    pose.east_m = 20;
    pose.down_m = -5;
    pose.yaw_deg = 90;
    pose.position_monotonic_ns = pose.attitude_monotonic_ns = frame.sample_monotonic_ns;
    pose.clock_domain = frame.clock_domain;
    pose.position_at = pose.attitude_at = now;
    pose.have_position = pose.have_attitude = true;

    CalibratedGroundGeometry geometry(calibration());
    const auto projection = geometry.project(frame, target, pose, 0, now);
    // 잘못된/멀리 있는 지면 좌표도 yaw 전용 광선의 방향을 바꾸지 않는다.
    auto grounded = frame;
    grounded.schema = kTracksV2Schema;
    auto grounded_target = target;
    grounded_target.ground = std::array<double, 6>{1000, -1000, .1, 0, .1, 0};
    const auto visual = geometry.project(grounded, grounded_target, pose, 0, now, true);
    assert(visual.bearing_valid && std::abs(visual.bearing_error_deg - 90) < 1e-6 &&
           !visual.range.valid);
    assert(projection.range.valid && std::string(projection.range.quality) == "METRIC");
    assert(std::abs(projection.range.rho_m - 0.5) < 1e-6);
    assert(projection.bearing_valid && std::abs(projection.bearing_error_deg - 90.0) < 1e-6);
    assert(std::abs(projection.north_m + 0.5) < 1e-6 && std::abs(projection.east_m) < 1e-6);

    frame.exposure_time_valid = false;
    frame.exposure_monotonic_ns.reset();
    const auto unaligned = geometry.project(frame, target, pose, 0, now);
    // 노출 정렬이 없어도 보정된 body 방향은 유효할 수 있지만 미터 거리는 사용할 수 없다.
    assert(!unaligned.range.valid && unaligned.bearing_valid);
    assert(std::abs(unaligned.bearing_error_deg - 90.0) < 1e-6);
    frame = observation();
    for (const bool bearing : {true, false}) {
        auto box = target;
        if (!bearing) {
            box.x1 = 310;
            box.x2 = 330;
        }
        const auto below = geometry.project(frame, box, pose, -10, now);
        assert(!below.range.valid && std::isnan(below.range.rho_m));
        assert(below.bearing_valid == bearing &&
               std::string(below.range.quality) == (bearing ? "SCALE_ONLY" : "INVALID"));
        assert(std::string(below.range.reason) == "target_plane_not_below_camera");
        auto inverted = pose;
        inverted.roll_deg = 180;
        const auto ray = geometry.project(frame, box, inverted, 0, now);
        assert(!ray.range.valid && ray.bearing_valid == bearing);
        assert(std::string(ray.range.quality) == (bearing ? "SCALE_ONLY" : "INVALID"));
        assert(std::string(ray.range.reason) == "ray_does_not_intersect_ground");
    }
    target.x1 = 310;
    target.x2 = 330;
    frame.exposure_time_valid = false;
    const auto unaligned_vertical = geometry.project(frame, target, pose, 0, now);
    assert(!unaligned_vertical.bearing_valid &&
           std::string(unaligned_vertical.range.quality) == "SCALE_ONLY");
    return 0;
}
