// 박스 기준점의 camera ray를 보정 행렬·FC pose로 NED에 옮겨 지면과 교차시킨다.
#include "argus/control/geometry/include/calibrated_ground_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "argus/perception/geometry/include/pinhole_distortion.hpp"

namespace follow {
namespace {
struct Vec3 {
    double x, y, z;
};
constexpr double kDegToRad = 0.01745329251994329577;
constexpr double kRadToDeg = 57.295779513082320876;

Vec3 rotate(const std::array<double, 9>& matrix, Vec3 v) {
    return {matrix[0] * v.x + matrix[1] * v.y + matrix[2] * v.z,
            matrix[3] * v.x + matrix[4] * v.y + matrix[5] * v.z,
            matrix[6] * v.x + matrix[7] * v.y + matrix[8] * v.z};
}
Vec3 body_to_ned(const Pose& pose, Vec3 v) {
    const double r = pose.roll_deg * kDegToRad;
    const double p = pose.pitch_deg * kDegToRad;
    const double y = pose.yaw_deg * kDegToRad;
    const double cr = std::cos(r), sr = std::sin(r);
    const double cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);
    return {cy * cp * v.x + (cy * sp * sr - sy * cr) * v.y + (cy * sp * cr + sy * sr) * v.z,
            sy * cp * v.x + (sy * sp * sr + cy * cr) * v.y + (sy * sp * cr - cy * sr) * v.z,
            -sp * v.x + cp * sr * v.y + cp * cr * v.z};
}
double wrap_degrees(double value) { return std::remainder(value, 360.0); }
bool valid_rotation(const std::array<double, 9>& r) {
    for (double value : r) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    const Vec3 x{r[0], r[3], r[6]}, y{r[1], r[4], r[7]}, z{r[2], r[5], r[8]};
    const auto dot = [](Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; };
    const Vec3 cross{y.y * z.z - y.z * z.y, y.z * z.x - y.x * z.z, y.x * z.y - y.y * z.x};
    return std::abs(dot(x, x) - 1) < 1e-3 && std::abs(dot(y, y) - 1) < 1e-3 &&
           std::abs(dot(z, z) - 1) < 1e-3 && std::abs(dot(x, y)) < 1e-3 &&
           std::abs(dot(x, z)) < 1e-3 && std::abs(dot(y, z)) < 1e-3 &&
           std::abs(dot(x, cross) - 1) < 1e-3;
}
GroundProjection invalid(const char* reason) {
    GroundProjection result;
    result.range = {NAN, "INVALID", reason, false};
    return result;
}
GroundProjection without_ground_range(GroundProjection result, const char* reason) {
    if (!result.bearing_valid) {
        return invalid(reason);
    }
    result.range = {NAN, "SCALE_ONLY", reason, false};
    return result;
}
}  // namespace

CalibratedGroundGeometry::CalibratedGroundGeometry(CameraGroundCalibration calibration)
    : calibration_(std::move(calibration)) {
    const auto& c = calibration_;
    for (double v : c.distortion) {
        if (!std::isfinite(v)) {
            throw std::invalid_argument("invalid distortion");
        }
    }
    const bool invalid_image = !c.image_width || !c.image_height;
    const bool invalid_focal_lengths =
        !std::isfinite(c.fx) || c.fx <= 0 || !std::isfinite(c.fy) || c.fy <= 0;
    const bool invalid_principal_point = !std::isfinite(c.cx) || !std::isfinite(c.cy) || c.cx < 0 ||
                                         c.cx > c.image_width || c.cy < 0 || c.cy > c.image_height;
    const bool invalid_mount =
        !valid_rotation(c.camera_to_body) ||
        !std::all_of(c.camera_offset_body_m.begin(), c.camera_offset_body_m.end(),
                     [](double value) { return std::isfinite(value); });
    const bool invalid_anchor = !std::isfinite(c.box_anchor_x) || c.box_anchor_x < 0 ||
                                c.box_anchor_x > 1 || !std::isfinite(c.box_anchor_y) ||
                                c.box_anchor_y < 0 || c.box_anchor_y > 1;
    const bool invalid_ground_ray = !std::isfinite(c.target_anchor_height_above_ground_m) ||
                                    !std::isfinite(c.minimum_down_ray) || c.minimum_down_ray <= 0 ||
                                    c.minimum_down_ray >= 1;
    const bool invalid_pose_limits =
        c.max_pose_bracket_distance.count() <= 0 || c.max_pose_receive_age.count() <= 0;
    if (invalid_image || invalid_focal_lengths || invalid_principal_point || invalid_mount ||
        invalid_anchor || invalid_ground_ray || invalid_pose_limits) {
        throw std::invalid_argument("invalid camera/ground calibration");
    }
}

GroundProjection CalibratedGroundGeometry::project(const TracksV1& frame,
                                                   const TrackRecordV1& target, const Pose& pose,
                                                   double ground_down_m, Clock::time_point now,
                                                   bool bearing_only) const {
    // 1. 실관측 박스와 보정 영상의 계약을 확인한다.
    if ((frame.schema != kTracksV1Schema && !ground_tracks_schema(frame.schema)) ||
        frame.coordinate_frame != "camera_pixel" || target.state != "CONFIRMED" ||
        !target.real_detection ||
        target.last_real_detection_monotonic_ns != frame.sample_monotonic_ns) {
        return invalid("target_or_coordinate_contract_invalid");
    }
    if (frame.image_width != calibration_.image_width ||
        frame.image_height != calibration_.image_height) {
        return invalid("calibration_or_image_size_mismatch");
    }
    const bool nonfinite_box = !std::isfinite(target.x1) || !std::isfinite(target.y1) ||
                               !std::isfinite(target.x2) || !std::isfinite(target.y2);
    const bool invalid_extent =
        target.x1 < 0 || target.y1 < 0 || target.x2 <= target.x1 || target.y2 <= target.y1;
    const bool outside_image = target.x2 > frame.image_width || target.y2 > frame.image_height;
    if (nonfinite_box || invalid_extent || outside_image) {
        return invalid("invalid_full_box");
    }

    // v2는 FC 센서 시각에 정렬해 외부 adapter가 검증한 절대 NED 지면 관측을 담는다.
    // 유효성/분산/정렬 오차는 codec와 여기에서 재검사하며 v1의 노출 계약은 그대로 둔다.
    if (!bearing_only && ground_tracks_schema(frame.schema) && target.ground) {
        const auto sampled = Clock::time_point{std::chrono::nanoseconds{frame.sample_monotonic_ns}};
        if (!valid_track_record(target) || frame.clock_domain != "host_monotonic" ||
            now < sampled || now - sampled > std::chrono::milliseconds(250) || !pose.fresh(now)) {
            return invalid("ground_observation_stale_or_invalid");
        }
        const auto& g = *target.ground;
        GroundProjection grounded;
        grounded.north_m = g[0] - pose.north_m;
        grounded.east_m = g[1] - pose.east_m;
        const double range = std::hypot(grounded.north_m, grounded.east_m);
        if (!std::isfinite(range) || range > 50) {
            return invalid("ground_range_invalid");
        }
        grounded.bearing_error_deg =
            wrap_degrees(std::atan2(grounded.east_m, grounded.north_m) * kRadToDeg - pose.yaw_deg);
        grounded.bearing_valid = true;
        grounded.range = {range, "METRIC", "fc_sensor_aligned_ground_observation", true};
        return grounded;
    }

    // 2. 박스 기준점을 body 광선으로 바꿔 yaw 방향을 계산한다.
    const double u = target.x1 + calibration_.box_anchor_x * (target.x2 - target.x1);
    const double v = target.y1 + calibration_.box_anchor_y * (target.y2 - target.y1);
    std::array<double, 2> ideal;
    try {
        ideal =
            undistort_normalized((u - calibration_.cx) / calibration_.fx,
                                 (v - calibration_.cy) / calibration_.fy, calibration_.distortion);
    } catch (const std::exception&) {
        return invalid("distortion_inverse_invalid");
    }
    const Vec3 camera_ray{ideal[0], ideal[1], 1.0};
    const Vec3 body_ray = rotate(calibration_.camera_to_body, camera_ray);
    GroundProjection result;
    const double body_horizontal = std::hypot(body_ray.x, body_ray.y);
    if (std::isfinite(body_horizontal) && body_horizontal > 1e-9) {
        // 픽셀 광선과 카메라 장착 보정으로 body 기준 방향은 계산한다. 노출 시각을 추정하지 않는다.
        result.bearing_error_deg = std::atan2(body_ray.y, body_ray.x) * kRadToDeg;
        result.bearing_valid = true;
    }

    // yaw 전용은 지면 점/거리 대신 원 영상의 보정된 광선만 사용한다.
    if (bearing_only) {
        result.range = {NAN, "SCALE_ONLY", "yaw_only_bearing", false};
        return result;
    }
    // 3. 노출과 pose가 정렬된 경우에만 NED 지면 교차로 미터 거리를 계산한다.
    const bool exposure_aligned = frame.exposure_time_valid && frame.exposure_monotonic_ns &&
                                  *frame.exposure_monotonic_ns == frame.sample_monotonic_ns &&
                                  frame.clock_domain == pose.clock_domain &&
                                  pose.position_monotonic_ns == frame.sample_monotonic_ns &&
                                  pose.attitude_monotonic_ns == frame.sample_monotonic_ns;
    const bool unaligned_pose =
        !exposure_aligned || !pose.fresh(now) || now < pose.position_at || now < pose.attitude_at;
    const bool stale_pose = now - pose.position_at > calibration_.max_pose_receive_age ||
                            now - pose.attitude_at > calibration_.max_pose_receive_age;
    const bool nonfinite_geometry = !std::isfinite(ground_down_m) || !std::isfinite(pose.north_m) ||
                                    !std::isfinite(pose.east_m) || !std::isfinite(pose.down_m) ||
                                    !std::isfinite(pose.roll_deg) ||
                                    !std::isfinite(pose.pitch_deg) || !std::isfinite(pose.yaw_deg);
    if (unaligned_pose || stale_pose || nonfinite_geometry) {
        // 정렬 pose가 없으면 미터 거리만 무효로 두고, 보정된 body 방향은 유지한다.
        result.range = {NAN, "SCALE_ONLY", "exposure_or_pose_not_time_aligned", false};
        return result;
    }

    const Vec3 ned_ray = body_to_ned(pose, body_ray);
    const Vec3 camera_offset_ned = body_to_ned(
        pose, {calibration_.camera_offset_body_m[0], calibration_.camera_offset_body_m[1],
               calibration_.camera_offset_body_m[2]});

    const double horizontal = std::hypot(ned_ray.x, ned_ray.y);
    if (std::isfinite(horizontal) && horizontal > 1e-9) {
        const double absolute_bearing = std::atan2(ned_ray.y, ned_ray.x) * kRadToDeg;
        result.bearing_error_deg = wrap_degrees(absolute_bearing - pose.yaw_deg);
        result.bearing_valid = true;
    }
    if (!std::isfinite(ned_ray.z) || ned_ray.z < calibration_.minimum_down_ray) {
        return without_ground_range(result, "ray_does_not_intersect_ground");
    }
    const double target_down = ground_down_m - calibration_.target_anchor_height_above_ground_m;
    const double height = target_down - (pose.down_m + camera_offset_ned.z);
    const double lambda = height / ned_ray.z;
    if (!std::isfinite(lambda) || lambda <= 0) {
        return without_ground_range(result, "target_plane_not_below_camera");
    }
    result.north_m = camera_offset_ned.x + lambda * ned_ray.x;
    result.east_m = camera_offset_ned.y + lambda * ned_ray.y;
    const double distance = std::hypot(result.north_m, result.east_m);
    if (!std::isfinite(distance)) {
        return invalid("non_finite_ground_intersection");
    }
    result.range = {distance, "METRIC", "calibrated_ground_plane_intersection", true};
    return result;
}

std::optional<std::array<double, 2>> CalibratedGroundGeometry::control_projection(
    const Pose& pose, std::array<double, 2> target, double ground_down,
    bool require_in_image) const {
    if (!std::isfinite(target[0] + target[1] + ground_down + pose.north_m + pose.east_m +
                       pose.down_m + pose.yaw_deg)) {
        return {};
    }
    // 원 관측은 FC 자세로 이미 NED 복원됐다. 제어에는 roll/pitch를 제거한 기준 frame을 써
    // 드론 자체 기울기의 영상 변화가 PID 미분항에 다시 들어가지 않게 한다.
    Pose level = pose;
    level.roll_deg = level.pitch_deg = 0;
    const Vec3 offset = body_to_ned(
        level, {calibration_.camera_offset_body_m[0], calibration_.camera_offset_body_m[1],
                calibration_.camera_offset_body_m[2]});
    const Vec3 relative{
        target[0] - pose.north_m - offset.x, target[1] - pose.east_m - offset.y,
        ground_down - calibration_.target_anchor_height_above_ground_m - pose.down_m - offset.z};
    const auto dot = [](Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; };
    const Vec3 body{dot(relative, body_to_ned(level, {1, 0, 0})),
                    dot(relative, body_to_ned(level, {0, 1, 0})),
                    dot(relative, body_to_ned(level, {0, 0, 1}))};
    const auto& r = calibration_.camera_to_body;
    const Vec3 optical{r[0] * body.x + r[3] * body.y + r[6] * body.z,
                       r[1] * body.x + r[4] * body.y + r[7] * body.z,
                       r[2] * body.x + r[5] * body.y + r[8] * body.z};
    if (!std::isfinite(optical.x + optical.y + optical.z) || optical.z <= .01) {
        return {};
    }
    const auto raw =
        distort_normalized(optical.x / optical.z, optical.y / optical.z, calibration_.distortion);
    const double u = calibration_.cx + calibration_.fx * raw[0];
    const double v = calibration_.cy + calibration_.fy * raw[1];
    if (require_in_image &&
        (u < 0 || v < 0 || u > calibration_.image_width || v > calibration_.image_height)) {
        return {};
    }
    return std::array<double, 2>{2 * u / calibration_.image_width - 1,
                                 2 * v / calibration_.image_height - 1};
}
std::optional<std::array<double, 2>> CalibratedGroundGeometry::image_projection(
    const Pose& pose, std::array<double, 3> target) const {
    const auto projected = image_projection_with_depth(pose, target);
    if (!projected) {
        return {};
    }
    return std::array<double, 2>{(*projected)[0], (*projected)[1]};
}
std::optional<std::array<double, 3>> CalibratedGroundGeometry::image_projection_with_depth(
    const Pose& pose, std::array<double, 3> target) const {
    if (!std::isfinite(target[0] + target[1] + target[2] + pose.north_m + pose.east_m +
                       pose.down_m + pose.roll_deg + pose.pitch_deg + pose.yaw_deg)) {
        return {};
    }
    const auto& c = calibration_;
    const auto o = body_to_ned(
        pose, {c.camera_offset_body_m[0], c.camera_offset_body_m[1], c.camera_offset_body_m[2]});
    const Vec3 v{target[0] - pose.north_m - o.x, target[1] - pose.east_m - o.y,
                 target[2] - pose.down_m - o.z};
    const auto dot = [](Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; };
    const Vec3 b{dot(v, body_to_ned(pose, {1, 0, 0})), dot(v, body_to_ned(pose, {0, 1, 0})),
                 dot(v, body_to_ned(pose, {0, 0, 1}))};
    const auto& r = c.camera_to_body;
    const Vec3 optical{r[0] * b.x + r[3] * b.y + r[6] * b.z, r[1] * b.x + r[4] * b.y + r[7] * b.z,
                       r[2] * b.x + r[5] * b.y + r[8] * b.z};
    if (optical.z <= .01 || !std::isfinite(optical.x + optical.y + optical.z)) {
        return {};
    }
    const auto raw = distort_normalized(optical.x / optical.z, optical.y / optical.z, c.distortion);
    if (!std::isfinite(raw[0]) || !std::isfinite(raw[1])) {
        return std::nullopt;
    }
    return std::array<double, 3>{2 * (c.cx + c.fx * raw[0]) / c.image_width - 1,
                                 2 * (c.cy + c.fy * raw[1]) / c.image_height - 1, optical.z};
}
}  // namespace follow
