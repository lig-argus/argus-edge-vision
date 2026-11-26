#include "argus/perception/geometry/include/pi_ground_projector.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <optional>
#include <stdexcept>

#include "argus/perception/geometry/include/attitude_interpolation.hpp"

namespace follow {
namespace {
template <typename Operation>
bool diagnostic_only(Operation&& operation) noexcept {
    try {
        operation();
        return true;
    } catch (...) {
        return false;
    }
}
}  // namespace

PiGroundProjector::PiGroundProjector(NativeJson geometry, NativeJson measurement,
                                     std::string expected_uid)
    : geometry_(std::move(geometry)),
      measurement_(std::move(measurement)),
      expected_uid_(std::move(expected_uid)) {
    const auto policy = measurement_.value("navigation_policy", "optical_flow");
    if (policy != "optical_flow" && policy != "gps") {
        throw std::invalid_argument("unknown ground projection navigation policy");
    }
    if (geometry_.contains("range_sensor_offset_body_m") &&
        geometry_.value("range_distance_reference", "") != "sensor_origin_raw_slant") {
        throw std::invalid_argument("range lever arm requires raw sensor distance");
    }
    if (expected_uid_.size() != 36) {
        throw std::invalid_argument("expected FC UID required");
    }
}
PiProjectedFrame PiGroundProjector::project(const NativeJson& pixels, const NativeJson& fc,
                                            std::int64_t now) const {
    if (pixels.value("schema", "") != "argus.perception.observations.v2" ||
        pixels.value("clock_domain", "") != "host_monotonic" ||
        pixels.value("coordinate_frame", "") != "camera_pixel") {
        throw std::runtime_error("real pixel v2/host clock required");
    }
    const auto stamp = pixels.at("captured_monotonic_ns").get<std::int64_t>();
    if (stamp <= 0 || now < stamp || now - stamp >= 250000000 ||
        pixels.at("image_width") != geometry_.at("image_width") ||
        pixels.at("image_height") != geometry_.at("image_height")) {
        throw std::runtime_error("original pixel sample expired or calibration size mismatch");
    }
    const auto& state = fc.at("state");
    if (fc.value("schema", "") != "argus.fc.observations.v1" ||
        fc.value("clock_domain", "") != "host_monotonic" ||
        state.value("uid", "") != expected_uid_ || !state.value("identity_ok", false) ||
        !state.value("fresh", false) || !state.value("local_ok", false)) {
        throw std::runtime_error("fresh identity/local position required");
    }
    if (measurement_.value("navigation_policy", "optical_flow") == "gps") {
        if (!fc.at("sensors").value("gps_valid", false) ||
            state.value("flight_environment", "") != "outdoor") {
            throw std::runtime_error(
                "outdoor projection requires verified GPS and outdoor FC profile");
        }
    } else if (!fc.at("sensors").value("flow_valid", false) || state.at("flow_quality").is_null()) {
        throw std::runtime_error("fresh optical flow required");
    }
    NativeJson selected;
    std::optional<std::int64_t> sensor_stamp;
    std::string alignment_clock = "host_receipt";
    const bool sdk_policy =
        measurement_.value("sdk_time_alignment", false) &&
        (state.value("flight_environment", "") == "indoor" ||
         state.value("flight_environment", "") == "outdoor") &&
        (pixels.value("source", "").rfind("ir_sdk:", 0) == 0 || pixels.contains("camera_clock"));
    const auto boot_ms = [](const NativeJson& data) -> std::optional<std::uint32_t> {
        if (!data.contains("time_boot_ms") || !data["time_boot_ms"].is_number_integer()) {
            return std::nullopt;
        }
        const auto value = data["time_boot_ms"].get<std::int64_t>();
        if (value < 0 || value > UINT32_MAX) {
            return std::nullopt;
        }
        return static_cast<std::uint32_t>(value);
    };
    if (sdk_policy && pixels.contains("camera_clock")) {
        if (!pixels["camera_clock"].is_array() || pixels["camera_clock"].size() != 4) {
            throw std::runtime_error("invalid SDK clock metadata shape");
        }
        const auto metadata = pixels.at("camera_clock").get<std::array<std::uint64_t, 4>>();
        if (metadata[0] > UINT32_MAX || metadata[1] > UINT32_MAX || metadata[2] > UINT32_MAX ||
            metadata[3] == 0 || metadata[3] > static_cast<std::uint64_t>(stamp) ||
            stamp - static_cast<std::int64_t>(metadata[3]) >= 250000000) {
            throw std::runtime_error("invalid original SDK clock metadata");
        }
        const auto source = pixels.value("source", "");
        const auto epoch = fc.at("epoch").get<std::uint64_t>();
        if (source != clock_source_ || metadata[0] != camera_serial_ || epoch != clock_epoch_) {
            camera_clock_ = DeviceClockAlignment{};
            fc_clock_ = DeviceClockAlignment{};
            clock_source_ = source;
            camera_serial_ = metadata[0];
            clock_epoch_ = epoch;
        }
        camera_clock_.observe(static_cast<std::uint32_t>(metadata[1]),
                              static_cast<std::int64_t>(metadata[3]));
        std::vector<std::pair<std::int64_t, std::uint32_t>> samples;
        bool all_kinds = true;
        for (const auto* kind : {"ATTITUDE", "LOCAL_POSITION_NED", "DISTANCE_SENSOR"}) {
            bool have = false;
            for (const auto& record : fc.at("history").at(kind)) {
                const auto received = record.at("received_monotonic_ns").get<std::int64_t>();
                if (const auto boot = boot_ms(record.at("data"));
                    boot && received > 0 && now >= received && now - received <= 200000000) {
                    samples.emplace_back(received, *boot);
                    have = true;
                }
            }
            all_kinds = all_kinds && have;
        }
        std::sort(samples.begin(), samples.end());
        for (const auto& sample : samples) {
            fc_clock_.observe(sample.second, sample.first);
        }
        const auto mapped = camera_clock_.map(static_cast<std::uint32_t>(metadata[1]));
        const bool fc_ready = !samples.empty() && fc_clock_.map(samples.back().second).has_value();
        if (mapped && fc_ready && all_kinds) {
            if (*mapped <= 0 || now < *mapped || now - *mapped >= 250000000) {
                throw std::runtime_error("mapped SDK sample expired or future");
            }
            sensor_stamp = mapped;
            alignment_clock = "sdk_fc_boot_receipt_estimate_unverified";
        } else {
            throw std::runtime_error(all_kinds ? "SDK/FC clocks warming up"
                                               : "FC original boot timestamps unavailable");
        }
    } else if (sdk_policy) {
        throw std::runtime_error("SDK frame clock metadata unavailable");
    }
    NativeJson selected_records;
    bool trace_lost = false;
    const auto trace_flag = std::getenv("ARGUS_FLIGHT_RECORD");
    const bool tracing = trace_flag && trace_flag[0] == '1' && trace_flag[1] == '\0';
    std::int64_t maximum_delta = 0;
    for (const auto* kind : {"ATTITUDE", "LOCAL_POSITION_NED", "DISTANCE_SENSOR"}) {
        const auto& history = fc.at("history").at(kind);
        std::int64_t best = std::numeric_limits<std::int64_t>::max();
        for (const auto& record : history) {
            if (std::string(kind) == "DISTANCE_SENSOR") {
                const auto& data = record.at("data");
                if (!data.contains("orientation") || !data["orientation"].is_number_integer() ||
                    data.at("orientation").get<int>() != 25 || !data.contains("current_distance") ||
                    !data["current_distance"].is_number() || !data.contains("min_distance") ||
                    !data["min_distance"].is_number() || !data.contains("max_distance") ||
                    !data["max_distance"].is_number()) {
                    continue;
                }
                const double current = data.at("current_distance").get<double>();
                const double minimum = data.at("min_distance").get<double>();
                const double maximum = data.at("max_distance").get<double>();
                if (!std::isfinite(current) || !std::isfinite(minimum) || !std::isfinite(maximum) ||
                    minimum < 0 || maximum <= minimum || current <= 0 || current < minimum ||
                    current > maximum) {
                    continue;
                }
            }
            const auto received = record.at("received_monotonic_ns").get<std::int64_t>();
            if (received <= 0 || now < received || now - received > 200000000) {
                continue;
            }
            auto measured = received;
            if (sensor_stamp) {
                const auto boot = boot_ms(record.at("data"));
                if (!boot) {
                    continue;
                }
                const auto mapped = fc_clock_.map(*boot);
                if (!mapped || *mapped <= 0 || *mapped > received || now - *mapped > 200000000) {
                    continue;
                }
                measured = *mapped;
            }
            const auto target_stamp = sensor_stamp.value_or(stamp);
            const auto delta =
                measured > target_stamp ? measured - target_stamp : target_stamp - measured;
            if (delta < best) {
                best = delta;
                selected[kind] = record.at("data");
                if (tracing) {
                    if (!diagnostic_only([&] { selected_records[kind] = record; })) {
                        trace_lost = true;
                    }
                }
            }
        }
        if (best > 50000000) {
            throw std::runtime_error(sensor_stamp ? "SDK/FC clock alignment exceeds 50ms"
                                                  : "FC receipt alignment exceeds 50ms");
        }
        maximum_delta = std::max(maximum_delta, best);
    }
    // 2026-11-12 임시 보정: 지면 투영 전용 자세 (기본 꺼짐이면 사용하지 않는다).
    std::optional<Eigen::Vector3d> projection_rpy;
    std::int64_t attitude_extrapolated_ns = 0;  // 2026-11-16 진단
    if (sensor_stamp) {
        std::vector<TimedAttitudeSample> samples;
        const auto& history = fc.at("history").at("ATTITUDE");
        for (std::size_t index = 0; index < history.size(); ++index) {
            const auto& record = history.at(index);
            const auto& data = record.at("data");
            const auto boot = boot_ms(data);
            if (!boot) {
                continue;
            }
            const auto mapped = fc_clock_.map(*boot);
            if (!mapped) {
                continue;
            }
            samples.push_back(
                {*mapped, record.at("received_monotonic_ns").get<std::int64_t>(),
                 attitude_rotation(data.at("roll").get<double>(), data.at("pitch").get<double>(),
                                   data.at("yaw").get<double>()),
                 index});
            // 2026-11-16: 외삽용 몸체 각속도 (세 값이 모두 유한한 수일 때만)
            if (data.contains("rollspeed") && data.contains("pitchspeed") &&
                data.contains("yawspeed") && data.at("rollspeed").is_number() &&
                data.at("pitchspeed").is_number() && data.at("yawspeed").is_number()) {
                const Eigen::Vector3d rate(data.at("rollspeed").get<double>(),
                                           data.at("pitchspeed").get<double>(),
                                           data.at("yawspeed").get<double>());
                if (rate.allFinite()) {
                    samples.back().body_rate = rate;
                }
            }
        }
        std::vector<TimedAttitudeSample> shifted_samples;
        if (attitude_offset_ns_ != 0) {
            shifted_samples = samples;
        }
        // 2026-11-16: 보간 실패(노출 뒤 표본 없음) 때만 외삽한다. 꺼져 있으면 기존과 같다.
        std::vector<TimedAttitudeSample> extrapolation_samples;
        if (max_extrapolation_ns_ > 0) {
            extrapolation_samples = samples;
        }
        auto interpolated = interpolate_attitude(std::move(samples), *sensor_stamp, now);
        if (!interpolated && max_extrapolation_ns_ > 0) {
            interpolated =
                extrapolate_attitude(extrapolation_samples, *sensor_stamp, now,
                                     max_extrapolation_ns_, max_extrapolation_rate_rad_s_);
        }
        if (!interpolated) {
            throw std::runtime_error(
                "FC attitude interpolation requires fresh brackets within 50ms");
        }
        attitude_extrapolated_ns = interpolated->extrapolated_ns;
        if (attitude_offset_ns_ != 0) {
            auto shifted =
                interpolate_attitude(shifted_samples, *sensor_stamp + attitude_offset_ns_, now);
            if (!shifted && max_extrapolation_ns_ > 0) {
                shifted = extrapolate_attitude(
                    std::move(shifted_samples), *sensor_stamp + attitude_offset_ns_, now,
                    max_extrapolation_ns_, max_extrapolation_rate_rad_s_);
            }
            if (!shifted) {
                throw std::runtime_error(
                    "FC attitude interpolation requires fresh brackets within 50ms");
            }
            projection_rpy = shifted->rpy;
        }
        selected.at("ATTITUDE").at("roll") = interpolated->rpy(0);
        selected.at("ATTITUDE").at("pitch") = interpolated->rpy(1);
        selected.at("ATTITUDE").at("yaw") = interpolated->rpy(2);
        maximum_delta = std::max(maximum_delta, interpolated->endpoint_delta_ns);
        if (tracing && !diagnostic_only([&] {
                selected_records["ATTITUDE_INTERPOLATION"] = {
                    {"target_ns", *sensor_stamp},
                    {"extrapolated_ns", interpolated->extrapolated_ns},
                    {"fraction", interpolated->fraction},
                    {"rpy", {interpolated->rpy(0), interpolated->rpy(1), interpolated->rpy(2)}},
                    {"before", history.at(interpolated->before_index)},
                    {"after", history.at(interpolated->after_index)}};
            })) {
            trace_lost = true;
        }
    }
    PiProjectedFrame result;
    result.sample_ns = stamp;
    result.attitude_extrapolated_ns = attitude_extrapolated_ns;
    result.motion_sample_ns = sensor_stamp.value_or(stamp);
    result.alignment_clock = alignment_clock;
    const auto& attitude = selected.at("ATTITUDE");
    result.pose.rpy =
        Eigen::Vector3d(attitude.at("roll").get<double>(), attitude.at("pitch").get<double>(),
                        attitude.at("yaw").get<double>());
    const auto& position = selected.at("LOCAL_POSITION_NED");
    result.pose.position =
        Eigen::Vector3d(position.at("x").get<double>(), position.at("y").get<double>(),
                        position.at("z").get<double>());
    result.pose.epoch = fc.at("epoch");
    result.pose.alignment_error_ms = maximum_delta / 1e6;
    // 지면 투영에만 쓰는 자세. 시각 보정은 SDK 시계 경로에서만 적용한다(수신 시각 경로는 배율만).
    AlignedFCPose projection_pose = result.pose;
    if (projection_rpy) {
        projection_pose.rpy = *projection_rpy;
    }
    projection_pose.rpy[1] *= pitch_scale_;
    const auto& distance = selected.at("DISTANCE_SENSOR");
    const double metres = distance.at("current_distance").get<double>() * .01;
    if (distance.at("orientation") != 25 || !std::isfinite(metres) ||
        metres < distance.at("min_distance").get<double>() * .01 ||
        metres > distance.at("max_distance").get<double>() * .01) {
        throw std::runtime_error("valid downward distance sensor required");
    }
    const auto& rpy = result.pose.rpy;
    Eigen::Matrix3d body = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
                               .toRotationMatrix();
    const double height = metres * body(2, 2);
    if (!result.pose.position.allFinite() || !rpy.allFinite() || body(2, 2) < .5 || height <= .02 ||
        height >= 100) {
        throw std::runtime_error("invalid FC pose or corrected range");
    }
    auto measurement = measurement_;
    std::array<double, 9> mount{};
    const auto& array = geometry_.at("camera_to_body");
    for (unsigned i = 0; i < 9; ++i) {
        mount[i] = array.at(i / 3).at(i % 3);
    }
    measurement["camera_to_body"] = mount;
    measurement["camera_offset_body_m"] = geometry_.at("camera_offset_body_m");
    const auto range_offset =
        geometry_.value("range_sensor_offset_body_m", std::array<double, 3>{0., 0., 0.});
    Eigen::Vector3d range_position(range_offset[0], range_offset[1], range_offset[2]);
    if (!range_position.allFinite()) {
        throw std::runtime_error("invalid range sensor offset");
    }
    measurement["ground_plane_down_m"] =
        result.pose.position[2] + (body * range_position)[2] + height;
    // 2026-11-19 PAMPC #5: 제어기 계획기가 같은 기준을 쓰도록 실제 투영 값을 함께 남긴다.
    result.ground_plane_down_m = measurement["ground_plane_down_m"].get<double>();
    result.projection_rpy = projection_pose.rpy;
    measurement["ground_contact_anchor"] = true;
    measurement["ground_roof_anchor"] = false;
    measurement["box_anchor_x"] = .5;
    measurement["box_anchor_y"] = 1.;
    measurement["target_anchor_height_above_ground_m"] = 0.;
    FrameTransform mapper(
        {geometry_.at("fx"), geometry_.at("fy"), geometry_.at("cx"), geometry_.at("cy")},
        measurement);
    if (tracing) {
        if (!diagnostic_only([&] {
                result.diagnostic = {
                    {"selected_fc_samples", selected_records},
                    {"effective_measurement", measurement},
                    {"camera_geometry", geometry_},
                    {"range_sensor_raw_m", metres},
                    {"range_corrected_vertical_m", height},
                    {"fc_epoch", result.pose.epoch},
                    {"alignment_error_ms", result.pose.alignment_error_ms},
                    {"exposure_alignment_verified", false},
                    {"attitude_correction",
                     {{"pitch_scale", pitch_scale_},
                      {"attitude_offset_ms", static_cast<double>(attitude_offset_ns_) / 1e6},
                      {"projection_rpy",
                       {projection_pose.rpy[0], projection_pose.rpy[1], projection_pose.rpy[2]}}}},
                    {"rejected_detections", NativeJson::array()}};
            })) {
            trace_lost = true;
        }
    }
    const auto& detections = pixels.at("detections");
    if (!detections.is_array() || detections.size() > 512) {
        throw std::runtime_error("invalid detection list");
    }
    for (const auto& detection : detections) {
        std::array<double, 5> box{detection.at("x1"), detection.at("y1"), detection.at("x2"),
                                  detection.at("y2"), detection.at("confidence")};
        if (std::any_of(box.begin(), box.end(), [](double v) { return !std::isfinite(v); }) ||
            box[4] <= 0 || box[4] > 1 || box[0] < 0 || box[1] < 0 || box[2] <= box[0] ||
            box[3] <= box[1] || box[2] > pixels.at("image_width").get<double>() ||
            box[3] > pixels.at("image_height").get<double>()) {
            throw std::runtime_error("invalid original detection bounds or score");
        }
        const auto class_id = detection.at("class_id").get<std::int32_t>();
        const auto label = detection.at("label").get<std::string>();
        if (class_id < 0 || label.empty() || label.size() > 128) {
            throw std::runtime_error("invalid class/label");
        }
        try {
            result.detections.push_back({class_id, label, mapper.measure(box, projection_pose)});
            result.detections.back().thermal = detection.value("thermal", NativeJson{});
        } catch (const std::runtime_error&
                     error) { /* Invalid geometry is not a replacement detection. */
            if (tracing) {
                if (!diagnostic_only([&] {
                        result.diagnostic["rejected_detections"].push_back(
                            {{"class_id", class_id},
                             {"label", label},
                             {"box", box},
                             {"reason", error.what()}});
                    })) {
                    trace_lost = true;
                }
            }
        }
    }
    if (tracing) {
        if (!diagnostic_only([&] {
                result.diagnostic["alignment_clock"] = alignment_clock;
                result.diagnostic["motion_sample_ns"] = result.motion_sample_ns;
                result.diagnostic["exposure_alignment_verified"] = false;
                if (pixels.contains("camera_clock")) {
                    result.diagnostic["camera_clock"] = pixels["camera_clock"];
                }
            })) {
            trace_lost = true;
        }
    }
    result.diagnostic_lost = trace_lost;
    return result;
}
}  // namespace follow
