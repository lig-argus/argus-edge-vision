#include "argus/mavlink_gateway/sensor_status_policy.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>

namespace argus::mavlink_gateway {
namespace {

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

std::uint16_t to_uint16(std::uint32_t value) {
    return static_cast<std::uint16_t>(
        std::min<std::uint32_t>(value, std::numeric_limits<std::uint16_t>::max()));
}

bool fresh(std::int64_t arrived_ns, std::int64_t now_ns) {
    return now_ns - arrived_ns <= kSensorStatusTimeoutNs;
}

}  // namespace

mavlink_argus_sensor_status_t compose_sensor_status(const SensorObservation& observation) {
    mavlink_argus_sensor_status_t status{};
    status.sensor_id = static_cast<std::uint8_t>(observation.sensor.sensor);
    status.infer_fps = kNan;

    std::uint32_t flags = 0;
    if (observation.vision.has_value() &&
        fresh(observation.vision->arrived_monotonic_ns, observation.now_ns)) {
        const perception_api::VisionStatus& vision = observation.vision->status;
        if (vision.camera_ok) {
            flags |= ARGUS_SENSOR_FLAGS_CAMERA_OK;
        }
        if (vision.inference_ok) {
            flags |= ARGUS_SENSOR_FLAGS_INFERENCE_OK;
        }
        status.infer_fps = static_cast<float>(vision.infer_fps);
        status.model_ver = to_uint16(vision.model_ver);
        status.img_w = to_uint16(vision.image_width);
        status.img_h = to_uint16(vision.image_height);
    }
    if (observation.tracking.has_value() &&
        fresh(observation.tracking->arrived_monotonic_ns, observation.now_ns)) {
        flags |= ARGUS_SENSOR_FLAGS_TRACKING;
        if (observation.sensor.followable) {
            flags |= ARGUS_SENSOR_FLAGS_FOLLOWABLE;
        }
    }
    status.flags = static_cast<std::uint8_t>(flags);
    return status;
}

}  // namespace argus::mavlink_gateway
