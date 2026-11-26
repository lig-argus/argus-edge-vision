// ARGUS_SENSOR_STATUS 한 센서의 값 구성. 입력 구조체만 받고 MAVLink 송수신, 시계, bus 를 모른다
// 규약: docs/spec/icd.md 3.1, 6.3, docs/spec/process_interfaces.md 5.1.3, 5.1.7
// 사용: ArgusSensorStatusReporter 가 센서마다 입력을 모아 부르고 결과를 송신한다
#ifndef ARGUS_ONBOARD_SENSOR_STATUS_POLICY_HPP_
#define ARGUS_ONBOARD_SENSOR_STATUS_POLICY_HPP_

#include <cstdint>
#include <optional>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/sensor_source_map.hpp"
#include "argus/perception_api/types.hpp"

namespace argus::mavlink_gateway {

// vision/status, tracking/status 를 이 시간 안에 받아야 그 값을 쓴다 (1 Hz 토픽의 두절 기준,
// follow/status 와 같음)
inline constexpr std::int64_t kSensorStatusTimeoutNs = 3'000'000'000;

struct SensorObservation {
    std::int64_t now_ns = 0;
    SensorSource sensor;
    std::optional<perception_api::ReceivedVisionStatus> vision;
    std::optional<perception_api::ReceivedTrackingStatus> tracking;
};

// - CAMERA_OK, INFERENCE_OK: vision/status 를 kSensorStatusTimeoutNs 안에 받았고 그 값이 true
// - TRACKING: tracking/status 를 kSensorStatusTimeoutNs 안에 받음
// - FOLLOWABLE: TRACKING 이고 sensor.followable
// - infer_fps, model_ver, img_w, img_h: kSensorStatusTimeoutNs 안에 받은 vision/status 의 값.
//   없으면 NaN, 0. uint16 을 넘는 값은 65535 로 한다
mavlink_argus_sensor_status_t compose_sensor_status(const SensorObservation& observation);

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_SENSOR_STATUS_POLICY_HPP_
