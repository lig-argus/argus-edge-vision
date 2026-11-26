// ARGUS_SENSOR_STATUS 송신
// 규약: docs/spec/icd.md 3.1, 6.3, docs/design/uml/class_mavlink_gateway.md 3절
// 사용: mavlink_gateway 의 main 이 하나를 생성하여 tick() 을 kSensorStatusCheckPeriod 주기 작업으로
//      등록
#ifndef ARGUS_ONBOARD_ARGUS_SENSOR_STATUS_REPORTER_HPP_
#define ARGUS_ONBOARD_ARGUS_SENSOR_STATUS_REPORTER_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

#include "argus/core/clock.hpp"
#include "argus/mavlink_gateway/gcs_link.hpp"
#include "argus/mavlink_gateway/sensor_source_map.hpp"
#include "argus/perception_api/tracking_status_source.hpp"
#include "argus/perception_api/vision_status_source.hpp"

namespace argus::mavlink_gateway {

// 센서마다의 ARGUS_SENSOR_STATUS 송신 주기 (icd 3.1)
inline constexpr std::chrono::milliseconds kSensorStatusPeriod{1000};

// tick() 의 호출 주기. flags 가 바뀐 것을 알아차리는 지연이며, 바뀌면 바로 보낸다
inline constexpr std::chrono::milliseconds kSensorStatusCheckPeriod{100};

// 입력. 모두 ArgusSensorStatusReporter 보다 오래 유지되어야 한다
struct ArgusSensorStatusReporterInputs {
    IGcsLink& link;  // 모든 통제기에 broadcast
    const core::IClock& clock;
    const perception_api::ITrackingStatusSource& tracking_status;
    const perception_api::IVisionStatusSource& vision_status;
    const SensorSourceMap& sensors;
};

// sensors 의 센서마다 compose_sensor_status() 로 값을 정해, kSensorStatusPeriod 마다와 그 센서의
// flags 가 바뀔 때 송신한다. 값을 받지 못한 센서도 flags 0 으로 송신한다 (통제기가 센서가 있음을
// 알도록)
class ArgusSensorStatusReporter {
public:
    explicit ArgusSensorStatusReporter(const ArgusSensorStatusReporterInputs& inputs);

    // kSensorStatusCheckPeriod 마다 호출한다
    void tick();

private:
    struct Sent {
        std::uint8_t flags = 0;
        std::int64_t sent_ns = 0;
    };

    ArgusSensorStatusReporterInputs inputs_;
    std::vector<std::optional<Sent>> last_sent_;  // sensors.entries() 와 같은 순서
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_ARGUS_SENSOR_STATUS_REPORTER_HPP_
