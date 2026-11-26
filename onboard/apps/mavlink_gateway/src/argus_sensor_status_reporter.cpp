#include "argus/mavlink_gateway/argus_sensor_status_reporter.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "argus/mavlink_gateway/sensor_status_policy.hpp"
#include "gcs_message_encoding.hpp"

namespace argus::mavlink_gateway {

ArgusSensorStatusReporter::ArgusSensorStatusReporter(const ArgusSensorStatusReporterInputs& inputs)
    : inputs_(inputs), last_sent_(inputs.sensors.entries().size()) {}

void ArgusSensorStatusReporter::tick() {
    const std::int64_t now_ns = inputs_.clock.now_monotonic_ns();
    const std::int64_t period_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(kSensorStatusPeriod).count();
    const auto& entries = inputs_.sensors.entries();
    for (std::size_t index = 0; index < entries.size(); ++index) {
        const SensorSource& sensor = entries[index];
        SensorObservation observation;
        observation.now_ns = now_ns;
        observation.sensor = sensor;
        observation.vision = inputs_.vision_status.latest(sensor.source);
        observation.tracking = inputs_.tracking_status.latest(sensor.source);
        const mavlink_argus_sensor_status_t status = compose_sensor_status(observation);

        std::optional<Sent>& last = last_sent_[index];
        const bool due =
            !last.has_value() || last->flags != status.flags || now_ns - last->sent_ns >= period_ns;
        if (!due) {
            continue;
        }
        inputs_.link.broadcast(detail::encode_argus_sensor_status(status));
        last = Sent{status.flags, now_ns};
    }
}

}  // namespace argus::mavlink_gateway
