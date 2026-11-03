// 테스트용 IModeControl. 받은 요청을 기록하고, 테스트가 지정한 응답을 반환한다.
#ifndef ARGUS_VEHICLE_API_TESTING_FAKE_MODE_CONTROL_HPP_
#define ARGUS_VEHICLE_API_TESTING_FAKE_MODE_CONTROL_HPP_

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "argus/vehicle_api/mode_control.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api_testing {

enum class ModeRequestKind : std::uint8_t {
    Offboard,
    Hold,
    Heading,
};

// FakeModeControl 이 받은 요청 하나
struct ModeRequestRecord {
    ModeRequestKind kind = ModeRequestKind::Hold;
    std::optional<double> heading_rad;  // Heading 요청일 때만
    std::int64_t valid_until_monotonic_ns = 0;
};

class FakeModeControl : public vehicle_api::IModeControl {
public:
    vehicle_api::ModeReply request_offboard(std::int64_t valid_until_monotonic_ns) override {
        return record({ModeRequestKind::Offboard, std::nullopt, valid_until_monotonic_ns});
    }

    vehicle_api::ModeReply request_hold(std::int64_t valid_until_monotonic_ns) override {
        return record({ModeRequestKind::Hold, std::nullopt, valid_until_monotonic_ns});
    }

    vehicle_api::ModeReply request_heading(double heading_rad,
                                           std::int64_t valid_until_monotonic_ns) override {
        return record({ModeRequestKind::Heading, heading_rad, valid_until_monotonic_ns});
    }

    // 다음 요청부터 반환할 응답. 지정하지 않으면 수락
    void set_reply(vehicle_api::ModeReply reply) { reply_ = std::move(reply); }
    const std::vector<ModeRequestRecord>& requests() const { return requests_; }

private:
    vehicle_api::ModeReply record(const ModeRequestRecord& request) {
        requests_.push_back(request);
        return reply_;
    }

    vehicle_api::ModeReply reply_{true, ""};
    std::vector<ModeRequestRecord> requests_;
};

}  // namespace argus::vehicle_api_testing

#endif  // ARGUS_VEHICLE_API_TESTING_FAKE_MODE_CONTROL_HPP_
