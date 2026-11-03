// 테스트용 IFlightModeControl. 받은 요청을 기록하고, 테스트가 지정한 응답을 반환한다.
#ifndef ARGUS_VEHICLE_API_TESTING_FAKE_FLIGHT_MODE_CONTROL_HPP_
#define ARGUS_VEHICLE_API_TESTING_FAKE_FLIGHT_MODE_CONTROL_HPP_

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "argus/vehicle_api/flight_mode_control.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api_testing {

enum class FlightModeRequestKind : std::uint8_t {
    Offboard,
    Hold,
    Heading,
};

// FakeFlightModeControl 이 받은 요청 하나
struct FlightModeRequestRecord {
    FlightModeRequestKind kind = FlightModeRequestKind::Hold;
    vehicle_api::RequestHeader header;
    std::optional<double> heading_rad;  // Heading 요청일 때만
};

class FakeFlightModeControl : public vehicle_api::IFlightModeControl {
public:
    vehicle_api::FlightModeReply request_offboard(
        const vehicle_api::RequestHeader& header) override {
        return record({FlightModeRequestKind::Offboard, header, std::nullopt});
    }
    vehicle_api::FlightModeReply request_hold(const vehicle_api::RequestHeader& header) override {
        return record({FlightModeRequestKind::Hold, header, std::nullopt});
    }
    vehicle_api::FlightModeReply request_heading(const vehicle_api::RequestHeader& header,
                                                 double heading_rad) override {
        return record({FlightModeRequestKind::Heading, header, heading_rad});
    }

    // 다음 요청부터 반환할 응답(accepted, reason). id 는 요청의 RequestId 로 채운다.
    // 지정하지 않으면 수락
    void set_reply(vehicle_api::FlightModeReply reply) { reply_ = std::move(reply); }

    const std::vector<FlightModeRequestRecord>& requests() const { return requests_; }

private:
    vehicle_api::FlightModeReply record(const FlightModeRequestRecord& request) {
        requests_.push_back(request);
        vehicle_api::FlightModeReply reply = reply_;
        reply.id = request.header.id;
        return reply;
    }

    static vehicle_api::FlightModeReply accepted_reply() {
        vehicle_api::FlightModeReply reply;
        reply.accepted = true;
        return reply;
    }

    vehicle_api::FlightModeReply reply_ = accepted_reply();
    std::vector<FlightModeRequestRecord> requests_;
};

}  // namespace argus::vehicle_api_testing

#endif  // ARGUS_VEHICLE_API_TESTING_FAKE_FLIGHT_MODE_CONTROL_HPP_
