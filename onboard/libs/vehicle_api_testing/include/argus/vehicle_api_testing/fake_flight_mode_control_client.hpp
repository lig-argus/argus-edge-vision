// 테스트용 IFlightModeControlClient. 보낸 요청을 기록하고, 테스트가 넣은 응답을 FakePoller 를 거쳐
// process_events() 안에서 handler 에 전달한다. 응답을 넣지 않으면 timeout 에 nullopt.
// 추종 제어(FollowController) 테스트가 onboard 대신 쓴다
#ifndef ARGUS_VEHICLE_API_TESTING_FAKE_FLIGHT_MODE_CONTROL_CLIENT_HPP_
#define ARGUS_VEHICLE_API_TESTING_FAKE_FLIGHT_MODE_CONTROL_CLIENT_HPP_

#include <chrono>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/vehicle_api/flight_mode_control_client.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_testing/fake_flight_mode_control.hpp"
#include "argus/vehicle_api_testing/fake_pending_replies.hpp"

namespace argus::vehicle_api_testing {

class FakeFlightModeControlClient : public vehicle_api::IFlightModeControlClient {
public:
    FakeFlightModeControlClient(core_testing::FakePoller& poller, const core::IClock& clock)
        : pending_(poller, clock) {}

    [[nodiscard]] core::Subscription request_offboard(const vehicle_api::RequestHeader& header,
                                                      std::chrono::milliseconds timeout,
                                                      ReplyHandler on_reply) override {
        requests_.push_back({FlightModeRequestKind::Offboard, header, std::nullopt});
        return pending_.add(timeout, std::move(on_reply));
    }

    [[nodiscard]] core::Subscription request_hold(const vehicle_api::RequestHeader& header,
                                                  std::chrono::milliseconds timeout,
                                                  ReplyHandler on_reply) override {
        requests_.push_back({FlightModeRequestKind::Hold, header, std::nullopt});
        return pending_.add(timeout, std::move(on_reply));
    }

    [[nodiscard]] core::Subscription request_heading(const vehicle_api::RequestHeader& header,
                                                     double heading_rad,
                                                     std::chrono::milliseconds timeout,
                                                     ReplyHandler on_reply) override {
        requests_.push_back({FlightModeRequestKind::Heading, header, heading_rad});
        return pending_.add(timeout, std::move(on_reply));
    }

    // 테스트용: 가장 오래 기다린 요청에 응답을 넣는다 (다음 process_events() 에서 전달).
    // 기다리는 요청이 없으면 false
    bool reply(vehicle_api::FlightModeReply reply) { return pending_.reply(std::move(reply)); }

    const std::vector<FlightModeRequestRecord>& requests() const { return requests_; }
    std::size_t pending_count() const { return pending_.pending_count(); }

private:
    FakePendingReplies<vehicle_api::FlightModeReply> pending_;
    std::vector<FlightModeRequestRecord> requests_;
};

}  // namespace argus::vehicle_api_testing

#endif  // ARGUS_VEHICLE_API_TESTING_FAKE_FLIGHT_MODE_CONTROL_CLIENT_HPP_
