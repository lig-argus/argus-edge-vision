// 테스트용 IFollowControlClient. 보낸 명령을 기록하고, 테스트가 넣은 결과를 FakePoller 를 거쳐
// process_events() 안에서 handler 에 전달한다. 결과를 넣지 않으면 timeout 에 nullopt.
// onboard 의 TrackStartHandler 등 명령 handler 테스트가 추종 제어 대신 쓴다
#ifndef ARGUS_VEHICLE_API_TESTING_FAKE_FOLLOW_CONTROL_CLIENT_HPP_
#define ARGUS_VEHICLE_API_TESTING_FAKE_FOLLOW_CONTROL_CLIENT_HPP_

#include <chrono>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/vehicle_api/follow_control_client.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_testing/fake_follow_control.hpp"
#include "argus/vehicle_api_testing/fake_pending_replies.hpp"

namespace argus::vehicle_api_testing {

class FakeFollowControlClient : public vehicle_api::IFollowControlClient {
public:
    FakeFollowControlClient(core_testing::FakePoller& poller, const core::IClock& clock)
        : pending_(poller, clock) {}

    [[nodiscard]] core::Subscription track_start(const vehicle_api::RequestHeader& header,
                                                 const vehicle_api::TrackTarget& target,
                                                 std::chrono::milliseconds timeout,
                                                 ResultHandler on_result) override {
        commands_.push_back(
            {FollowCommandKind::TrackStart, header, target, std::nullopt, std::nullopt});
        return pending_.add(timeout, std::move(on_result));
    }

    [[nodiscard]] core::Subscription stop(const vehicle_api::RequestHeader& header,
                                          vehicle_api::StopReason reason,
                                          std::chrono::milliseconds timeout,
                                          ResultHandler on_result) override {
        commands_.push_back({FollowCommandKind::Stop, header, std::nullopt, reason, std::nullopt});
        return pending_.add(timeout, std::move(on_result));
    }

    [[nodiscard]] core::Subscription scan_start(const vehicle_api::RequestHeader& header,
                                                const vehicle_api::ScanRequest& scan,
                                                std::chrono::milliseconds timeout,
                                                ResultHandler on_result) override {
        commands_.push_back(
            {FollowCommandKind::ScanStart, header, std::nullopt, std::nullopt, scan});
        return pending_.add(timeout, std::move(on_result));
    }

    // 테스트용: 가장 오래 기다린 명령에 결과를 넣는다 (다음 process_events() 에서 전달).
    // 기다리는 명령이 없으면 false
    bool simulate_reply(vehicle_api::FollowResult result) {
        return pending_.reply(std::move(result));
    }

    const std::vector<FollowCommandRecord>& commands() const { return commands_; }
    std::size_t pending_count() const { return pending_.pending_count(); }

private:
    FakePendingReplies<vehicle_api::FollowResult> pending_;
    std::vector<FollowCommandRecord> commands_;
};

}  // namespace argus::vehicle_api_testing

#endif  // ARGUS_VEHICLE_API_TESTING_FAKE_FOLLOW_CONTROL_CLIENT_HPP_
