#include "argus/vehicle_api_bus/bus_follow_control_server.hpp"

#include <optional>
#include <utility>
#include <variant>

#include "argus/bus/message.hpp"
#include "argus/protocol/topics.hpp"
#include "argus/vehicle_api/types.hpp"
#include "detail.hpp"
#include "follow_command_codec.hpp"

namespace argus::vehicle_api_bus {
namespace {

// std::variant 의 타입마다 IFollowControl 의 대응하는 함수를 부른다
struct ActionCaller {
    vehicle_api::IFollowControl& control;
    const vehicle_api::RequestHeader& header;

    vehicle_api::FollowResult operator()(const vehicle_api::TrackTarget& target) const {
        return control.track_start(header, target);
    }
    vehicle_api::FollowResult operator()(vehicle_api::StopReason reason) const {
        return control.stop(header, reason);
    }
    vehicle_api::FollowResult operator()(const vehicle_api::ScanRequest& scan) const {
        return control.scan_start(header, scan);
    }
};

bus::Bytes handle(vehicle_api::IFollowControl& control, const bus::BusMessage& message) {
    // 요청에는 바이너리가 없다
    const std::optional<FollowCommandRequest> request =
        message.binary.empty() ? decode_follow_command(message.body) : std::nullopt;
    if (!request) {
        detail::log_error("follow/command: 형식이 맞지 않는 요청 거부");
        vehicle_api::FollowResult rejected;
        rejected.outcome = vehicle_api::FollowOutcome::Rejected;
        rejected.reason = "malformed request";
        return encode_follow_result(rejected);
    }
    return encode_follow_result(
        std::visit(ActionCaller{control, request->header}, request->action));
}

}  // namespace

std::unique_ptr<BusFollowControlServer> BusFollowControlServer::create(
    bus::IBus& bus, vehicle_api::IFollowControl& control) {
    auto replier = bus.create_replier(protocol::kFollowCommand);
    if (!replier) {
        return nullptr;
    }
    core::Subscription serving = replier->serve(
        [&control](const bus::BusMessage& message) { return handle(control, message); });
    return std::unique_ptr<BusFollowControlServer>(
        new BusFollowControlServer(std::move(replier), std::move(serving)));
}

BusFollowControlServer::BusFollowControlServer(std::unique_ptr<bus::IReplier> replier,
                                               core::Subscription serving)
    : replier_(std::move(replier)), serving_(std::move(serving)) {}

}  // namespace argus::vehicle_api_bus
