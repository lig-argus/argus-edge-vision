#include "argus/vehicle_api_bus/bus_follow_control_server.hpp"

#include <utility>
#include <variant>

#include "argus/protocol/topics.hpp"

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

}  // namespace

std::unique_ptr<BusFollowControlServer> BusFollowControlServer::create(
    bus::IBus& bus, vehicle_api::IFollowControl& control) {
    auto replier = bus::make_replier<FollowCommandRequest, vehicle_api::FollowResult>(
        bus, protocol::kFollowCommand, kFollowCommandType, kFollowResultType);
    if (!replier) {
        return nullptr;
    }
    core::Subscription serving = replier->serve([&control](const FollowCommandRequest& request) {
        return std::visit(ActionCaller{control, request.header}, request.action);
    });
    return std::unique_ptr<BusFollowControlServer>(
        new BusFollowControlServer(std::move(replier), std::move(serving)));
}

BusFollowControlServer::BusFollowControlServer(std::unique_ptr<Replier> replier,
                                               core::Subscription serving)
    : replier_(std::move(replier)), serving_(std::move(serving)) {}

}  // namespace argus::vehicle_api_bus
