#include "argus/video_api_bus/bus_video_stream_info_publisher.hpp"

#include <utility>

#include "argus/protocol/topics.hpp"
#include "argus/video_api_bus/message_types.hpp"
#include "detail.hpp"

namespace argus::video_api_bus {

std::unique_ptr<BusVideoStreamInfoPublisher> BusVideoStreamInfoPublisher::create(bus::IBus& bus) {
    auto publisher = bus::make_publisher<video_api::VideoStreamInfo>(
        bus, protocol::kVideoStreamInfo, kVideoStreamInfoType,
        protocol::kVideoStreamInfoQueueDepth);
    if (!publisher) {
        return nullptr;
    }
    return std::unique_ptr<BusVideoStreamInfoPublisher>(
        new BusVideoStreamInfoPublisher(std::move(publisher)));
}

BusVideoStreamInfoPublisher::BusVideoStreamInfoPublisher(
    std::unique_ptr<bus::Publisher<video_api::VideoStreamInfo>> publisher)
    : publisher_(std::move(publisher)) {}

bool BusVideoStreamInfoPublisher::publish(const video_api::VideoStreamInfo& info) {
    const bool published = publisher_->publish(info);
    detail::record_publish_result(publisher_->topic(), published, failing_);
    return published;
}

}  // namespace argus::video_api_bus
