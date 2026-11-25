#include "argus/perception_api_bus/subscribers.hpp"

#include "argus/perception_api_bus/message_types.hpp"
#include "argus/protocol/topics.hpp"

namespace argus::perception_api_bus {

std::unique_ptr<BusFrameSubscriber> create_frame_subscriber(bus::IBus& bus) {
    return bus::make_subscriber<perception_api::Frame>(bus, protocol::kVisionFrameGray8,
                                                       kVisionFrameGray8Type,
                                                       protocol::kVisionFrameGray8QueueDepth);
}

std::unique_ptr<BusObservationsSubscriber> create_observations_subscriber(bus::IBus& bus) {
    return bus::make_subscriber<perception_api::Observations>(
        bus, protocol::kPerceptionObservations, kPerceptionObservationsType,
        protocol::kPerceptionObservationsQueueDepth);
}

std::unique_ptr<BusTracksSubscriber> create_tracks_subscriber(bus::IBus& bus) {
    return bus::make_subscriber<perception_api::Tracks>(
        bus, protocol::kTrackingTracks, kTrackingTracksType, protocol::kTrackingTracksQueueDepth);
}

}  // namespace argus::perception_api_bus
