#include "argus/video_sender/video_bus_input.hpp"

#include <utility>

namespace argus::video_sender {

std::unique_ptr<VideoBusInput> VideoBusInput::create(bus::IBus& bus, FrameCollector& collector) {
    auto frames = BusFrameSource::create(bus);
    auto observations = perception_api_bus::create_observations_subscriber(bus);
    auto tracks = perception_api_bus::create_tracks_subscriber(bus);
    if (!frames || !observations || !tracks) {
        return nullptr;
    }
    return std::unique_ptr<VideoBusInput>(new VideoBusInput(
        std::move(frames), std::move(observations), std::move(tracks), collector));
}

VideoBusInput::VideoBusInput(
    std::unique_ptr<BusFrameSource> frames,
    std::unique_ptr<perception_api_bus::BusObservationsSubscriber> observations,
    std::unique_ptr<perception_api_bus::BusTracksSubscriber> tracks, FrameCollector& collector)
    : frames_(std::move(frames)),
      observations_(std::move(observations)),
      tracks_(std::move(tracks)),
      frame_subscription_(frames_->subscribe(
          [&collector](const perception_api::Frame& frame) { collector.on_frame(frame); })),
      observations_subscription_(
          observations_->subscribe([&collector](const perception_api::Observations& observations,
                                                const bus::MessageInfo& /*info*/) {
              collector.on_observations(observations);
          })),
      tracks_subscription_(tracks_->subscribe(
          [&collector](const perception_api::Tracks& tracks, const bus::MessageInfo& /*info*/) {
              collector.on_tracks(tracks);
          })) {}

}  // namespace argus::video_sender
