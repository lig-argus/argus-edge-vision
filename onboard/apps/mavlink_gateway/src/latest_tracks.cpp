#include "argus/mavlink_gateway/latest_tracks.hpp"

#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace argus::mavlink_gateway {

std::unique_ptr<LatestTracks> LatestTracks::create(bus::IBus& bus) {
    std::unique_ptr<perception_api_bus::BusTracksSubscriber> subscriber =
        perception_api_bus::create_tracks_subscriber(bus);
    if (!subscriber) {
        return nullptr;
    }
    auto by_source = std::make_shared<BySource>();
    const std::weak_ptr<BySource> weak = by_source;
    core::Subscription subscription = subscriber->subscribe(
        [weak](const perception_api::Tracks& tracks, const bus::MessageInfo& /*info*/) {
            if (const auto locked = weak.lock()) {
                (*locked)[tracks.source] = tracks;
            }
        });
    return std::unique_ptr<LatestTracks>(
        new LatestTracks(std::move(by_source), std::move(subscriber), std::move(subscription)));
}

LatestTracks::LatestTracks(std::shared_ptr<BySource> by_source,
                           std::unique_ptr<perception_api_bus::BusTracksSubscriber> subscriber,
                           core::Subscription subscription)
    : by_source_(std::move(by_source)),
      subscriber_(std::move(subscriber)),
      subscription_(std::move(subscription)) {}

std::optional<perception_api::Tracks> LatestTracks::latest(const std::string& source) const {
    const auto found = by_source_->find(source);
    if (found == by_source_->end()) {
        return std::nullopt;
    }
    return found->second;
}

}  // namespace argus::mavlink_gateway
