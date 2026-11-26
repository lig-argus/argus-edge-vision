// 카메라마다 최신 추적 결과(`tracking/tracks`)를 보관하고 조회
// 규약: docs/spec/icd.md 4.2 ④, docs/spec/process_interfaces.md 5.1.1, 5.2 (label 보완)
// 사용: TrackStartHandler 가 ILatestTracks 로 조회한다. main 이 LatestTracks 를 생성
#ifndef ARGUS_ONBOARD_LATEST_TRACKS_HPP_
#define ARGUS_ONBOARD_LATEST_TRACKS_HPP_

#include <map>
#include <memory>
#include <optional>
#include <string>

#include "argus/bus/bus.hpp"
#include "argus/core/subscription.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/perception_api_bus/subscribers.hpp"

namespace argus::mavlink_gateway {

class ILatestTracks {
public:
    ILatestTracks() = default;
    virtual ~ILatestTracks() = default;
    ILatestTracks(const ILatestTracks&) = delete;
    ILatestTracks& operator=(const ILatestTracks&) = delete;
    ILatestTracks(ILatestTracks&&) = delete;
    ILatestTracks& operator=(ILatestTracks&&) = delete;

    // source(카메라 식별 문자열)의 가장 최근 추적 결과. 받지 못했으면 nullopt.
    // 최신인지는 호출자가 sample_monotonic_ns 로 판단한다
    virtual std::optional<perception_api::Tracks> latest(const std::string& source) const = 0;
};

class LatestTracks : public ILatestTracks {
public:
    // bus 가 구독자를 만들지 못하면 nullptr (이유는 bus 구현이 로그에 남긴다)
    static std::unique_ptr<LatestTracks> create(bus::IBus& bus);

    std::optional<perception_api::Tracks> latest(const std::string& source) const override;

private:
    using BySource = std::map<std::string, perception_api::Tracks>;

    LatestTracks(std::shared_ptr<BySource> by_source,
                 std::unique_ptr<perception_api_bus::BusTracksSubscriber> subscriber,
                 core::Subscription subscription);

    // bus 의 handler 는 이 객체를 소유하지 않고 weak_ptr 로 참조한다
    std::shared_ptr<BySource> by_source_;
    std::unique_ptr<perception_api_bus::BusTracksSubscriber> subscriber_;
    core::Subscription subscription_;  // 이 객체가 사라지면 해제된다
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_LATEST_TRACKS_HPP_
