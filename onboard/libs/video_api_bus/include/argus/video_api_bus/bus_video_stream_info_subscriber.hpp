// video/stream_info 토픽을 구독하는 IVideoStreamInfoSource 구현
// 규약: docs/spec/process_interfaces.md 5.1.6. 조회 규칙은
// argus/video_api/video_stream_info_source.hpp
// 사용: mavlink_gateway 의 main() 이 만들어 IVideoStreamInfoSource 로 넘긴다
#ifndef ARGUS_VIDEO_API_BUS_BUS_VIDEO_STREAM_INFO_SUBSCRIBER_HPP_
#define ARGUS_VIDEO_API_BUS_BUS_VIDEO_STREAM_INFO_SUBSCRIBER_HPP_

#include <cstdint>
#include <memory>
#include <optional>

#include "argus/bus/bus.hpp"
#include "argus/bus/typed.hpp"
#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/video_api/types.hpp"
#include "argus/video_api/video_stream_info_source.hpp"

namespace argus::video_api_bus {

class BusVideoStreamInfoSubscriber : public video_api::IVideoStreamInfoSource {
public:
    // clock 은 받은 시각을 얻는 데 쓴다. 이 객체보다 오래 살아야 한다.
    // bus 가 구독자를 만들지 못하면 nullptr 를 반환한다 (이유는 bus 구현이 로그에 남긴다)
    static std::unique_ptr<BusVideoStreamInfoSubscriber> create(bus::IBus& bus,
                                                                const core::IClock& clock);

    std::optional<video_api::ReceivedVideoStreamInfo> latest() const override;

private:
    // bus 의 handler 는 이 객체를 소유하지 않고 weak_ptr 로 받은 값을 참조한다. 그래서 이 객체가
    // 사라진 뒤 handler 가 불려도 안전하다
    struct State {
        std::optional<video_api::ReceivedVideoStreamInfo> latest;
    };

    BusVideoStreamInfoSubscriber(
        std::shared_ptr<State> state,
        std::unique_ptr<bus::Subscriber<video_api::VideoStreamInfo>> subscriber,
        core::Subscription subscription);

    std::shared_ptr<State> state_;
    std::unique_ptr<bus::Subscriber<video_api::VideoStreamInfo>> subscriber_;
    core::Subscription subscription_;  // 이 객체가 사라지면 해제된다
};

}  // namespace argus::video_api_bus

#endif  // ARGUS_VIDEO_API_BUS_BUS_VIDEO_STREAM_INFO_SUBSCRIBER_HPP_
