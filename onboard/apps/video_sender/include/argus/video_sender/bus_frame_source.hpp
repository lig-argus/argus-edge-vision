// bus 의 vision/frame/gray8 을 구독해 프레임을 공급하는 IFrameSource 구현
// 규약: docs/spec/process_interfaces.md 5.1 vision/frame/gray8, 5.1.5
// 사용: VideoBusInput 이 하나를 만들어 FrameCollector 에 연결
#ifndef ARGUS_VIDEO_SENDER_BUS_FRAME_SOURCE_HPP_
#define ARGUS_VIDEO_SENDER_BUS_FRAME_SOURCE_HPP_

#include <memory>

#include "argus/bus/bus.hpp"
#include "argus/core/handler_list.hpp"
#include "argus/core/subscription.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/perception_api_bus/subscribers.hpp"
#include "argus/video_sender/frame_source.hpp"

namespace argus::video_sender {

class BusFrameSource : public IFrameSource {
public:
    // bus 가 구독자를 만들지 못하면 nullptr 를 반환한다 (이유는 bus 구현이 로그에 남긴다)
    static std::unique_ptr<BusFrameSource> create(bus::IBus& bus);

    // handler 가 비어 있으면 등록하지 않고 빈 Subscription 을 반환한다
    [[nodiscard]] core::Subscription subscribe(FrameHandler handler) override;

private:
    // 등록한 handler 목록. bus 의 handler 와 반환한 Subscription 이 이 객체보다 오래 유지되어도
    // 안전하도록 공유 객체에 두고, bus 의 handler 는 weak_ptr 로 참조한다
    using Handlers = core::HandlerList<const perception_api::Frame&>;

    BusFrameSource(std::unique_ptr<perception_api_bus::BusFrameSubscriber> subscriber,
                   std::shared_ptr<Handlers> handlers, core::Subscription subscription);

    std::unique_ptr<perception_api_bus::BusFrameSubscriber> subscriber_;
    std::shared_ptr<Handlers> handlers_;
    core::Subscription subscription_;  // bus 구독. 이 객체가 사라지면 해제된다
};

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_BUS_FRAME_SOURCE_HPP_
