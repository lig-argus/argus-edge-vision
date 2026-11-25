// bus 의 프레임, 탐지 결과, 추적 결과를 구독해 FrameCollector 에 넣는다
// 규약: docs/spec/process_interfaces.md 5.1, 5.3, docs/design/uml/class_video.md 1절
// 사용: 영상 송신부의 main 이 통신 구현(ZmqBus)을 만든 뒤 하나를 생성
//
// 통신 구현을 알지 못하고 bus::IBus 로만 구독한다. 구독한 메시지는 core::IPoller::process_events()
// 안에서 FrameCollector 의 on_frame(), on_observations(), on_tracks() 로 전달된다
#ifndef ARGUS_VIDEO_SENDER_VIDEO_BUS_INPUT_HPP_
#define ARGUS_VIDEO_SENDER_VIDEO_BUS_INPUT_HPP_

#include <memory>

#include "argus/bus/bus.hpp"
#include "argus/core/subscription.hpp"
#include "argus/perception_api_bus/subscribers.hpp"
#include "argus/video_sender/bus_frame_source.hpp"
#include "argus/video_sender/frame_collector.hpp"

namespace argus::video_sender {

class VideoBusInput {
public:
    // bus 와 collector 는 반환된 객체보다 오래 유지되어야 한다.
    // bus 가 구독자 셋 중 하나라도 만들지 못하면 nullptr 를 반환한다 (이유는 bus 구현이 로그에
    // 남긴다)
    static std::unique_ptr<VideoBusInput> create(bus::IBus& bus, FrameCollector& collector);

    VideoBusInput(const VideoBusInput&) = delete;
    VideoBusInput& operator=(const VideoBusInput&) = delete;
    VideoBusInput(VideoBusInput&&) = delete;
    VideoBusInput& operator=(VideoBusInput&&) = delete;
    ~VideoBusInput() = default;

private:
    VideoBusInput(std::unique_ptr<BusFrameSource> frames,
                  std::unique_ptr<perception_api_bus::BusObservationsSubscriber> observations,
                  std::unique_ptr<perception_api_bus::BusTracksSubscriber> tracks,
                  FrameCollector& collector);

    // 멤버는 선언의 역순으로 소멸한다. 구독을 구독자보다 뒤에 선언하여 먼저 해제한다
    std::unique_ptr<BusFrameSource> frames_;
    std::unique_ptr<perception_api_bus::BusObservationsSubscriber> observations_;
    std::unique_ptr<perception_api_bus::BusTracksSubscriber> tracks_;

    core::Subscription frame_subscription_;
    core::Subscription observations_subscription_;
    core::Subscription tracks_subscription_;
};

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_VIDEO_BUS_INPUT_HPP_
