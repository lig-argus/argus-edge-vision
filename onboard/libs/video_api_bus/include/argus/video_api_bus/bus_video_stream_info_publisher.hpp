// 영상 스트림 정보를 video/stream_info 토픽으로 발행한다
// 규약: docs/spec/process_interfaces.md 5.1.6
// 사용: 영상 송신부의 VideoStreamInfoPublisher. 값을 정하는 일과 발행 시점(1 Hz 와 값이 바뀔 때)은
// 사용하는 코드가 맡는다
// 구독 클래스: BusVideoStreamInfoSubscriber
#ifndef ARGUS_VIDEO_API_BUS_BUS_VIDEO_STREAM_INFO_PUBLISHER_HPP_
#define ARGUS_VIDEO_API_BUS_BUS_VIDEO_STREAM_INFO_PUBLISHER_HPP_

#include <memory>

#include "argus/bus/bus.hpp"
#include "argus/bus/typed.hpp"
#include "argus/video_api/types.hpp"

namespace argus::video_api_bus {

class BusVideoStreamInfoPublisher {
public:
    // bus 가 발행자를 만들지 못하면 nullptr 를 반환한다 (이유는 bus 구현이 로그에 남긴다)
    static std::unique_ptr<BusVideoStreamInfoPublisher> create(bus::IBus& bus);

    // 호출마다 한 번 발행하고 (발행 번호는 bus 가 붙인다), 발행했으면 true 를 반환한다. 실패해도
    // 기다리거나 다시 보내지 않는다. 실패가 시작될 때와 복구될 때 한 번씩 로그에 남긴다
    bool publish(const video_api::VideoStreamInfo& info);

private:
    explicit BusVideoStreamInfoPublisher(
        std::unique_ptr<bus::Publisher<video_api::VideoStreamInfo>> publisher);

    std::unique_ptr<bus::Publisher<video_api::VideoStreamInfo>> publisher_;
    bool failing_ = false;  // 직전 발행이 실패했는지
};

}  // namespace argus::video_api_bus

#endif  // ARGUS_VIDEO_API_BUS_BUS_VIDEO_STREAM_INFO_PUBLISHER_HPP_
