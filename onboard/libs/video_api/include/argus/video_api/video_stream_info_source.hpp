// 영상 송신부가 발행한 최신 스트림 정보(`video/stream_info`)를 조회하는 인터페이스
// 규약: docs/spec/process_interfaces.md 5.1.6, docs/design/uml/class_mavlink_gateway.md 3절
// 구현: BusVideoStreamInfoSubscriber (video_api_bus. 받은 시각은 core::IClock)
// 사용: mavlink_gateway 의 RequestMessageHandler (VIDEO_STREAM_INFORMATION 응답)
#ifndef ARGUS_VIDEO_API_VIDEO_STREAM_INFO_SOURCE_HPP_
#define ARGUS_VIDEO_API_VIDEO_STREAM_INFO_SOURCE_HPP_

#include <optional>

#include "argus/video_api/types.hpp"

namespace argus::video_api {

class IVideoStreamInfoSource {
public:
    IVideoStreamInfoSource() = default;
    virtual ~IVideoStreamInfoSource() = default;
    IVideoStreamInfoSource(const IVideoStreamInfoSource&) = delete;
    IVideoStreamInfoSource& operator=(const IVideoStreamInfoSource&) = delete;
    IVideoStreamInfoSource(IVideoStreamInfoSource&&) = delete;
    IVideoStreamInfoSource& operator=(IVideoStreamInfoSource&&) = delete;

    // 가장 최근에 받은 값. 아직 하나도 받지 못했으면 nullopt.
    // 값이 얼마나 오래됐는지는 호출자가 arrived_monotonic_ns 로 판단한다
    virtual std::optional<ReceivedVideoStreamInfo> latest() const = 0;
};

}  // namespace argus::video_api

#endif  // ARGUS_VIDEO_API_VIDEO_STREAM_INFO_SOURCE_HPP_
