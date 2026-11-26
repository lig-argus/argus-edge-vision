// 테스트용 IVideoStreamInfoSource. 테스트가 넣은 값을 그대로 돌려준다
#ifndef ARGUS_ONBOARD_TEST_FAKE_VIDEO_STREAM_INFO_SOURCE_HPP_
#define ARGUS_ONBOARD_TEST_FAKE_VIDEO_STREAM_INFO_SOURCE_HPP_

#include <optional>

#include "argus/video_api/types.hpp"
#include "argus/video_api/video_stream_info_source.hpp"

namespace argus::mavlink_gateway {

class FakeVideoStreamInfoSource : public video_api::IVideoStreamInfoSource {
public:
    std::optional<video_api::ReceivedVideoStreamInfo> latest() const override { return value; }

    std::optional<video_api::ReceivedVideoStreamInfo> value;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_TEST_FAKE_VIDEO_STREAM_INFO_SOURCE_HPP_
