// video/stream_info 메시지와 video_api 값 타입 사이의 변환.
// 생성된 메시지 타입(argus_msg/video_stream_info.proto 의 argus::msg::VideoStreamInfo)을 사용하는
// 코드는 이 파일의 함수뿐이다. 라이브러리 밖에서는 include 하지 않는다
#ifndef ARGUS_VIDEO_API_BUS_SRC_VIDEO_STREAM_INFO_CODEC_HPP_
#define ARGUS_VIDEO_API_BUS_SRC_VIDEO_STREAM_INFO_CODEC_HPP_

#include <cstdint>
#include <optional>

#include "argus/bus_zmq/zmq_codec.hpp"
#include "argus/video_api/types.hpp"

namespace argus::video_api_bus {

// 받은 메시지에서 꺼낸 값과 그 메시지의 seq
struct DecodedVideoStreamInfo {
    video_api::VideoStreamInfo value;
    std::uint64_t seq = 0;
};

bus_zmq::Bytes encode_video_stream_info(const video_api::VideoStreamInfo& info, std::uint64_t seq);
// 본문을 파싱할 수 없으면 nullopt.
// enum 의 UNSPECIFIED 와 이 코드에 정의되지 않은 값은 Unknown 으로 바꾼다
std::optional<DecodedVideoStreamInfo> decode_video_stream_info(const bus_zmq::Bytes& body);

}  // namespace argus::video_api_bus

#endif  // ARGUS_VIDEO_API_BUS_SRC_VIDEO_STREAM_INFO_CODEC_HPP_
