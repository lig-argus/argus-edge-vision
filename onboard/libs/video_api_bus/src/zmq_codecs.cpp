#include "argus/video_api_bus/zmq_codecs.hpp"

#include <cstdint>
#include <optional>
#include <utility>

#include "argus/video_api/types.hpp"
#include "video_stream_info_codec.hpp"

namespace argus::video_api_bus {

bus_zmq::ZmqCodecTable zmq_codecs() {
    using bus_zmq::Bytes;
    bus_zmq::ZmqCodecTable table;
    // 발행 번호를 본문의 seq 에 싣는다. 바이너리가 있는 메시지는 형식이 맞지 않는 것이다
    bus_zmq::add_zmq_codec<video_api::VideoStreamInfo>(
        table,
        [](const video_api::VideoStreamInfo& value, std::uint64_t sequence, Bytes& body,
           Bytes& /*binary*/) {
            body = encode_video_stream_info(value, sequence);
            return true;
        },
        [](const Bytes& body, const Bytes& binary, video_api::VideoStreamInfo& value,
           std::uint64_t& sequence) {
            std::optional<DecodedVideoStreamInfo> decoded =
                binary.empty() ? decode_video_stream_info(body) : std::nullopt;
            if (!decoded) {
                return false;
            }
            value = std::move(decoded->value);
            sequence = decoded->seq;
            return true;
        });
    return table;
}

}  // namespace argus::video_api_bus
