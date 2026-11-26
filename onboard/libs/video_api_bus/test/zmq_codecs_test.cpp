#include "argus/video_api_bus/zmq_codecs.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>
#include <typeindex>

#include "argus/bus_zmq/zmq_codec.hpp"
#include "argus/video_api/types.hpp"
#include "argus/video_api_bus/message_types.hpp"
#include "argus_msg/video_stream_info.pb.h"

namespace argus::video_api_bus {
namespace {

using bus_zmq::Bytes;
using video_api::VideoCodec;
using video_api::VideoStream;
using video_api::VideoStreamInfo;

constexpr std::int64_t kMs = 1'000'000;

// 필드 번호 1, wire type length-delimited(2), 길이 5 인데 내용이 없는 본문 (파싱 실패)
Bytes broken_body() { return {0x0A, 0x05}; }

template <typename Message>
Bytes to_body(const Message& message) {
    const std::string serialized = message.SerializeAsString();
    return {serialized.begin(), serialized.end()};
}

template <typename Message>
Message parsed(const Bytes& body) {
    Message message;
    EXPECT_TRUE(message.ParseFromArray(body.data(), static_cast<std::int32_t>(body.size())));
    return message;
}

// 모든 필드를 기본값이 아닌 값으로 채운 스트림 둘
VideoStreamInfo full_info() {
    VideoStream ir;
    ir.stream_id = 1;
    ir.sensor = vehicle_api::Sensor::Ir;
    ir.name = "IR";
    ir.rtsp_port = 8554;
    ir.rtsp_path = "/argus/ir";
    ir.width = 640;
    ir.height = 480;
    ir.framerate_hz = 30.0;
    ir.codec = VideoCodec::H264;
    ir.bitrate_bps = 2'000'000;
    ir.hfov_deg = 24.5;
    ir.running = true;
    VideoStream eo = ir;
    eo.stream_id = 2;
    eo.sensor = vehicle_api::Sensor::Eo;
    eo.name = "EO";
    eo.rtsp_path = "/argus/eo";
    eo.codec = VideoCodec::Jpeg;
    eo.running = false;

    VideoStreamInfo info;
    info.source = "argus_video_sender";
    info.published_monotonic_ns = 7 * kMs;
    info.streams = {ir, eo};
    return info;
}

class ZmqCodecsTest : public ::testing::Test {
protected:
    std::optional<Bytes> encode(const VideoStreamInfo& value, std::uint64_t sequence) {
        Bytes body;
        Bytes binary;
        if (!codec().encode(&value, sequence, body, binary)) {
            return std::nullopt;
        }
        EXPECT_TRUE(binary.empty());
        return body;
    }

    // 본문에 실린 번호는 sequence_ 에 남는다
    std::optional<VideoStreamInfo> decode(const Bytes& body, const Bytes& binary = {}) {
        VideoStreamInfo value;
        sequence_ = 0;
        if (!codec().decode(body, binary, &value, sequence_)) {
            return std::nullopt;
        }
        return value;
    }

    const bus_zmq::ZmqCodec& codec() const {
        return table_.at(std::type_index(typeid(VideoStreamInfo)));
    }

    bus_zmq::ZmqCodecTable table_ = zmq_codecs();
    std::uint64_t sequence_ = 0;
};

TEST_F(ZmqCodecsTest, HasCodecForEveryMessageType) {
    const auto found = table_.find(kVideoStreamInfoType.type);
    ASSERT_NE(found, table_.end());
    EXPECT_TRUE(found->second.encode);
    EXPECT_TRUE(found->second.decode);
    EXPECT_EQ(table_.size(), 1U);
}

TEST_F(ZmqCodecsTest, RoundTripKeepsEveryFieldAndSequence) {
    const VideoStreamInfo sent = full_info();

    const std::optional<Bytes> body = encode(sent, 6);
    ASSERT_TRUE(body.has_value());
    EXPECT_EQ(parsed<msg::VideoStreamInfo>(body.value_or(Bytes{})).seq(), 6U);
    const VideoStreamInfo received = decode(body.value_or(Bytes{})).value_or(VideoStreamInfo{});

    EXPECT_EQ(sequence_, 6U);
    EXPECT_EQ(received.source, sent.source);
    EXPECT_EQ(received.published_monotonic_ns, sent.published_monotonic_ns);
    ASSERT_EQ(received.streams.size(), 2U);
    EXPECT_EQ(received.streams[0], sent.streams[0]);
    EXPECT_EQ(received.streams[1], sent.streams[1]);
}

TEST_F(ZmqCodecsTest, UnspecifiedAndUndefinedEnumValuesBecomeUnknown) {
    msg::VideoStreamInfo message;
    msg::VideoStreamInfo::Stream* unspecified = message.add_streams();
    unspecified->set_sensor(msg::SENSOR_UNKNOWN);
    unspecified->set_codec(msg::VIDEO_CODEC_UNSPECIFIED);
    msg::VideoStreamInfo::Stream* undefined = message.add_streams();
    // proto3 의 enum 은 정의되지 않은 값도 나르므로, 더 새 버전의 발행자가 보낸 값을 reflection
    // 으로 넣는다
    const auto* descriptor = msg::VideoStreamInfo::Stream::descriptor();
    undefined->GetReflection()->SetEnumValue(undefined, descriptor->FindFieldByName("sensor"), 99);
    undefined->GetReflection()->SetEnumValue(undefined, descriptor->FindFieldByName("codec"), 99);

    const VideoStreamInfo received = decode(to_body(message)).value_or(VideoStreamInfo{});

    ASSERT_EQ(received.streams.size(), 2U);
    for (const VideoStream& stream : received.streams) {
        EXPECT_EQ(stream.sensor, vehicle_api::Sensor::Unknown);
        EXPECT_EQ(stream.codec, VideoCodec::Unknown);
    }
}

TEST_F(ZmqCodecsTest, DecodeFailsForBodyThatCannotBeParsed) {
    EXPECT_FALSE(decode(broken_body()).has_value());
}

TEST_F(ZmqCodecsTest, DecodeFailsForMessageWithBinary) {
    EXPECT_TRUE(decode(Bytes{}).has_value());  // 같은 본문이 바이너리 없이는 성공한다
    EXPECT_FALSE(decode(Bytes{}, Bytes{1}).has_value());
}

}  // namespace
}  // namespace argus::video_api_bus
