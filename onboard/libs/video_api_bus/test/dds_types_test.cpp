// dds_types() 의 변환 테스트. IDL 타입이 .proto 의 필드를 빠짐없이 나르는지 확인한다: 값을 IDL
// 표본으로 바꿨다가 되돌린 결과를 zmq_codecs() 로 protobuf 본문으로 만들어, 원래 값의 본문과
// 바이트가 같은지 비교한다
#include "argus/video_api_bus/dds_types.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <typeindex>

#include "argus/bus_dds/dds_type_table.hpp"
#include "argus/bus_zmq/zmq_codec.hpp"
#include "argus/video_api/types.hpp"
#include "argus/video_api_bus/message_types.hpp"
#include "argus/video_api_bus/zmq_codecs.hpp"
#include "argus_msg/video_stream_info.hpp"

namespace argus::video_api_bus {
namespace {

using bus_zmq::Bytes;
using video_api::VideoCodec;
using video_api::VideoStream;
using video_api::VideoStreamInfo;

class DdsTypesTest : public ::testing::Test {
protected:
    const bus_dds::DdsType& type() const {
        return dds_.at(std::type_index(typeid(VideoStreamInfo)));
    }

    // value 를 IDL 표본으로 바꿨다가 되돌린다. 어느 쪽이든 실패하면 nullopt
    std::optional<VideoStreamInfo> through_idl(const VideoStreamInfo& value) const {
        idl::VideoStreamInfo sample;
        if (!type().to_idl(&value, &sample)) {
            return std::nullopt;
        }
        return from_sample(sample);
    }

    std::optional<VideoStreamInfo> from_sample(const idl::VideoStreamInfo& sample) const {
        VideoStreamInfo value;
        if (!type().from_idl(&sample, &value)) {
            return std::nullopt;
        }
        return value;
    }

    // value 의 protobuf 본문 (발행 번호 1)
    Bytes proto_body(const VideoStreamInfo& value) const {
        Bytes body;
        Bytes binary;
        EXPECT_TRUE(
            zmq_.at(std::type_index(typeid(VideoStreamInfo))).encode(&value, 1, body, binary));
        return body;
    }

    bus_dds::DdsTypeTable dds_ = dds_types();
    bus_zmq::ZmqCodecTable zmq_ = zmq_codecs();
};

TEST_F(DdsTypesTest, HasTypeForEveryMessageType) {
    EXPECT_NE(dds_.find(kVideoStreamInfoType.type), dds_.end());
    EXPECT_EQ(dds_.size(), 1U);
}

TEST_F(DdsTypesTest, VideoStreamInfoKeepsEveryField) {
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
    eo.codec = VideoCodec::Jpeg;
    eo.running = false;
    VideoStreamInfo info;
    info.source = "argus_video_sender";
    info.published_monotonic_ns = 123'456'789;
    info.streams = {ir, eo};

    const std::optional<VideoStreamInfo> returned = through_idl(info);

    ASSERT_TRUE(returned.has_value());
    EXPECT_EQ(proto_body(returned.value_or(VideoStreamInfo{})), proto_body(info));
}

TEST_F(DdsTypesTest, ConvertingIntoUsedSampleReplacesPreviousStreams) {
    VideoStreamInfo two;
    two.streams.resize(2);
    VideoStreamInfo one;
    one.streams.resize(1);
    idl::VideoStreamInfo sample;
    ASSERT_TRUE(type().to_idl(&two, &sample));

    ASSERT_TRUE(type().to_idl(&one, &sample));

    EXPECT_EQ(sample.streams().size(), 1U);
}

TEST_F(DdsTypesTest, UnspecifiedCodecBecomesUnknown) {
    idl::VideoStreamInfo sample;
    idl::VideoStreamInfo_Stream stream;
    stream.codec(idl::VideoCodec::VIDEO_CODEC_UNSPECIFIED);
    stream.sensor(idl::Sensor::SENSOR_UNKNOWN);
    sample.streams().push_back(stream);

    const VideoStreamInfo received = from_sample(sample).value_or(VideoStreamInfo{});

    ASSERT_EQ(received.streams.size(), 1U);
    EXPECT_EQ(received.streams.front().codec, VideoCodec::Unknown);
    EXPECT_EQ(received.streams.front().sensor, vehicle_api::Sensor::Unknown);
}

}  // namespace
}  // namespace argus::video_api_bus
