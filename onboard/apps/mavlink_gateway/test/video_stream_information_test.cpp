#include "argus/mavlink_gateway/video_stream_information.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/mavlink_ids.hpp"
#include "argus/video_api/types.hpp"
#include "fake_gcs_link.hpp"
#include "fake_video_stream_info_source.hpp"
#include "gcs_test_messages.hpp"

namespace argus::mavlink_gateway {
namespace {

using test_messages::gcs_peer;

video_api::VideoStream ir_stream() {
    video_api::VideoStream stream;
    stream.stream_id = 1;
    stream.sensor = vehicle_api::Sensor::Ir;
    stream.name = "IR";
    stream.rtsp_port = 8554;
    stream.rtsp_path = "/argus/ir";
    stream.width = 640;
    stream.height = 480;
    stream.framerate_hz = 30.0;
    stream.codec = video_api::VideoCodec::H264;
    stream.bitrate_bps = 2'000'000;
    stream.hfov_deg = 24.4;
    stream.running = true;
    return stream;
}

video_api::VideoStream eo_stream() {
    video_api::VideoStream stream = ir_stream();
    stream.stream_id = 2;
    stream.sensor = vehicle_api::Sensor::Eo;
    stream.name = "EO";
    stream.rtsp_path = "/argus/eo";
    stream.codec = video_api::VideoCodec::Jpeg;
    return stream;
}

mavlink_video_stream_information_t decoded(const mavlink_message_t& message) {
    mavlink_video_stream_information_t information{};
    mavlink_msg_video_stream_information_decode(&message, &information);
    return information;
}

std::string text_of(const char* field, std::size_t length) {
    return {field, strnlen(field, length)};
}

class VideoStreamInformationTest : public ::testing::Test {
protected:
    VideoStreamInformationTest() {
        link_.set_local_address(sender_.address, "192.168.0.2");
        set_streams({ir_stream()});
    }

    void set_streams(const std::vector<video_api::VideoStream>& streams) {
        video_api::ReceivedVideoStreamInfo received;
        received.info.streams = streams;
        source_.value = received;
    }

    RequestedMessages build(float param2) const {
        CommandRequest request;
        request.sender = sender_;
        request.command.command = MAV_CMD_REQUEST_MESSAGE;
        request.command.param1 = MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION;
        request.command.param2 = param2;
        return build_video_stream_information(request, source_, link_);
    }

    const GcsPeer sender_ = gcs_peer(251, "192.168.0.11", 40001);
    FakeGcsLink link_;
    FakeVideoStreamInfoSource source_;
};

TEST_F(VideoStreamInformationTest, FillsEveryFieldFromLatestStreamInfo) {
    const RequestedMessages built = build(1.0F);

    ASSERT_EQ(built.result, MAV_RESULT_ACCEPTED);
    ASSERT_EQ(built.messages.size(), 1U);
    const mavlink_message_t& message = built.messages.front();
    EXPECT_EQ(message.msgid, MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION);
    EXPECT_EQ(message.sysid, kVehicleSysid);
    EXPECT_EQ(message.compid, kOnboardCommandCompid);
    const mavlink_video_stream_information_t information = decoded(message);
    EXPECT_EQ(information.stream_id, 1);
    EXPECT_EQ(information.count, 1);
    EXPECT_EQ(information.type, VIDEO_STREAM_TYPE_RTSP);
    EXPECT_EQ(information.flags,
              VIDEO_STREAM_STATUS_FLAGS_RUNNING | VIDEO_STREAM_STATUS_FLAGS_THERMAL);
    EXPECT_FLOAT_EQ(information.framerate, 30.0F);
    EXPECT_EQ(information.resolution_h, 640);
    EXPECT_EQ(information.resolution_v, 480);
    EXPECT_EQ(information.bitrate, 2'000'000U);
    EXPECT_EQ(information.rotation, 0);
    EXPECT_EQ(information.hfov, 24);
    EXPECT_EQ(text_of(information.name, sizeof(information.name)), "IR");
    EXPECT_EQ(text_of(information.uri, sizeof(information.uri)),
              "rtsp://192.168.0.2:8554/argus/ir");
    EXPECT_EQ(information.encoding, VIDEO_STREAM_ENCODING_H264);
    EXPECT_EQ(information.camera_device_id, 0);
}

TEST_F(VideoStreamInformationTest, StreamIdZeroSendsEveryStream) {
    set_streams({ir_stream(), eo_stream()});

    const RequestedMessages built = build(0.0F);

    ASSERT_EQ(built.result, MAV_RESULT_ACCEPTED);
    ASSERT_EQ(built.messages.size(), 2U);
    EXPECT_EQ(decoded(built.messages[0]).stream_id, 1);
    EXPECT_EQ(decoded(built.messages[1]).stream_id, 2);
    EXPECT_EQ(decoded(built.messages[1]).count, 2);
}

TEST_F(VideoStreamInformationTest, StreamIdSelectsOneStreamWithTotalCount) {
    set_streams({ir_stream(), eo_stream()});

    const RequestedMessages built = build(2.0F);

    ASSERT_EQ(built.messages.size(), 1U);
    const mavlink_video_stream_information_t information = decoded(built.messages.front());
    EXPECT_EQ(information.stream_id, 2);
    EXPECT_EQ(information.count, 2);
    EXPECT_EQ(text_of(information.uri, sizeof(information.uri)),
              "rtsp://192.168.0.2:8554/argus/eo");
}

TEST_F(VideoStreamInformationTest, FlagsFollowRunningAndSensor) {
    video_api::VideoStream ir_stopped = ir_stream();
    ir_stopped.running = false;
    video_api::VideoStream eo_running = eo_stream();
    set_streams({ir_stopped, eo_running});

    const RequestedMessages built = build(0.0F);

    ASSERT_EQ(built.messages.size(), 2U);
    EXPECT_EQ(decoded(built.messages[0]).flags, VIDEO_STREAM_STATUS_FLAGS_THERMAL);
    EXPECT_EQ(decoded(built.messages[1]).flags, VIDEO_STREAM_STATUS_FLAGS_RUNNING);
}

TEST_F(VideoStreamInformationTest, JpegIsSentAsUnknownEncoding) {
    set_streams({eo_stream()});

    const RequestedMessages built = build(0.0F);

    ASSERT_EQ(built.messages.size(), 1U);
    EXPECT_EQ(decoded(built.messages.front()).encoding, VIDEO_STREAM_ENCODING_UNKNOWN);
}

TEST_F(VideoStreamInformationTest, DeniesWhenNothingReceivedOrNoStream) {
    source_.value.reset();
    EXPECT_EQ(build(0.0F).result, MAV_RESULT_DENIED);

    set_streams({});
    EXPECT_EQ(build(0.0F).result, MAV_RESULT_DENIED);
}

TEST_F(VideoStreamInformationTest, DeniesUnknownOrInvalidStreamId) {
    for (const float param2 : {3.0F, 1.5F, -1.0F, test_messages::kNan, 256.0F}) {
        const RequestedMessages built = build(param2);
        EXPECT_EQ(built.result, MAV_RESULT_DENIED) << param2;
        EXPECT_FALSE(built.reason.empty());
        EXPECT_TRUE(built.messages.empty());
    }
}

TEST_F(VideoStreamInformationTest, FailsWithoutLocalAddress) {
    FakeGcsLink no_address;
    CommandRequest request;
    request.sender = sender_;
    request.command.param2 = 0.0F;

    const RequestedMessages built = build_video_stream_information(request, source_, no_address);

    EXPECT_EQ(built.result, MAV_RESULT_FAILED);
    EXPECT_TRUE(built.messages.empty());
}

TEST_F(VideoStreamInformationTest, FailsWhenUriDoesNotFitField) {
    video_api::VideoStream stream = ir_stream();
    stream.rtsp_path = "/" + std::string(200, 'a');
    set_streams({stream});

    EXPECT_EQ(build(0.0F).result, MAV_RESULT_FAILED);
}

}  // namespace
}  // namespace argus::mavlink_gateway
