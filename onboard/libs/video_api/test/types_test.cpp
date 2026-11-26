#include "argus/video_api/types.hpp"

#include <gtest/gtest.h>

namespace argus::video_api {
namespace {

VideoStream ir_stream() {
    VideoStream stream;
    stream.stream_id = 1;
    stream.sensor = vehicle_api::Sensor::Ir;
    stream.name = "IR";
    stream.rtsp_port = 8554;
    stream.rtsp_path = "/argus/ir";
    stream.width = 640;
    stream.height = 480;
    stream.framerate_hz = 30.0;
    stream.codec = VideoCodec::H264;
    stream.bitrate_bps = 2'000'000;
    stream.hfov_deg = 0.0;
    stream.running = true;
    return stream;
}

TEST(VideoStreamTest, EqualWhenEveryFieldIsEqual) { EXPECT_EQ(ir_stream(), ir_stream()); }

// 영상 송신부는 이 비교로 값이 바뀌었는지 판단해 바로 발행한다. 필드 하나만 달라도 달라야 한다
TEST(VideoStreamTest, DifferentWhenAnyFieldDiffers) {
    const VideoStream base = ir_stream();
    VideoStream changed = base;

    changed = base;
    changed.stream_id = 2;
    EXPECT_NE(changed, base);
    changed = base;
    changed.sensor = vehicle_api::Sensor::Eo;
    EXPECT_NE(changed, base);
    changed = base;
    changed.name = "EO";
    EXPECT_NE(changed, base);
    changed = base;
    changed.rtsp_port = 8555;
    EXPECT_NE(changed, base);
    changed = base;
    changed.rtsp_path = "/argus/eo";
    EXPECT_NE(changed, base);
    changed = base;
    changed.width = 320;
    EXPECT_NE(changed, base);
    changed = base;
    changed.height = 240;
    EXPECT_NE(changed, base);
    changed = base;
    changed.framerate_hz = 15.0;
    EXPECT_NE(changed, base);
    changed = base;
    changed.codec = VideoCodec::Jpeg;
    EXPECT_NE(changed, base);
    changed = base;
    changed.bitrate_bps = 0;
    EXPECT_NE(changed, base);
    changed = base;
    changed.hfov_deg = 24.0;
    EXPECT_NE(changed, base);
    changed = base;
    changed.running = false;
    EXPECT_NE(changed, base);
}

}  // namespace
}  // namespace argus::video_api
