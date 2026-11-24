#include "argus/video_sender/video_encode_config.hpp"

#include <gtest/gtest.h>

#include <string>

namespace argus::video_sender {
namespace {

bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

TEST(VideoEncodeConfigTest, DefaultsToRtpH264WithDynamicPayloadType) {
    const VideoEncodeConfig config;

    EXPECT_EQ(config.codec, VideoCodec::H264);
    EXPECT_TRUE(contains(config.gstreamer_elements, "x264enc"));
    EXPECT_TRUE(contains(config.gstreamer_elements, "tune=zerolatency"));
    EXPECT_FALSE(contains(config.gstreamer_elements, "jpegenc"));
    // 마지막 요소가 RTP 페이로더이다 (RtspServer 가 name 과 pt 를 그 뒤에 지정한다)
    EXPECT_TRUE(contains(config.gstreamer_elements, "! rtph264pay"));
    // 고정 번호가 없는 코덱이므로 동적 번호(96~127)를 사용한다
    EXPECT_GE(config.payload_type, 96);
    EXPECT_LE(config.payload_type, 127);
    EXPECT_FALSE(config.quality.has_value());
    EXPECT_FALSE(config.bitrate_kbps.has_value());
}

TEST(VideoEncodeConfigTest, H264ConfigEqualsDefault) {
    const VideoEncodeConfig made = make_video_encode_config(VideoCodec::H264);
    const VideoEncodeConfig defaults;

    EXPECT_EQ(made.codec, VideoCodec::H264);
    EXPECT_EQ(made.gstreamer_elements, defaults.gstreamer_elements);
    EXPECT_EQ(made.payload_type, defaults.payload_type);
}

TEST(VideoEncodeConfigTest, JpegConfigUsesJpegencWithStaticPayloadType) {
    const VideoEncodeConfig config = make_video_encode_config(VideoCodec::Jpeg);

    EXPECT_EQ(config.codec, VideoCodec::Jpeg);
    EXPECT_TRUE(contains(config.gstreamer_elements, "jpegenc"));
    EXPECT_FALSE(contains(config.gstreamer_elements, "x264enc"));
    EXPECT_TRUE(contains(config.gstreamer_elements, "! rtpjpegpay"));
    // JPEG 의 고정 번호 (RFC 3551)
    EXPECT_EQ(config.payload_type, 26);
    EXPECT_FALSE(config.quality.has_value());
    EXPECT_FALSE(config.bitrate_kbps.has_value());
}

}  // namespace
}  // namespace argus::video_sender
