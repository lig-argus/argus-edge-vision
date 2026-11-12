// RtspServer 를 로컬에서 열고 GStreamer 의 RTSP 클라이언트(rtspsrc)로 수신해 확인한다
#include "argus/video_sender/rtsp_server.hpp"

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "argus/core/steady_clock.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/video_sender/metadata_rtp_packetizer.hpp"
#include "argus/video_sender/stream_timing.hpp"
#include "argus/video_sender/video_encode_config.hpp"

namespace argus::video_sender {
namespace {

using perception_api::Frame;

constexpr std::uint8_t kMetadataPayloadType = 107;
constexpr std::uint32_t kSsrc = 0x11223344;
constexpr std::uint32_t kWidth = 640;
constexpr std::uint32_t kHeight = 480;
constexpr std::int64_t kFirstUnixNs = 1'792'048'527'321'000'000;
constexpr std::int64_t kFrameIntervalNs = 33'333'333;
constexpr std::chrono::milliseconds kFramePeriod{33};
constexpr std::chrono::seconds kReceiveTimeout{10};
constexpr GstClockTime kPullTimeout = 20 * GST_MSECOND;
constexpr std::size_t kMaxPacketBytes = 300;
constexpr std::size_t kFrameIdDigits = 8;
const char* const kFrameIdOpen = "<FrameId>";

// 여러 패킷으로 나뉘는 길이의 문서. frame_id 를 고정 폭으로 기록하므로 길이가 frame_id 와 무관하다
std::string metadata_document(std::uint64_t frame_id) {
    std::ostringstream document;
    document << R"(<tt:MetadataStream xmlns:tt="http://www.onvif.org/ver10/schema">)"
             << kFrameIdOpen << std::setfill('0') << std::setw(kFrameIdDigits) << frame_id
             << "</FrameId>" << std::string(700, ' ') << "</tt:MetadataStream>";
    return document.str();
}

// metadata_document() 로 만든 문서의 frame_id. 형식이 다르면 값이 없다
std::optional<std::uint64_t> frame_id_of(const std::string& document) {
    const std::size_t open = document.find(kFrameIdOpen);
    if (open == std::string::npos) {
        return std::nullopt;
    }
    const std::string digits =
        document.substr(open + std::string{kFrameIdOpen}.size(), kFrameIdDigits);
    if (digits.size() != kFrameIdDigits ||
        digits.find_first_not_of("0123456789") != std::string::npos) {
        return std::nullopt;
    }
    return std::strtoull(digits.c_str(), nullptr, 10);
}

std::int64_t unix_ns_of(std::uint64_t frame_id) {
    return kFirstUnixNs + static_cast<std::int64_t>(frame_id) * kFrameIntervalNs;
}

Frame make_frame(const core::IClock& clock, std::uint64_t frame_id) {
    Frame frame;
    frame.source = "camera-ir";
    frame.frame_id = frame_id;
    frame.unix_ns = unix_ns_of(frame_id);
    frame.received_monotonic_ns = clock.now_monotonic_ns();
    frame.width = kWidth;
    frame.height = kHeight;
    frame.stride = kWidth;
    // 품질 설정에 따라 JPEG 크기가 달라지도록 밝기가 픽셀마다 변하는 영상을 사용한다
    frame.pixels.resize(frame.expected_pixel_bytes());
    for (std::uint32_t y = 0; y < kHeight; ++y) {
        for (std::uint32_t x = 0; x < kWidth; ++x) {
            frame.pixels[static_cast<std::size_t>(y) * kWidth + x] =
                static_cast<std::uint8_t>((x * 7 + y * 13 + frame_id) % 256);
        }
    }
    return frame;
}

// 영상 트랙의 RTP 패킷에서 인코딩된 프레임을 복원하는 요소
const char* const kJpegDepayloader = "rtpjpegdepay";
const char* const kH264Depayloader = "rtph264depay";
const char* const kH264EncoderFactory = "x264enc";

// RTSP 클라이언트. 영상은 RTP 패킷에서 복원한 인코딩된 프레임을, 메타데이터는 RTP 패킷을 appsink 로
// 받는다
class RtspClient {
public:
    explicit RtspClient(std::uint16_t port, const std::string& depayloader = kJpegDepayloader,
                        const std::string& path = "/argus/ir") {
        const std::string description =
            "rtspsrc name=source protocols=tcp latency=0 location=rtsp://127.0.0.1:" +
            std::to_string(port) + path + " source. ! application/x-rtp,media=video ! " +
            depayloader +
            " ! "
            "appsink name=video sync=false "
            "source. ! application/x-rtp,media=application ! appsink name=metadata sync=false";
        pipeline_ = gst_parse_launch(description.c_str(), nullptr);
        if (pipeline_ == nullptr) {
            return;
        }
        video_ = gst_bin_get_by_name(GST_BIN(pipeline_), "video");
        metadata_ = gst_bin_get_by_name(GST_BIN(pipeline_), "metadata");
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    }

    ~RtspClient() {
        if (pipeline_ != nullptr) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
        }
        gst_clear_object(&video_);
        gst_clear_object(&metadata_);
        gst_clear_object(&pipeline_);
    }

    RtspClient(const RtspClient&) = delete;
    RtspClient& operator=(const RtspClient&) = delete;
    RtspClient(RtspClient&&) = delete;
    RtspClient& operator=(RtspClient&&) = delete;

    // 테스트에 필요한 GStreamer 요소가 설치되어 있어 클라이언트를 구성했는지
    bool valid() const { return pipeline_ != nullptr && video_ != nullptr && metadata_ != nullptr; }

    GstSample* pull_video() {
        return gst_app_sink_try_pull_sample(GST_APP_SINK(video_), kPullTimeout);
    }
    GstSample* pull_metadata() {
        return gst_app_sink_try_pull_sample(GST_APP_SINK(metadata_), kPullTimeout);
    }

private:
    GstElement* pipeline_ = nullptr;
    GstElement* video_ = nullptr;
    GstElement* metadata_ = nullptr;
};

struct ReceivedPacket {
    bool marker = false;
    std::uint8_t payload_type = 0;
    std::uint32_t timestamp = 0;
    std::uint32_t ssrc = 0;
    std::string payload;
};

std::uint32_t u32_at(const guint8* data, std::size_t offset) {
    return (static_cast<std::uint32_t>(data[offset]) << 24U) |
           (static_cast<std::uint32_t>(data[offset + 1]) << 16U) |
           (static_cast<std::uint32_t>(data[offset + 2]) << 8U) |
           static_cast<std::uint32_t>(data[offset + 3]);
}

ReceivedPacket parse_packet(GstSample* sample) {
    ReceivedPacket packet;
    GstMapInfo map;
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    if (gst_buffer_map(buffer, &map, GST_MAP_READ) == FALSE) {
        return packet;
    }
    if (map.size >= MetadataRtpPacketizer::kHeaderBytes) {
        packet.marker = (map.data[1] & 0x80U) != 0;
        packet.payload_type = static_cast<std::uint8_t>(map.data[1] & 0x7FU);
        packet.timestamp = u32_at(map.data, 4);
        packet.ssrc = u32_at(map.data, 8);
        packet.payload.assign(map.data + MetadataRtpPacketizer::kHeaderBytes, map.data + map.size);
    }
    gst_buffer_unmap(buffer, &map);
    return packet;
}

// marker 가 1 인 패킷까지의 payload 를 이은 문서 하나와 그 문서의 패킷들
struct ReceivedDocument {
    std::string text;
    std::vector<ReceivedPacket> packets;
};

MetadataRtpConfig metadata_rtp_config() {
    MetadataRtpConfig config{kMetadataPayloadType, kSsrc};
    config.max_packet_bytes = kMaxPacketBytes;
    return config;
}

// 코덱과 무관한 서버 동작의 테스트는 JPEG 로 실행한다. JPEG 인코더(jpegenc)는 plugins-good 에
// 있어 x264enc(plugins-ugly)가 없는 환경에서도 실행된다. 기본 코덱 H.264 는 아래 H.264 테스트에서
// 확인한다
RtspServerConfig server_config() {
    RtspServerConfig config{metadata_rtp_config()};
    config.port = 0;  // 운영체제가 정한 포트. 다른 프로세스와 겹치지 않는다
    config.video = make_video_encode_config(VideoCodec::Jpeg);
    return config;
}

// 서버에 프레임을 push 하면서 클라이언트로 수신한다
class Session {
public:
    Session(const core::IClock& clock, RtspServer& server) : clock_(clock), server_(server) {}

    void push_frame() {
        const std::uint64_t frame_id = next_frame_id_++;
        server_.push(make_frame(clock_, frame_id), metadata_document(frame_id));
    }

    // done() 이 true 를 반환할 때까지 프레임을 push 하고 수신한 것을 모은다.
    // kReceiveTimeout 안에 끝나지 않으면 false 를 반환한다
    bool push_and_receive_until(RtspClient& client, const std::function<bool()>& done) {
        const auto deadline = std::chrono::steady_clock::now() + kReceiveTimeout;
        while (!done()) {
            if (std::chrono::steady_clock::now() >= deadline) {
                return false;
            }
            push_frame();
            std::this_thread::sleep_for(kFramePeriod);
            receive(client);
        }
        return true;
    }

    std::int32_t video_frames = 0;
    std::string video_media_type;  // 수신한 영상 버퍼의 caps 이름 (image/jpeg, video/x-h264)
    gint video_width = 0;
    gint video_height = 0;
    std::size_t video_bytes = 0;
    std::vector<ReceivedDocument> documents;

private:
    void receive(RtspClient& client) {
        if (GstSample* sample = client.pull_video(); sample != nullptr) {
            const GstStructure* structure = gst_caps_get_structure(gst_sample_get_caps(sample), 0);
            video_media_type = gst_structure_get_name(structure);
            gst_structure_get_int(structure, "width", &video_width);
            gst_structure_get_int(structure, "height", &video_height);
            video_bytes += gst_buffer_get_size(gst_sample_get_buffer(sample));
            ++video_frames;
            gst_sample_unref(sample);
        }
        while (GstSample* sample = client.pull_metadata()) {
            ReceivedPacket packet = parse_packet(sample);
            gst_sample_unref(sample);
            partial_.text += packet.payload;
            const bool is_last = packet.marker;
            partial_.packets.push_back(std::move(packet));
            if (is_last) {
                documents.push_back(std::move(partial_));
                partial_ = {};
            }
        }
    }

    const core::IClock& clock_;
    RtspServer& server_;
    std::uint64_t next_frame_id_ = 0;
    ReceivedDocument partial_;
};

class RtspServerTest : public ::testing::Test {
protected:
    void SetUp() override { gst_init(nullptr, nullptr); }

    core::SteadyClock clock_;
    RtspServer server_{clock_, server_config()};
    Session session_{clock_, server_};
};

TEST_F(RtspServerTest, StartOpensPortAndStopClosesIt) {
    EXPECT_EQ(server_.port(), 0U);

    ASSERT_TRUE(server_.start());
    EXPECT_NE(server_.port(), 0U);
    EXPECT_TRUE(server_.start());  // 이미 열려 있으면 true

    server_.stop();
    EXPECT_EQ(server_.port(), 0U);
}

TEST_F(RtspServerTest, StartFailsWhenPortIsInUse) {
    ASSERT_TRUE(server_.start());
    RtspServerConfig same_port = server_config();
    same_port.port = server_.port();
    RtspServer second{clock_, same_port};

    EXPECT_FALSE(second.start());
    EXPECT_EQ(second.port(), 0U);
}

TEST_F(RtspServerTest, StartFailsWhenVideoElementsCannotBuildPipeline) {
    RtspServerConfig config = server_config();
    config.video.gstreamer_elements = "videoconvert ! element_that_does_not_exist";
    RtspServer server{clock_, config};

    EXPECT_FALSE(server.start());
    EXPECT_EQ(server.port(), 0U);
}

TEST_F(RtspServerTest, StartFailsWhenQualityIsSetForNonJpegCodec) {
    RtspServerConfig config = server_config();
    config.video.codec = VideoCodec::H264;
    config.video.quality = 50;
    RtspServer server{clock_, config};

    EXPECT_FALSE(server.start());
}

TEST_F(RtspServerTest, StartFailsWhenBitrateIsSetForNonH264Codec) {
    RtspServerConfig config = server_config();
    config.video.bitrate_kbps = 1000;
    RtspServer server{clock_, config};

    EXPECT_FALSE(server.start());
}

TEST_F(RtspServerTest, StartFailsWhenEncoderForSettingIsNotInElements) {
    // codec 은 H264 인데 gstreamer_elements 는 JPEG 의 것이어서 bitrate_kbps 를 지정할 요소가 없다
    RtspServerConfig config = server_config();
    config.video.codec = VideoCodec::H264;
    config.video.bitrate_kbps = 1000;
    RtspServer server{clock_, config};

    EXPECT_FALSE(server.start());
}

TEST_F(RtspServerTest, StartFailsWhenVideoAndMetadataPayloadTypesAreEqual) {
    RtspServerConfig config = server_config();
    config.video = make_video_encode_config(VideoCodec::H264);
    config.video.payload_type = kMetadataPayloadType;
    RtspServer server{clock_, config};

    EXPECT_FALSE(server.start());
}

TEST_F(RtspServerTest, PushWithoutClientIsCountedAsDropped) {
    ASSERT_TRUE(server_.start());

    for (std::int32_t i = 0; i < 5; ++i) {
        session_.push_frame();
    }

    EXPECT_EQ(server_.stats().dropped_no_client, 5U);
    EXPECT_EQ(server_.stats().sent, 0U);

    server_.stop();
    session_.push_frame();  // 서버를 닫은 뒤의 push 도 같은 방식으로 폐기한다
    EXPECT_EQ(server_.stats().dropped_no_client, 6U);
}

TEST_F(RtspServerTest, ClientReceivesVideoAndMetadataTracks) {
    ASSERT_TRUE(server_.start());
    RtspClient client{server_.port()};
    ASSERT_TRUE(client.valid());

    ASSERT_TRUE(session_.push_and_receive_until(
        client, [&] { return session_.video_frames >= 3 && session_.documents.size() >= 4; }));

    EXPECT_EQ(session_.video_media_type, "image/jpeg");
    EXPECT_EQ(session_.video_width, static_cast<gint>(kWidth));
    EXPECT_EQ(session_.video_height, static_cast<gint>(kHeight));
    EXPECT_GT(server_.stats().sent, 0U);
    EXPECT_EQ(server_.stats().dropped_invalid_frame, 0U);
    EXPECT_EQ(server_.stats().dropped_send_failed, 0U);

    // 수신을 시작한 문서가 문서의 중간일 수 있으므로 두 번째 문서부터 확인한다
    for (std::size_t i = 1; i < session_.documents.size(); ++i) {
        const ReceivedDocument& document = session_.documents[i];
        const std::optional<std::uint64_t> frame_id = frame_id_of(document.text);
        ASSERT_TRUE(frame_id.has_value()) << "문서 " << i;
        // 나뉜 패킷을 이으면 push 한 문서와 같다
        EXPECT_EQ(document.text, metadata_document(frame_id.value_or(0)));
        EXPECT_GT(document.packets.size(), 1U);
        for (const ReceivedPacket& packet : document.packets) {
            EXPECT_EQ(packet.payload_type, kMetadataPayloadType);
            EXPECT_EQ(packet.ssrc, kSsrc);
            // RTP 타임스탬프는 그 문서의 프레임의 unix_ns 에서 계산한 값이다
            EXPECT_EQ(packet.timestamp,
                      rtp_timestamp_from_unix_ns(unix_ns_of(frame_id.value_or(0))));
        }
    }
}

TEST_F(RtspServerTest, FrameWithWrongSizeIsCountedAsInvalid) {
    ASSERT_TRUE(server_.start());
    RtspClient client{server_.port()};
    ASSERT_TRUE(client.valid());
    ASSERT_TRUE(
        session_.push_and_receive_until(client, [&] { return session_.video_frames >= 1; }));

    Frame frame = make_frame(clock_, 0);
    frame.pixels.resize(frame.pixels.size() / 2);  // 크기 정보보다 픽셀이 적다
    server_.push(frame, metadata_document(0));

    EXPECT_EQ(server_.stats().dropped_invalid_frame, 1U);
}

// 클라이언트가 끊은 뒤 다시 접속하면 파이프라인과 appsrc 가 새로 생성된다. 새 appsrc 에도 영상의
// caps 가 설정되어 영상이 송신되는지 확인한다
TEST_F(RtspServerTest, ReconnectedClientReceivesVideo) {
    ASSERT_TRUE(server_.start());

    constexpr std::int32_t kConnections = 3;
    for (std::int32_t connection = 1; connection <= kConnections; ++connection) {
        RtspClient client{server_.port()};
        ASSERT_TRUE(client.valid());
        const std::int32_t frames_before = session_.video_frames;

        EXPECT_TRUE(session_.push_and_receive_until(
            client, [&] { return session_.video_frames > frames_before; }))
            << "접속 " << connection << " 번째에서 영상을 수신하지 못함";
    }
}

// 설정대로 서버를 열고 수신한 인코딩된 프레임 크기의 평균(바이트)을 반환한다.
// 프레임을 수신하지 못하면 0 을 반환한다
std::size_t mean_video_bytes(const core::IClock& clock, const RtspServerConfig& config,
                             const std::string& depayloader) {
    constexpr std::int32_t kFrames = 20;
    RtspServer server{clock, config};
    if (!server.start()) {
        return 0;
    }
    RtspClient client{server.port(), depayloader};
    Session session{clock, server};
    if (!client.valid() ||
        !session.push_and_receive_until(client, [&] { return session.video_frames >= kFrames; })) {
        return 0;
    }
    return session.video_bytes / static_cast<std::size_t>(session.video_frames);
}

TEST_F(RtspServerTest, HigherJpegQualityMakesLargerFrames) {
    constexpr std::int32_t kLowQuality = 20;
    constexpr std::int32_t kHighQuality = 95;
    RtspServerConfig low = server_config();
    low.video.quality = kLowQuality;
    RtspServerConfig high = server_config();
    high.video.quality = kHighQuality;

    const std::size_t low_bytes = mean_video_bytes(clock_, low, kJpegDepayloader);
    const std::size_t high_bytes = mean_video_bytes(clock_, high, kJpegDepayloader);

    ASSERT_GT(low_bytes, 0U);
    ASSERT_GT(high_bytes, 0U);
    EXPECT_GT(high_bytes, low_bytes * 2);
}

// H.264 인코더와 디페이로더가 설치되어 있는지. 설치되어 있지 않은 환경에서는 H.264 송수신 테스트를
// 실행하지 않는다
bool has_h264_elements() {
    GstElementFactory* encoder = gst_element_factory_find(kH264EncoderFactory);
    GstElementFactory* depayloader = gst_element_factory_find(kH264Depayloader);
    const bool found = encoder != nullptr && depayloader != nullptr;
    gst_clear_object(&encoder);
    gst_clear_object(&depayloader);
    return found;
}

RtspServerConfig h264_server_config() {
    RtspServerConfig config = server_config();
    config.video = make_video_encode_config(VideoCodec::H264);
    return config;
}

TEST_F(RtspServerTest, ClientReceivesH264VideoAndMetadataTracks) {
    if (!has_h264_elements()) {
        GTEST_SKIP() << "x264enc 또는 rtph264depay 가 설치되어 있지 않음";
    }
    RtspServer server{clock_, h264_server_config()};
    ASSERT_TRUE(server.start());
    RtspClient client{server.port(), kH264Depayloader};
    ASSERT_TRUE(client.valid());
    Session session{clock_, server};

    ASSERT_TRUE(session.push_and_receive_until(
        client, [&] { return session.video_frames >= 3 && session.documents.size() >= 4; }));

    EXPECT_EQ(session.video_media_type, "video/x-h264");
    EXPECT_GT(server.stats().sent, 0U);
    EXPECT_EQ(server.stats().dropped_send_failed, 0U);
    // 메타데이터 트랙은 영상 트랙의 코덱과 무관하게 같은 문서를 송신한다
    for (std::size_t i = 1; i < session.documents.size(); ++i) {
        const std::optional<std::uint64_t> frame_id = frame_id_of(session.documents[i].text);
        ASSERT_TRUE(frame_id.has_value()) << "문서 " << i;
        EXPECT_EQ(session.documents[i].text, metadata_document(frame_id.value_or(0)));
    }
}

TEST_F(RtspServerTest, HigherH264BitrateMakesLargerFrames) {
    if (!has_h264_elements()) {
        GTEST_SKIP() << "x264enc 또는 rtph264depay 가 설치되어 있지 않음";
    }
    constexpr std::int32_t kLowBitrateKbps = 200;
    constexpr std::int32_t kHighBitrateKbps = 8000;
    RtspServerConfig low = h264_server_config();
    low.video.bitrate_kbps = kLowBitrateKbps;
    RtspServerConfig high = h264_server_config();
    high.video.bitrate_kbps = kHighBitrateKbps;

    const std::size_t low_bytes = mean_video_bytes(clock_, low, kH264Depayloader);
    const std::size_t high_bytes = mean_video_bytes(clock_, high, kH264Depayloader);

    ASSERT_GT(low_bytes, 0U);
    ASSERT_GT(high_bytes, 0U);
    EXPECT_GT(high_bytes, low_bytes * 2);
}

}  // namespace
}  // namespace argus::video_sender
