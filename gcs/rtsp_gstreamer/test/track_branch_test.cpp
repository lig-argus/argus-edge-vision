// 트랙 출력 단자에 붙이는 처리 부분 (rtspsrc 없이 RTP 를 만드는 파이프라인에 붙여 확인한다).
// CI 에 있는 GStreamer 패키지(base, good)의 요소만 쓴다
#include "track_branch.hpp"

#include <gst/app/gstappsrc.h>

#include <QtTest>
#include <mutex>
#include <vector>

#include "decode_elements.hpp"

namespace argus::gcs::rtsp_gstreamer {

namespace {

constexpr GstClockTime kEosTimeout = 10 * GST_SECOND;

class RecordingSink : public ITrackSink {
public:
    void onVideoRtpPacket(GstBuffer* /*buffer*/) override {
        const std::scoped_lock lock(mutex_);
        ++videoPackets_;
    }

    void onDecodedFrame(rtsp::DecodedFrame frame, std::optional<std::uint64_t> pts) override {
        const std::scoped_lock lock(mutex_);
        frames_.push_back(std::move(frame));
        framesWithPts_ += pts ? 1U : 0U;
    }

    void onMetadataRtpPacket(GstBuffer* buffer) override {
        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_READ) == FALSE) {
            return;
        }
        const std::scoped_lock lock(mutex_);
        packets_.emplace_back(reinterpret_cast<const char*>(map.data),
                              static_cast<qsizetype>(map.size));
        gst_buffer_unmap(buffer, &map);
    }

    std::size_t videoPackets() const {
        const std::scoped_lock lock(mutex_);
        return videoPackets_;
    }
    std::vector<rtsp::DecodedFrame> frames() const {
        const std::scoped_lock lock(mutex_);
        return frames_;
    }
    std::size_t framesWithPts() const {
        const std::scoped_lock lock(mutex_);
        return framesWithPts_;
    }
    std::vector<QByteArray> packets() const {
        const std::scoped_lock lock(mutex_);
        return packets_;
    }

private:
    mutable std::mutex mutex_;
    std::size_t videoPackets_{0};
    std::vector<rtsp::DecodedFrame> frames_;
    std::size_t framesWithPts_{0};
    std::vector<QByteArray> packets_;
};

// 재생 중인 파이프라인의 스트림 끝(EOS)이나 오류까지 기다린 뒤 멈춘다. EOS 로 끝나면 true
bool waitForEos(GstElement* pipeline) {
    GstBus* bus = gst_element_get_bus(pipeline);
    // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange): 비트 플래그 조합
    const auto types = static_cast<GstMessageType>(static_cast<std::uint32_t>(GST_MESSAGE_EOS) |
                                                   static_cast<std::uint32_t>(GST_MESSAGE_ERROR));
    GstMessage* message = gst_bus_timed_pop_filtered(bus, kEosTimeout, types);
    const bool eos = message != nullptr && GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS;
    if (message != nullptr) {
        gst_message_unref(message);
    }
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    return eos;
}

// 이름이 name 인 요소의 출력 단자에 트랙 처리 부분을 붙인다
std::optional<QString> attachTo(GstElement* pipeline, const char* name, ITrackSink& sink) {
    GstElement* element = gst_bin_get_by_name(GST_BIN(pipeline), name);
    GstPad* pad = gst_element_get_static_pad(element, "src");
    std::optional<QString> error = attachTrack(GST_BIN(pipeline), pad, sink);
    gst_object_unref(pad);
    gst_object_unref(element);
    return error;
}

GstElement* launch(const char* description) {
    GError* error = nullptr;
    GstElement* pipeline = gst_parse_launch(description, &error);
    g_clear_error(&error);
    return pipeline;
}

// appsrc ! capsfilter name=out. caps 는 트랙 처리 부분이 읽는 SDP 값 (media, encoding-name).
// 재생 전 appsrc 의 caps 조회는 ANY 라서 capsfilter 의 출력 단자에 붙인다
GstElement* appsrcPipeline(const char* caps) {
    GstElement* pipeline = gst_pipeline_new("test");
    GstElement* source = gst_element_factory_make("appsrc", "src");
    GstElement* filter = gst_element_factory_make("capsfilter", "out");
    GstCaps* parsed = gst_caps_from_string(caps);
    g_object_set(source, "caps", parsed, "format", GST_FORMAT_TIME, nullptr);
    g_object_set(filter, "caps", parsed, nullptr);
    gst_caps_unref(parsed);
    gst_bin_add_many(GST_BIN(pipeline), source, filter, nullptr);
    gst_element_link(source, filter);
    return pipeline;
}

}  // namespace

class TrackBranchTest : public QObject {
    Q_OBJECT

private slots:
    void initTestCase() { ensureGstreamerInitialized(); }

    void decodeElementsByEncodingName() {
        QCOMPARE(videoDecodeElements(QStringLiteral("H264")),
                 std::optional<QString>(QStringLiteral("rtph264depay ! avdec_h264")));
        QCOMPARE(videoDecodeElements(QStringLiteral("jpeg")),
                 std::optional<QString>(QStringLiteral("rtpjpegdepay ! jpegdec")));
        QVERIFY(!videoDecodeElements(QStringLiteral("H265")).has_value());
    }

    // RTP/JPEG 영상 트랙: 해독한 프레임을 RGBx 로 받고, 해독 전 RTP 패킷도 본다.
    // RFC 2435 는 4:2:0 JPEG 만 다루므로 I420 으로 인코딩한다 (docs/spec/video.md 2.1)
    void decodesJpegVideoTrack() {
        GstElement* pipeline = launch(
            "videotestsrc num-buffers=3 ! "
            "video/x-raw,format=I420,width=64,height=48,framerate=30/1 ! jpegenc ! rtpjpegpay "
            "name=pay");
        QVERIFY(pipeline != nullptr);
        RecordingSink sink;

        QVERIFY(!attachTo(pipeline, "pay", sink).has_value());
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
        QVERIFY(waitForEos(pipeline));
        gst_object_unref(pipeline);

        const std::vector<rtsp::DecodedFrame> frames = sink.frames();
        QCOMPARE(frames.size(), std::size_t{3});
        QCOMPARE(frames.front().width, std::int32_t{64});
        QCOMPARE(frames.front().height, std::int32_t{48});
        QVERIFY(frames.front().bytesPerLine >= 64 * 4);
        QCOMPARE(frames.front().rgbx.size(),
                 static_cast<qsizetype>(frames.front().bytesPerLine) * 48);
        QVERIFY(!frames.front().utcNs.has_value());  // UTC 는 수신기가 채운다
        QCOMPARE(sink.framesWithPts(), std::size_t{3});
        QVERIFY(sink.videoPackets() >= 3);
    }

    // 메타데이터 트랙: RTP 패킷을 그대로 넘긴다
    void passesMetadataPackets() {
        GstElement* pipeline = appsrcPipeline(
            "application/x-rtp,media=application,encoding-name=VND.ONVIF.METADATA,"
            "clock-rate=90000,payload=107");
        QVERIFY(pipeline != nullptr);
        RecordingSink sink;
        QVERIFY(!attachTo(pipeline, "out", sink).has_value());
        gst_element_set_state(pipeline, GST_STATE_PLAYING);  // 멈춘 appsrc 는 넣은 버퍼를 버린다
        GstElement* source = gst_bin_get_by_name(GST_BIN(pipeline), "src");
        const std::vector<QByteArray> sent{
            QByteArray("\x80\x6B\x00\x01\x00\x00\x00\x64xxxx<a>", 15),
            QByteArray("\x80\xEB\x00\x02\x00\x00\x00\x64xxxx</a>", 16)};
        for (const QByteArray& packet : sent) {
            GstBuffer* buffer =
                gst_buffer_new_memdup(packet.constData(), static_cast<gsize>(packet.size()));
            gst_app_src_push_buffer(GST_APP_SRC(source), buffer);
        }
        gst_app_src_end_of_stream(GST_APP_SRC(source));
        gst_object_unref(source);

        QVERIFY(waitForEos(pipeline));
        gst_object_unref(pipeline);

        QCOMPARE(sink.packets(), sent);
        QVERIFY(sink.frames().empty());
    }

    // 모르는 트랙(음성 등)은 받고 버린다
    void unknownTrackIsDiscarded() {
        GstElement* pipeline = appsrcPipeline(
            "application/x-rtp,media=audio,encoding-name=OPUS,clock-rate=48000,payload=111");
        QVERIFY(pipeline != nullptr);
        RecordingSink sink;

        QVERIFY(!attachTo(pipeline, "out", sink).has_value());
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
        GstElement* source = gst_bin_get_by_name(GST_BIN(pipeline), "src");
        gst_app_src_end_of_stream(GST_APP_SRC(source));
        gst_object_unref(source);
        QVERIFY(waitForEos(pipeline));
        gst_object_unref(pipeline);

        QVERIFY(sink.packets().empty());
        QVERIFY(sink.frames().empty());
    }
};

}  // namespace argus::gcs::rtsp_gstreamer

QTEST_GUILESS_MAIN(argus::gcs::rtsp_gstreamer::TrackBranchTest)
#include "track_branch_test.moc"
