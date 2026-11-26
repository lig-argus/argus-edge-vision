#include "track_branch.hpp"

#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

#include <mutex>
#include <utility>

#include "decode_elements.hpp"

namespace argus::gcs::rtsp_gstreamer {

namespace {

constexpr QLatin1StringView kOnvifEncoding{"VND.ONVIF.METADATA"};

QString capsString(const GstStructure* structure, const char* field) {
    const gchar* value = gst_structure_get_string(structure, field);
    return value != nullptr ? QString::fromUtf8(value) : QString();
}

std::optional<std::uint64_t> ptsOf(GstBuffer* buffer) {
    const GstClockTime pts = GST_BUFFER_PTS(buffer);
    if (!GST_CLOCK_TIME_IS_VALID(pts)) {
        return std::nullopt;
    }
    return static_cast<std::uint64_t>(pts);
}

GstPadProbeReturn onVideoRtp(GstPad* /*pad*/, GstPadProbeInfo* info, gpointer user) {
    auto* sink = static_cast<ITrackSink*>(user);
    if ((GST_PAD_PROBE_INFO_TYPE(info) & GST_PAD_PROBE_TYPE_BUFFER_LIST) != 0) {
        GstBufferList* list = GST_PAD_PROBE_INFO_BUFFER_LIST(info);
        for (guint i = 0; i < gst_buffer_list_length(list); ++i) {
            sink->onVideoRtpPacket(gst_buffer_list_get(list, i));
        }
    } else {
        sink->onVideoRtpPacket(GST_PAD_PROBE_INFO_BUFFER(info));
    }
    return GST_PAD_PROBE_OK;
}

// RGBx 프레임 → DecodedFrame (버퍼를 돌려주기 전에 복사한다)
std::optional<rtsp::DecodedFrame> copyFrame(GstSample* sample) {
    GstVideoInfo info;
    GstVideoFrame frame;
    if (gst_video_info_from_caps(&info, gst_sample_get_caps(sample)) == FALSE ||
        gst_video_frame_map(&frame, &info, gst_sample_get_buffer(sample), GST_MAP_READ) == FALSE) {
        return std::nullopt;
    }
    rtsp::DecodedFrame decoded;
    decoded.width = GST_VIDEO_FRAME_WIDTH(&frame);
    decoded.height = GST_VIDEO_FRAME_HEIGHT(&frame);
    decoded.bytesPerLine = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    decoded.rgbx = QByteArray(static_cast<const char*>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0)),
                              static_cast<qsizetype>(decoded.bytesPerLine) * decoded.height);
    gst_video_frame_unmap(&frame);
    return decoded;
}

GstFlowReturn onVideoSample(GstAppSink* appsink, gpointer user) {
    GstSample* sample = gst_app_sink_pull_sample(appsink);
    if (sample == nullptr) {
        return GST_FLOW_OK;
    }
    std::optional<rtsp::DecodedFrame> decoded = copyFrame(sample);
    const std::optional<std::uint64_t> pts = ptsOf(gst_sample_get_buffer(sample));
    gst_sample_unref(sample);
    if (decoded) {
        static_cast<ITrackSink*>(user)->onDecodedFrame(std::move(*decoded), pts);
    }
    return GST_FLOW_OK;
}

GstFlowReturn onMetadataSample(GstAppSink* appsink, gpointer user) {
    GstSample* sample = gst_app_sink_pull_sample(appsink);
    if (sample == nullptr) {
        return GST_FLOW_OK;
    }
    static_cast<ITrackSink*>(user)->onMetadataRtpPacket(gst_sample_get_buffer(sample));
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

struct Branch {
    QString description;
    GstAppSinkCallbacks callbacks{};
    bool probeVideoRtp{false};
};

Branch branchFor(const QString& media, const QString& encoding) {
    Branch branch;
    if (media == QLatin1StringView("video")) {
        if (const std::optional<QString> decode = videoDecodeElements(encoding)) {
            branch.description =
                *decode + QLatin1StringView(
                              " ! videoconvert ! video/x-raw,format=RGBx"
                              " ! appsink name=sink sync=false max-buffers=2 drop=true");
            branch.callbacks.new_sample = &onVideoSample;
            branch.probeVideoRtp = true;
        }
    } else if (media == QLatin1StringView("application") &&
               encoding.compare(kOnvifEncoding, Qt::CaseInsensitive) == 0) {
        branch.description = QStringLiteral("appsink name=sink sync=false");
        branch.callbacks.new_sample = &onMetadataSample;
    }
    if (branch.description.isEmpty()) {
        branch.description = QStringLiteral("fakesink sync=false");  // 모르는 트랙은 받고 버린다
    }
    return branch;
}

}  // namespace

void ensureGstreamerInitialized() {
    static std::once_flag once;
    std::call_once(once, [] { gst_init(nullptr, nullptr); });
}

std::optional<QString> attachTrack(GstBin* pipeline, GstPad* pad, ITrackSink& sink) {
    GstCaps* caps = gst_pad_get_current_caps(pad);
    if (caps == nullptr) {
        caps = gst_pad_query_caps(pad, nullptr);
    }
    QString media;
    QString encoding;
    if (caps != nullptr && gst_caps_get_size(caps) > 0) {  // caps 를 모르면 모르는 트랙으로 둔다
        const GstStructure* structure = gst_caps_get_structure(caps, 0);
        media = capsString(structure, "media");
        encoding = capsString(structure, "encoding-name");
    }
    if (caps != nullptr) {
        gst_caps_unref(caps);
    }

    Branch branch = branchFor(media, encoding);
    GError* error = nullptr;
    GstElement* bin =
        gst_parse_bin_from_description(branch.description.toUtf8().constData(), TRUE, &error);
    if (bin == nullptr) {
        const QString text = error != nullptr ? QString::fromUtf8(error->message) : QString();
        g_clear_error(&error);
        return QStringLiteral("트랙 연결 실패 (%1): %2").arg(branch.description, text);
    }
    g_clear_error(&error);
    if (branch.callbacks.new_sample != nullptr) {
        GstElement* appsink = gst_bin_get_by_name(GST_BIN(bin), "sink");
        gst_app_sink_set_callbacks(GST_APP_SINK(appsink), &branch.callbacks, &sink, nullptr);
        gst_object_unref(appsink);
    }
    if (branch.probeVideoRtp) {
        // GstPadProbeType 은 비트 플래그 enum 이라 조합 값이 열거자에 없다
        // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
        const auto probeType = static_cast<GstPadProbeType>(GST_PAD_PROBE_TYPE_BUFFER |
                                                            GST_PAD_PROBE_TYPE_BUFFER_LIST);
        gst_pad_add_probe(pad, probeType, &onVideoRtp, &sink, nullptr);
    }
    gst_bin_add(pipeline, bin);
    gst_element_sync_state_with_parent(bin);
    GstPad* sinkPad = gst_element_get_static_pad(bin, "sink");
    const GstPadLinkReturn linked = gst_pad_link(pad, sinkPad);
    gst_object_unref(sinkPad);
    if (linked != GST_PAD_LINK_OK) {
        return QStringLiteral("트랙 연결 실패 (%1): pad 연결 오류 %2")
            .arg(branch.description)
            .arg(static_cast<std::int32_t>(linked));
    }
    return std::nullopt;
}

}  // namespace argus::gcs::rtsp_gstreamer
