#include "argus/gcs/rtsp_gstreamer/gst_rtsp_receiver.hpp"

#include <gst/gst.h>
#include <gst/rtp/gstrtcpbuffer.h>
#include <gst/rtp/gstrtpbuffer.h>

#include <QTimer>
#include <mutex>
#include <set>
#include <utility>

#include "argus/gcs/rtsp/metadata_reassembler.hpp"
#include "argus/gcs/rtsp/rtcp_clock.hpp"
#include "argus/gcs/rtsp/rtp_frame_times.hpp"
#include "argus/gcs/rtsp/rtp_sequence.hpp"
#include "track_branch.hpp"

namespace argus::gcs::rtsp_gstreamer {

namespace {

constexpr std::int32_t kBusPollMs = 50;

struct RtpHeader {
    std::uint32_t ssrc{0};
    std::uint16_t seq{0};
    std::uint32_t timestamp{0};
};

std::optional<RtpHeader> readRtpHeader(GstBuffer* buffer) {
    // GST_RTP_BUFFER_INIT 이 맵 플래그 enum 을 0 으로 채운다 (GStreamer 매크로)
    // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
    GstRTPBuffer rtp = GST_RTP_BUFFER_INIT;
    if (gst_rtp_buffer_map(buffer, GST_MAP_READ, &rtp) == FALSE) {
        return std::nullopt;
    }
    const RtpHeader header{gst_rtp_buffer_get_ssrc(&rtp), gst_rtp_buffer_get_seq(&rtp),
                           gst_rtp_buffer_get_timestamp(&rtp)};
    gst_rtp_buffer_unmap(&rtp);
    return header;
}

std::optional<std::uint64_t> ptsOf(GstBuffer* buffer) {
    const GstClockTime pts = GST_BUFFER_PTS(buffer);
    if (!GST_CLOCK_TIME_IS_VALID(pts)) {
        return std::nullopt;
    }
    return static_cast<std::uint64_t>(pts);
}

}  // namespace

struct GstRtspReceiver::Impl : public ITrackSink {
    Impl(GstRtspReceiver* receiver, const core::IClock& clockSource)
        : owner(receiver), clock(clockSource) {}

    GstRtspReceiver* owner;
    const core::IClock& clock;
    GstElement* pipeline{nullptr};
    QTimer busTimer;

    // GStreamer 스레드와 함께 쓰는 값 (mutex)
    mutable std::mutex mutex;
    std::uint64_t session{0};  // start() 마다 하나씩. 앞 연결에서 넘어온 늦은 알림을 버린다
    rtsp::RtcpClock rtcpClock;
    rtsp::RtpFrameTimes frameTimes;
    std::set<guint> rtcpSessions;  // on-receiving-rtcp 를 연결한 수신 세션
    rtsp::MetadataReassembler reassembler;
    rtsp::ReceiverCounters counters;
    rtsp::RtpSequenceCounter videoSequence;
    rtsp::RtpSequenceCounter metadataSequence;

    // ---- rtspsrc: 트랙마다 출력 단자가 생긴다
    static void onPadAdded(GstElement* /*source*/, GstPad* pad, gpointer user) {
        auto* self = static_cast<Impl*>(user);
        if (const std::optional<QString> error = attachTrack(GST_BIN(self->pipeline), pad, *self)) {
            self->post([self, text = *error] {
                self->owner->setState(rtsp::ReceiverState::Failed, text);
            });
        }
    }

    // ---- RTCP SR: rtspsrc 가 만든 rtpbin 의 수신 세션마다 연결한다
    static void onNewManager(GstElement* /*source*/, GstElement* manager, gpointer user) {
        g_signal_connect(manager, "on-new-ssrc", G_CALLBACK(&Impl::onNewSsrc), user);
    }

    static void onNewSsrc(GstElement* manager, guint rtpSession, guint /*ssrc*/, gpointer user) {
        auto* self = static_cast<Impl*>(user);
        {
            const std::scoped_lock lock(self->mutex);
            if (!self->rtcpSessions.insert(rtpSession).second) {
                return;
            }
        }
        GObject* internal = nullptr;
        g_signal_emit_by_name(manager, "get-internal-session", rtpSession, &internal);
        if (internal != nullptr) {
            g_signal_connect(internal, "on-receiving-rtcp", G_CALLBACK(&Impl::onReceivingRtcp),
                             user);
            g_object_unref(internal);
        }
    }

    static void onReceivingRtcp(GObject* /*rtpSession*/, GstBuffer* buffer, gpointer user) {
        auto* self = static_cast<Impl*>(user);
        GstRTCPBuffer rtcp = GST_RTCP_BUFFER_INIT;
        if (gst_rtcp_buffer_map(buffer, GST_MAP_READ, &rtcp) == FALSE) {
            return;
        }
        GstRTCPPacket packet;
        for (gboolean more = gst_rtcp_buffer_get_first_packet(&rtcp, &packet); more != FALSE;
             more = gst_rtcp_packet_move_to_next(&packet)) {
            if (gst_rtcp_packet_get_type(&packet) != GST_RTCP_TYPE_SR) {
                continue;
            }
            guint32 ssrc = 0;
            guint64 ntpTime = 0;
            guint32 rtpTime = 0;
            guint32 packets = 0;
            guint32 octets = 0;
            gst_rtcp_packet_sr_get_sender_info(&packet, &ssrc, &ntpTime, &rtpTime, &packets,
                                               &octets);
            const std::scoped_lock lock(self->mutex);
            self->rtcpClock.onSenderReport(ssrc, ntpTime, rtpTime);
        }
        gst_rtcp_buffer_unmap(&rtcp);
    }

    // ---- ITrackSink (GStreamer 스레드)
    void onVideoRtpPacket(GstBuffer* buffer) override {
        const std::optional<RtpHeader> header = readRtpHeader(buffer);
        if (!header) {
            return;
        }
        const std::int64_t nowNs = clock.nowMonotonicNs();
        const std::scoped_lock lock(mutex);
        counters.bytes += gst_buffer_get_size(buffer);
        videoSequence.push(header->ssrc, header->seq, nowNs);
        frameTimes.recordPacket(ptsOf(buffer), header->ssrc, header->timestamp);
    }

    void onDecodedFrame(rtsp::DecodedFrame frame, std::optional<std::uint64_t> pts) override {
        {
            const std::scoped_lock lock(mutex);
            if (pts) {
                if (const std::optional<rtsp::RtpFrameTime> time = frameTimes.takeFrame(*pts)) {
                    frame.utcNs = rtcpClock.utcNs(time->ssrc, time->rtpTime);
                }
            }
            ++counters.frames;
            counters.framesWithUtc += frame.utcNs ? 1U : 0U;
        }
        post([self = this, decoded = std::move(frame)] {
            emit self->owner->frameReceived(decoded);
        });
    }

    void onMetadataRtpPacket(GstBuffer* buffer) override {
        const std::optional<RtpHeader> header = readRtpHeader(buffer);
        const std::int64_t nowNs = clock.nowMonotonicNs();
        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_READ) == FALSE) {
            return;
        }
        const QByteArray packet(reinterpret_cast<const char*>(map.data),
                                static_cast<qsizetype>(map.size));
        gst_buffer_unmap(buffer, &map);
        std::optional<rtsp::MetadataDocument> document;
        {
            const std::scoped_lock lock(mutex);
            counters.bytes += static_cast<std::uint64_t>(packet.size());
            if (header) {
                metadataSequence.push(header->ssrc, header->seq, nowNs);
            }
            document = reassembler.push(packet);
            counters.droppedDocuments = reassembler.droppedDocuments();
        }
        if (!document) {
            return;
        }
        std::optional<rtsp::FrameMetadata> metadata = rtsp::parseOnvifMetadata(document->xml);
        {
            const std::scoped_lock lock(mutex);
            ++(metadata ? counters.metadataDocuments : counters.badDocuments);
        }
        if (metadata) {
            post([self = this, found = std::move(*metadata)] {
                emit self->owner->metadataReceived(found);
            });
        }
    }

    // GStreamer 스레드 → 이 객체의 스레드. 그 사이 다시 연결했으면(session 이 바뀜) 버린다.
    // 객체가 먼저 사라지면 Qt 가 호출을 버린다
    template <typename Function>
    void post(Function function) {
        std::uint64_t postedSession = 0;
        {
            const std::scoped_lock lock(mutex);
            postedSession = session;
        }
        QMetaObject::invokeMethod(
            owner,
            [self = this, postedSession, function = std::move(function)] {
                if (self->currentSession() == postedSession) {
                    function();
                }
            },
            Qt::QueuedConnection);
    }

    std::uint64_t currentSession() const {
        const std::scoped_lock lock(mutex);
        return session;
    }

    // ---- GStreamer 버스 (Qt 이벤트 루프에서 읽는다)
    void pollBus() {
        if (pipeline == nullptr) {
            return;
        }
        GstBus* bus = gst_element_get_bus(pipeline);
        while (GstMessage* message = gst_bus_pop(bus)) {
            handleMessage(message);
            gst_message_unref(message);
        }
        gst_object_unref(bus);
        // 오류나 스트림 끝: 파이프라인을 정리한다 (다시 받으려면 start())
        if (owner->state() == rtsp::ReceiverState::Failed) {
            teardown();
        }
    }

    // GstMessageType 은 비트 플래그 enum 이라 쓰는 세 종류만 if 로 가른다 (그 밖의 메시지는 쓰지
    // 않는다)
    void handleMessage(GstMessage* message) {
        const GstMessageType type = GST_MESSAGE_TYPE(message);
        if (type == GST_MESSAGE_ERROR) {
            GError* error = nullptr;
            gst_message_parse_error(message, &error, nullptr);
            const QString text =
                error != nullptr ? QString::fromUtf8(error->message) : QStringLiteral("오류");
            g_clear_error(&error);
            owner->setState(rtsp::ReceiverState::Failed, text);
        } else if (type == GST_MESSAGE_EOS) {
            owner->setState(rtsp::ReceiverState::Failed, QStringLiteral("스트림이 끝남"));
        } else if (type == GST_MESSAGE_STATE_CHANGED &&
                   GST_MESSAGE_SRC(message) == GST_OBJECT(pipeline)) {
            GstState next = GST_STATE_NULL;
            gst_message_parse_state_changed(message, nullptr, &next, nullptr);
            if (next == GST_STATE_PLAYING) {
                owner->setState(rtsp::ReceiverState::Playing);
            }
        }
    }

    void teardown() {
        busTimer.stop();
        if (pipeline != nullptr) {
            // NULL 로 바꾸면 GStreamer 스레드가 모두 멈춘 뒤 돌아온다 (그 뒤 콜백 없음)
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
            pipeline = nullptr;
        }
        const std::scoped_lock lock(mutex);
        ++session;
        rtcpClock = rtsp::RtcpClock{};
        frameTimes.clear();
        rtcpSessions.clear();
        reassembler = rtsp::MetadataReassembler{};
    }
};

GstRtspReceiver::GstRtspReceiver(QString url, const core::IClock& clock, std::int32_t latencyMs,
                                 QObject* parent)
    : rtsp::IRtspReceiver(parent),
      url_(std::move(url)),
      latencyMs_(latencyMs),
      impl_(std::make_unique<Impl>(this, clock)) {
    impl_->busTimer.setInterval(kBusPollMs);
    connect(&impl_->busTimer, &QTimer::timeout, this, [this] { impl_->pollBus(); });
}

GstRtspReceiver::~GstRtspReceiver() { impl_->teardown(); }

void GstRtspReceiver::start() {
    ensureGstreamerInitialized();
    impl_->teardown();
    {
        const std::scoped_lock lock(impl_->mutex);
        impl_->counters = rtsp::ReceiverCounters{};
        impl_->videoSequence = rtsp::RtpSequenceCounter{};
        impl_->metadataSequence = rtsp::RtpSequenceCounter{};
    }
    GstElement* pipeline = gst_pipeline_new("argus-rtsp");
    GstElement* source = gst_element_factory_make("rtspsrc", "source");
    if (pipeline == nullptr || source == nullptr) {
        if (pipeline != nullptr) {
            gst_object_unref(pipeline);
        }
        setState(rtsp::ReceiverState::Failed, QStringLiteral("GStreamer rtspsrc 요소가 없음"));
        return;
    }
    g_object_set(source, "location", url_.toUtf8().constData(), "latency",
                 static_cast<guint>(latencyMs_), nullptr);
    g_signal_connect(source, "pad-added", G_CALLBACK(&Impl::onPadAdded), impl_.get());
    g_signal_connect(source, "new-manager", G_CALLBACK(&Impl::onNewManager), impl_.get());
    gst_bin_add(GST_BIN(pipeline), source);
    impl_->pipeline = pipeline;
    setState(rtsp::ReceiverState::Connecting);
    impl_->busTimer.start();
    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        impl_->pollBus();  // 실패 이유를 읽는다
        if (state_ != rtsp::ReceiverState::Failed) {
            setState(rtsp::ReceiverState::Failed, QStringLiteral("재생을 시작하지 못함"));
            impl_->teardown();
        }
    }
}

void GstRtspReceiver::stop() {
    impl_->teardown();
    setState(rtsp::ReceiverState::Stopped);
}

rtsp::ReceiverCounters GstRtspReceiver::counters() const {
    const std::scoped_lock lock(impl_->mutex);
    rtsp::ReceiverCounters out = impl_->counters;
    out.videoRtp = impl_->videoSequence.stats();
    out.metadataRtp = impl_->metadataSequence.stats();
    return out;
}

void GstRtspReceiver::setState(rtsp::ReceiverState state, const QString& error) {
    lastError_ = state == rtsp::ReceiverState::Failed ? error : QString();
    if (state_ != state) {
        state_ = state;
        emit stateChanged(state);
    }
}

}  // namespace argus::gcs::rtsp_gstreamer
