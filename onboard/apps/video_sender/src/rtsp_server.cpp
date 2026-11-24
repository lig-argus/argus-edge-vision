#include "argus/video_sender/rtsp_server.hpp"

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/video/video.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <future>
#include <iostream>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include "argus/video_sender/stream_timing.hpp"

namespace argus::video_sender {
namespace {

// 파이프라인 안의 요소 이름. gst-rtsp-server 는 이름이 pay0, pay1 인 페이로더를 트랙으로 사용한다
constexpr const char* kVideoSourceName = "video";
constexpr const char* kMetadataSourceName = "meta";
constexpr const char* kVideoPayloaderName = "pay0";
constexpr const char* kMetadataPayloaderName = "pay1";

// 인코더 요소의 factory 이름과, 설정값을 지정하는 속성 이름
constexpr const char* kJpegEncoderFactory = "jpegenc";
constexpr const char* kJpegQualityProperty = "quality";
constexpr const char* kH264EncoderFactory = "x264enc";
constexpr const char* kH264BitrateProperty = "bitrate";  // kbit/s

// 영상 appsrc 가 보관하는 프레임 수의 상한. 초과하면 오래된 것부터 폐기한다. 값은 프레임 토픽의 큐
// 길이와 같다 (process_interfaces.md 5.1). 메타데이터 appsrc 에는 적용하지 않는다 (문서 하나가
// 여러 버퍼이므로 개수로 제한하면 문서의 일부만 폐기된다). 메타데이터는 appsrc 의 기본 상한
// (max-bytes)을 사용한다
constexpr std::int32_t kMaxQueuedFrames = 4;

void log_error(const std::string& message) { std::cerr << "RtspServer: " << message << "\n"; }

std::string launch_description(const RtspServerConfig& config) {
    const std::string source_options = " is-live=true format=time leaky-type=downstream";
    return "( appsrc name=" + std::string{kVideoSourceName} + source_options +
           " max-buffers=" + std::to_string(kMaxQueuedFrames) + " ! " +
           config.video.gstreamer_elements + " name=" + kVideoPayloaderName +
           " pt=" + std::to_string(config.video.payload_type) +
           " appsrc name=" + std::string{kMetadataSourceName} + source_options +
           " caps=\"application/x-rtp,media=application,clock-rate=" +
           std::to_string(kRtpClockRate) + ",encoding-name=VND.ONVIF.METADATA,payload=" +
           std::to_string(config.metadata_rtp.payload_type) +
           "\" ! rtppassthroughpay name=" + kMetadataPayloaderName + " )";
}

bool has_element(GstElement* pipeline, const char* name) {
    GstElement* element = gst_bin_get_by_name(GST_BIN(pipeline), name);
    if (element == nullptr) {
        return false;
    }
    gst_object_unref(element);
    return true;
}

// 파이프라인이 생성된 뒤 인코더 요소에 지정하는 값 하나 (JPEG 품질 또는 H.264 비트레이트)
struct EncoderSetting {
    const char* factory;
    const char* property;
    std::int32_t value;
};

// 설정에서 인코더에 지정할 값을 구한다. 지정한 값이 없으면 값이 없는 optional 을 반환한다
std::optional<EncoderSetting> encoder_setting_of(const VideoEncodeConfig& video) {
    if (video.quality.has_value()) {
        return EncoderSetting{kJpegEncoderFactory, kJpegQualityProperty, video.quality.value()};
    }
    if (video.bitrate_kbps.has_value()) {
        return EncoderSetting{kH264EncoderFactory, kH264BitrateProperty,
                              video.bitrate_kbps.value()};
    }
    return std::nullopt;
}

// 설정값들이 서로 맞는지 확인한다. 맞지 않으면 원인을 기록하고 false 를 반환한다
bool is_consistent(const RtspServerConfig& config) {
    if (config.video.quality.has_value() && config.video.codec != VideoCodec::Jpeg) {
        log_error("quality 는 codec 이 Jpeg 인 경우에만 지원함");
        return false;
    }
    if (config.video.bitrate_kbps.has_value() && config.video.codec != VideoCodec::H264) {
        log_error("bitrate_kbps 는 codec 이 H264 인 경우에만 지원함");
        return false;
    }
    if (config.video.payload_type == config.metadata_rtp.payload_type) {
        log_error("영상 트랙과 메타데이터 트랙의 payload type 이 같음");
        return false;
    }
    return true;
}

bool has_element_of_factory(GstElement* pipeline, const char* factory) {
    GstIterator* elements = gst_bin_iterate_all_by_element_factory_name(GST_BIN(pipeline), factory);
    GValue item = G_VALUE_INIT;
    const bool found = gst_iterator_next(elements, &item) == GST_ITERATOR_OK;
    g_value_unset(&item);
    gst_iterator_free(elements);
    return found;
}

// launch 문자열로 파이프라인을 구성할 수 있고, RtspServer 가 이름으로 찾는 요소가 모두 있는지
// 확인한다. gst-rtsp-server 는 클라이언트가 접속할 때 파이프라인을 구성하므로, 여기서 확인하지
// 않으면 설정 오류가 start() 에서 드러나지 않는다
bool is_valid_launch(const std::string& description,
                     const std::optional<EncoderSetting>& encoder_setting) {
    GError* error = nullptr;
    GstElement* pipeline = gst_parse_launch(description.c_str(), &error);
    const bool parsed = pipeline != nullptr && error == nullptr && GST_IS_BIN(pipeline);
    const bool valid = parsed && has_element(pipeline, kVideoSourceName) &&
                       has_element(pipeline, kMetadataSourceName) &&
                       has_element(pipeline, kVideoPayloaderName) &&
                       has_element(pipeline, kMetadataPayloaderName);
    // 값을 지정할 인코더 요소가 파이프라인에 없으면 지정한 값이 적용되지 않는다
    const bool has_encoder = !valid || !encoder_setting.has_value() ||
                             has_element_of_factory(pipeline, encoder_setting.value().factory);
    if (error != nullptr) {
        log_error(std::string{"파이프라인을 구성할 수 없음: "} + error->message);
    } else if (!valid) {
        log_error("파이프라인에 필요한 요소가 없음 (gstreamer_elements 의 마지막 요소 확인)");
    } else if (!has_encoder) {
        log_error(std::string{"gstreamer_elements 에 값을 지정할 인코더 요소가 없음: "} +
                  encoder_setting.value().factory);
    }
    g_clear_error(&error);
    gst_clear_object(&pipeline);
    return valid && has_encoder;
}

// 프레임의 픽셀을 GStreamer 의 GRAY8 버퍼로 복사한다. 줄의 바이트 수(stride)가 서로 다를 수
// 있으므로 줄 단위로 복사한다. 프레임의 크기 정보가 픽셀 수와 맞지 않으면 nullptr 을 반환한다
GstBuffer* make_video_buffer(const perception_api::Frame& frame) {
    if (frame.width == 0 || frame.height == 0 || frame.stride < frame.width ||
        frame.pixels.size() < frame.expected_pixel_bytes()) {
        return nullptr;
    }
    GstVideoInfo info;
    gst_video_info_set_format(&info, GST_VIDEO_FORMAT_GRAY8, frame.width, frame.height);
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, GST_VIDEO_INFO_SIZE(&info), nullptr);
    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE) == FALSE) {
        gst_buffer_unref(buffer);
        return nullptr;
    }
    const auto row_bytes = static_cast<std::size_t>(GST_VIDEO_INFO_PLANE_STRIDE(&info, 0));
    for (std::uint32_t y = 0; y < frame.height; ++y) {
        std::memcpy(map.data + y * row_bytes,
                    frame.pixels.data() + static_cast<std::size_t>(y) * frame.stride, frame.width);
    }
    gst_buffer_unmap(buffer, &map);
    return buffer;
}

GstCaps* video_caps(const perception_api::Frame& frame) {
    return gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "GRAY8", "width", G_TYPE_INT,
                               static_cast<gint>(frame.width), "height", G_TYPE_INT,
                               static_cast<gint>(frame.height), "framerate", GST_TYPE_FRACTION, 0,
                               1, nullptr);
}

enum class PushResult : std::uint8_t { Sent, NoClient, NotPlaying, InvalidFrame, SendFailed };

// 현재 파이프라인의 appsrc 두 개와, 그 appsrc 에 대한 상태를 함께 관리한다.
// RTSP 요청을 처리하는 스레드(set, release)와 push() 를 호출하는 스레드가 함께 사용하므로 모든
// 멤버를 mutex 로 보호한다. GStreamer 신호의 handler 가 RtspServer 보다 늦게 호출될 수 있으므로
// shared_ptr 로 보관한다
class MediaSources {
public:
    MediaSources() = default;
    ~MediaSources() { release(); }
    MediaSources(const MediaSources&) = delete;
    MediaSources& operator=(const MediaSources&) = delete;
    MediaSources(MediaSources&&) = delete;
    MediaSources& operator=(MediaSources&&) = delete;

    // owner(파이프라인을 가진 GstRTSPMedia)의 appsrc 로 교체한다. video 와 metadata 의 참조를
    // 넘겨받는다
    void set(const void* owner, GstElement* video, GstElement* metadata) {
        const std::scoped_lock lock(mutex_);
        release_locked();
        owner_ = owner;
        video_ = video;
        metadata_ = metadata;
    }

    // owner 의 appsrc 를 보관 중인 경우에만 해제한다. 클라이언트가 끊은 직후 다시 접속하면 이전
    // 파이프라인의 정리가 새 파이프라인의 생성보다 늦게 끝날 수 있으며, 그때 새 파이프라인의
    // appsrc 를 해제하지 않기 위함이다
    void release_if_owned_by(const void* owner) {
        const std::scoped_lock lock(mutex_);
        if (owner_ == owner) {
            release_locked();
        }
    }

    void release() {
        const std::scoped_lock lock(mutex_);
        release_locked();
    }

    bool has_pipeline() {
        const std::scoped_lock lock(mutex_);
        return video_ != nullptr && metadata_ != nullptr;
    }

    // 프레임과 메타데이터 RTP 패킷을 같은 PTS 로 파이프라인에 넘긴다. 영상을 먼저 넘긴다
    PushResult push(const perception_api::Frame& frame,
                    const std::vector<MetadataRtpPacketizer::Packet>& metadata_packets,
                    std::int64_t now_monotonic_ns) {
        const std::scoped_lock lock(mutex_);
        if (video_ == nullptr || metadata_ == nullptr) {
            return PushResult::NoClient;
        }
        const std::optional<GstClockTime> running_time = running_time_now();
        if (!running_time.has_value()) {
            return PushResult::NotPlaying;
        }
        GstBuffer* video_buffer = make_video_buffer(frame);
        if (video_buffer == nullptr) {
            return PushResult::InvalidFrame;
        }
        const GstClockTime pts = frame_running_time_ns(running_time.value(), now_monotonic_ns,
                                                       frame.received_monotonic_ns);
        const bool video_ok = push_video(frame, video_buffer, pts);
        const bool metadata_ok = push_metadata(metadata_packets, pts);
        return video_ok && metadata_ok ? PushResult::Sent : PushResult::SendFailed;
    }

private:
    void release_locked() {
        owner_ = nullptr;
        gst_clear_object(&video_);
        gst_clear_object(&metadata_);
        caps_width_ = 0;
        caps_height_ = 0;
    }

    // 파이프라인의 현재 running time. 재생 상태가 아니어서 구할 수 없으면 값이 없다
    std::optional<GstClockTime> running_time_now() const {
        GstClock* pipeline_clock = gst_element_get_clock(video_);
        if (pipeline_clock == nullptr) {
            return std::nullopt;
        }
        const GstClockTime now = gst_clock_get_time(pipeline_clock);
        const GstClockTime base = gst_element_get_base_time(video_);
        gst_object_unref(pipeline_clock);
        if (now < base) {
            return std::nullopt;
        }
        return now - base;
    }

    bool push_video(const perception_api::Frame& frame, GstBuffer* buffer, GstClockTime pts) {
        if (frame.width != caps_width_ || frame.height != caps_height_) {
            GstCaps* caps = video_caps(frame);
            gst_app_src_set_caps(GST_APP_SRC(video_), caps);
            gst_caps_unref(caps);
            caps_width_ = frame.width;
            caps_height_ = frame.height;
        }
        GST_BUFFER_PTS(buffer) = pts;
        return gst_app_src_push_buffer(GST_APP_SRC(video_), buffer) == GST_FLOW_OK;
    }

    bool push_metadata(const std::vector<MetadataRtpPacketizer::Packet>& packets,
                       GstClockTime pts) {
        if (packets.empty()) {
            return true;
        }
        GstBufferList* list = gst_buffer_list_new_sized(static_cast<guint>(packets.size()));
        for (const MetadataRtpPacketizer::Packet& packet : packets) {
            GstBuffer* buffer = gst_buffer_new_allocate(nullptr, packet.size(), nullptr);
            gst_buffer_fill(buffer, 0, packet.data(), packet.size());
            GST_BUFFER_PTS(buffer) = pts;
            gst_buffer_list_add(list, buffer);
        }
        return gst_app_src_push_buffer_list(GST_APP_SRC(metadata_), list) == GST_FLOW_OK;
    }

    std::mutex mutex_;
    const void* owner_ = nullptr;     // video_ 와 metadata_ 가 속한 GstRTSPMedia. 비교에만 사용한다
    GstElement* video_ = nullptr;     // 영상 트랙의 appsrc. 파이프라인이 없으면 nullptr
    GstElement* metadata_ = nullptr;  // 메타데이터 트랙의 appsrc

    // video_ 에 설정한 caps 의 영상 크기. video_ 가 바뀌면 0 으로 되돌려 새 appsrc 에 caps 를 다시
    // 설정하게 한다
    std::uint32_t caps_width_ = 0;
    std::uint32_t caps_height_ = 0;
};

// GStreamer 신호의 handler 에 넘기는 값. 생성 뒤에는 변경하지 않는다
struct MediaContext {
    std::shared_ptr<MediaSources> sources;
    std::optional<EncoderSetting> encoder_setting;
};

void delete_media_context(gpointer data, GClosure* /*closure*/) {
    delete static_cast<MediaContext*>(data);
}

void connect_with_context(gpointer instance, const char* signal, GCallback handler,
                          const MediaContext& context) {
    g_signal_connect_data(instance, signal, handler, new MediaContext(context),
                          delete_media_context, static_cast<GConnectFlags>(0));
}

void on_media_unprepared(GstRTSPMedia* media, gpointer data) {
    static_cast<MediaContext*>(data)->sources->release_if_owned_by(media);
}

void apply_encoder_setting(GstElement* pipeline, const EncoderSetting& setting) {
    GstIterator* encoders =
        gst_bin_iterate_all_by_element_factory_name(GST_BIN(pipeline), setting.factory);
    GValue item = G_VALUE_INIT;
    while (gst_iterator_next(encoders, &item) == GST_ITERATOR_OK) {
        g_object_set(g_value_get_object(&item), setting.property, static_cast<gint>(setting.value),
                     nullptr);
        g_value_reset(&item);
    }
    g_value_unset(&item);
    gst_iterator_free(encoders);
}

// 클라이언트의 요청으로 파이프라인이 생성된 시점에 호출된다
void on_media_configure(GstRTSPMediaFactory* /*factory*/, GstRTSPMedia* media, gpointer data) {
    const MediaContext& context = *static_cast<MediaContext*>(data);
    GstElement* pipeline = gst_rtsp_media_get_element(media);
    if (context.encoder_setting.has_value()) {
        apply_encoder_setting(pipeline, context.encoder_setting.value());
    }
    context.sources->set(media, gst_bin_get_by_name_recurse_up(GST_BIN(pipeline), kVideoSourceName),
                         gst_bin_get_by_name_recurse_up(GST_BIN(pipeline), kMetadataSourceName));
    gst_object_unref(pipeline);
    connect_with_context(media, "unprepared", G_CALLBACK(on_media_unprepared), context);
}

GstRTSPFilterResult remove_client(GstRTSPServer* /*server*/, GstRTSPClient* /*client*/,
                                  gpointer /*data*/) {
    return GST_RTSP_FILTER_REMOVE;
}

}  // namespace

// 서버와 GLib main loop 의 수명을 관리하고, push() 로 받은 문서를 RTP 패킷으로 나누어 MediaSources
// 에 넘긴다
class RtspServer::Impl {
public:
    Impl(const core::IClock& clock, RtspServerConfig config)
        : clock_(clock),
          config_(std::move(config)),
          packetizer_(config_.metadata_rtp),
          sources_(std::make_shared<MediaSources>()) {}

    bool start() {
        if (server_ != nullptr) {
            return true;
        }
        gst_init(nullptr, nullptr);
        if (!is_consistent(config_)) {
            return false;
        }
        const std::string description = launch_description(config_);
        if (!is_valid_launch(description, encoder_setting_of(config_.video))) {
            return false;
        }
        create_server(description);
        attach_id_ = gst_rtsp_server_attach(server_, context_);
        if (attach_id_ == 0) {
            log_error("포트 " + std::to_string(config_.port) + " 을 열지 못함");
            release_server();
            return false;
        }
        if (!run_loop_thread()) {
            log_error("GLib main loop 스레드를 시작하지 못함");
            release_server();
            return false;
        }
        return true;
    }

    void stop() {
        if (server_ == nullptr) {
            return;
        }
        // 반환값은 filter 가 REF 를 반환한 클라이언트의 목록이다. remove_client 는 REF 를 반환하지
        // 않으므로 빈 목록이다
        g_list_free_full(gst_rtsp_server_client_filter(server_, remove_client, nullptr),
                         g_object_unref);
        g_main_loop_quit(loop_);
        if (loop_thread_.joinable()) {
            loop_thread_.join();
        }
        sources_->release();
        release_server();
    }

    std::uint16_t port() const {
        if (server_ == nullptr) {
            return 0;
        }
        return static_cast<std::uint16_t>(std::max(gst_rtsp_server_get_bound_port(server_), 0));
    }

    RtspServerStats stats() const { return stats_; }

    void push(const perception_api::Frame& frame, std::string_view metadata_document) {
        if (!sources_->has_pipeline()) {
            ++stats_.dropped_no_client;  // 패킷으로 나누기 전에 확인해 불필요한 복사를 피한다
            return;
        }
        const std::vector<MetadataRtpPacketizer::Packet> packets =
            packetizer_.packetize(metadata_document, rtp_timestamp_from_unix_ns(frame.unix_ns));
        count(sources_->push(frame, packets, clock_.now_monotonic_ns()));
    }

private:
    void count(PushResult result) {
        switch (result) {
            case PushResult::Sent:
                ++stats_.sent;
                break;
            case PushResult::NoClient:
                ++stats_.dropped_no_client;
                break;
            case PushResult::NotPlaying:
                ++stats_.dropped_not_playing;
                break;
            case PushResult::InvalidFrame:
                ++stats_.dropped_invalid_frame;
                break;
            case PushResult::SendFailed:
                ++stats_.dropped_send_failed;
                break;
        }
    }

    void create_server(const std::string& description) {
        context_ = g_main_context_new();
        loop_ = g_main_loop_new(context_, FALSE);
        server_ = gst_rtsp_server_new();
        gst_rtsp_server_set_service(server_, std::to_string(config_.port).c_str());

        GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();
        gst_rtsp_media_factory_set_launch(factory, description.c_str());
        gst_rtsp_media_factory_set_shared(factory, TRUE);
        connect_with_context(factory, "media-configure", G_CALLBACK(on_media_configure),
                             MediaContext{sources_, encoder_setting_of(config_.video)});
        GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server_);
        gst_rtsp_mount_points_add_factory(mounts, config_.path.c_str(), factory);
        g_object_unref(mounts);
    }

    // loop 스레드를 시작하고 loop 가 실행을 시작한 뒤 반환한다. 그 전에 stop() 이
    // g_main_loop_quit() 을 호출하면 loop 가 멈추지 않기 때문이다.
    // 스레드를 시작하지 못하면 false 를 반환한다
    bool run_loop_thread() {
        std::promise<void> running;
        GSource* notify_running = g_idle_source_new();
        g_source_set_callback(
            notify_running,
            [](gpointer promise) -> gboolean {
                static_cast<std::promise<void>*>(promise)->set_value();
                return G_SOURCE_REMOVE;
            },
            &running, nullptr);
        g_source_attach(notify_running, context_);
        try {
            loop_thread_ = std::thread([loop = loop_] { g_main_loop_run(loop); });
        } catch (const std::exception&) {
            g_source_destroy(notify_running);  // running 의 주소를 가진 source 가 남지 않게 한다
            g_source_unref(notify_running);
            return false;
        }
        g_source_unref(notify_running);
        running.get_future().wait();
        return true;
    }

    void release_server() {
        if (attach_id_ != 0) {
            GSource* accept_source = g_main_context_find_source_by_id(context_, attach_id_);
            if (accept_source != nullptr) {
                g_source_destroy(accept_source);
            }
            attach_id_ = 0;
        }
        g_clear_object(&server_);
        g_clear_pointer(&loop_, g_main_loop_unref);
        g_clear_pointer(&context_, g_main_context_unref);
    }

    const core::IClock& clock_;
    RtspServerConfig config_;
    MetadataRtpPacketizer packetizer_;
    std::shared_ptr<MediaSources> sources_;
    RtspServerStats stats_;
    GMainContext* context_ = nullptr;
    GMainLoop* loop_ = nullptr;
    GstRTSPServer* server_ = nullptr;
    guint attach_id_ = 0;
    std::thread loop_thread_;
};

RtspServer::RtspServer(const core::IClock& clock, RtspServerConfig config)
    : impl_(std::make_unique<Impl>(clock, std::move(config))) {}

RtspServer::~RtspServer() { impl_->stop(); }

bool RtspServer::start() { return impl_->start(); }

void RtspServer::stop() { impl_->stop(); }

std::uint16_t RtspServer::port() const { return impl_->port(); }

RtspServerStats RtspServer::stats() const { return impl_->stats(); }

void RtspServer::push(const perception_api::Frame& frame, std::string_view metadata_document) {
    impl_->push(frame, metadata_document);
}

}  // namespace argus::video_sender
