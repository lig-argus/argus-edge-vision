#include "argus/gcs/video_rtsp/rtsp_video_source.hpp"

#include <QImage>
#include <algorithm>
#include <deque>
#include <iterator>
#include <utility>

#include "argus/gcs/rtsp/frame_synchronizer.hpp"
#include "argus/gcs/rtsp/rtp_sequence.hpp"

namespace argus::gcs::video_rtsp {

namespace {

constexpr double kBytesPerKb = 1024.0;
constexpr double kNsPerSecond = 1e9;

// DecodedFrame 의 바이트를 복사하지 않고 QImage 로 감싼다. QImage 가 사라질 때 바이트 배열 참조를
// 놓는다
std::optional<QImage> toImage(const rtsp::DecodedFrame& frame) {
    const qsizetype needed = static_cast<qsizetype>(frame.bytesPerLine) * frame.height;
    if (frame.width <= 0 || frame.height <= 0 || frame.bytesPerLine < frame.width * 4 ||
        frame.rgbx.size() < needed) {
        return std::nullopt;
    }
    auto* bytes = new QByteArray(frame.rgbx);  // 데이터는 공유한다 (implicit sharing)
    return QImage(
        reinterpret_cast<const uchar*>(bytes->constData()), frame.width, frame.height,
        frame.bytesPerLine, QImage::Format_RGBX8888,
        [](void* info) { delete static_cast<QByteArray*>(info); }, bytes);
}

video::VideoTrackStats trackStats(video::VideoTrack track, const rtsp::RtpStats& rtp) {
    return video::VideoTrackStats{track, rtp.received, rtp.lost, rtp.reordered, rtp.duplicated};
}

void appendLosses(std::vector<video::VideoLossRecord>& out, video::VideoTrack track,
                  const rtsp::RtpStats& rtp) {
    for (const rtsp::RtpLossRecord& record : rtp.losses) {
        out.push_back(video::VideoLossRecord{record.detectedNs, track, record.lastSeq,
                                             record.nextSeq, record.missing});
    }
}

}  // namespace

struct RtspVideoSource::Channel {
    Stream stream;
    QString session;
    std::unique_ptr<rtsp::IRtspReceiver> receiver;
    rtsp::FrameSynchronizer sync;
    std::optional<video::VideoView> view;
    std::deque<video::VideoFrame> held;   // 오래된 것이 앞
    std::deque<std::int64_t> frameTimes;  // 최근 kRateWindowNs 의 프레임 시각

    // (시각, 받은 바이트 누적), 최근 kRateWindowNs
    std::deque<std::pair<std::int64_t, std::uint64_t>> byteSamples;

    std::optional<std::int64_t> lastFrameNs;
    std::optional<std::int64_t> failedNs;  // Failed 가 된 시각 (다시 열 때 지움)
};

RtspVideoSource::RtspVideoSource(rtsp::RtspReceiverFactory factory, const core::IClock& clock,
                                 QObject* parent)
    : video::IVideoSource(parent), factory_(std::move(factory)), clock_(clock) {}

RtspVideoSource::~RtspVideoSource() { closeAll(); }

void RtspVideoSource::setStreams(std::vector<Stream> streams) {
    if (streams == streams_) {
        return;
    }
    streams_ = std::move(streams);
    closeAll();
    openAll();
}

void RtspVideoSource::openAll() {
    ++connection_;
    for (const Stream& stream : streams_) {
        auto channel = std::make_unique<Channel>();
        channel->stream = stream;
        channel->session = QStringLiteral("%1#%2").arg(stream.url).arg(connection_);
        channel->receiver = factory_(stream.url);
        if (!channel->receiver) {
            continue;
        }
        Channel* raw = channel.get();
        connect(raw->receiver.get(), &rtsp::IRtspReceiver::metadataReceived, this,
                [raw](const rtsp::FrameMetadata& metadata) { raw->sync.addMetadata(metadata); });
        connect(raw->receiver.get(), &rtsp::IRtspReceiver::frameReceived, this,
                [this, raw](const rtsp::DecodedFrame& frame) { onFrame(*raw, frame); });
        connect(raw->receiver.get(), &rtsp::IRtspReceiver::stateChanged, this,
                [this, raw](rtsp::ReceiverState state) {
                    raw->failedNs = state == rtsp::ReceiverState::Failed
                                        ? std::optional<std::int64_t>(clock_.nowMonotonicNs())
                                        : std::nullopt;
                });
        channels_.push_back(std::move(channel));
        raw->receiver->start();
    }
}

void RtspVideoSource::closeAll() {
    for (auto& channel : channels_) {
        channel->receiver->disconnect(this);
        channel->receiver->stop();
    }
    channels_.clear();
}

void RtspVideoSource::tick() {
    const std::int64_t nowNs = clock_.nowMonotonicNs();
    for (auto& channel : channels_) {
        if (channel->receiver->state() == rtsp::ReceiverState::Failed && channel->failedNs &&
            nowNs - *channel->failedNs >= kRetryNs) {
            channel->failedNs.reset();
            channel->receiver->start();
        }
    }
}

void RtspVideoSource::onFrame(Channel& channel, const rtsp::DecodedFrame& frame) {
    std::optional<QImage> image = toImage(frame);
    if (!image) {
        return;
    }
    const std::int64_t nowNs = clock_.nowMonotonicNs();
    const std::uint8_t sensor = channel.stream.sensor;
    std::uint32_t frameId = 0;
    if (frame.utcNs) {
        if (std::optional<rtsp::FrameMetadata> metadata = channel.sync.takeFor(*frame.utcNs)) {
            frameId = static_cast<std::uint32_t>(metadata->frameId.value_or(0));
            // 상자가 영상보다 먼저 받는 쪽에 들어가야 영상 칸이 바로 짝짓는다
            emit metadataReceived(sensor, *metadata, channel.session);
        }
    }
    std::deque<std::int64_t>& times = channel.frameTimes;
    times.push_back(nowNs);
    while (!times.empty() && nowNs - times.front() > kRateWindowNs) {
        times.pop_front();
    }
    auto& bytes = channel.byteSamples;
    bytes.emplace_back(nowNs, channel.receiver->counters().bytes);
    while (bytes.size() > 1 && nowNs - bytes.front().first > kRateWindowNs) {
        bytes.pop_front();
    }
    const std::int64_t spanNs = std::max<std::int64_t>(bytes.back().first - bytes.front().first, 1);

    video::VideoView view;
    view.image = *image;
    view.frameId = frameId;
    view.receivedNs = nowNs;
    view.fps = static_cast<double>(times.size());
    view.kbPerSec = static_cast<double>(bytes.back().second - bytes.front().second) / kBytesPerKb *
                    kNsPerSecond / static_cast<double>(spanNs);
    channel.view = view;
    channel.lastFrameNs = nowNs;
    channel.held.push_back(video::VideoFrame{std::move(*image), frameId, nowNs});
    while (channel.held.size() > 1 && nowNs - channel.held.front().receivedNs > kHoldNs) {
        channel.held.pop_front();
    }
    ++displayed_;
    lastDisplayNs_ = nowNs;
    emit frameReady(sensor);
}

const RtspVideoSource::Channel* RtspVideoSource::channelFor(std::uint8_t sensor) const {
    const auto found =
        std::find_if(channels_.begin(), channels_.end(),
                     [sensor](const auto& channel) { return channel->stream.sensor == sensor; });
    return found == channels_.end() ? nullptr : found->get();
}

std::optional<video::VideoView> RtspVideoSource::view(std::uint8_t sensor) const {
    const Channel* channel = channelFor(sensor);
    return channel != nullptr ? channel->view : std::nullopt;
}

std::vector<video::VideoFrame> RtspVideoSource::heldFrames(std::uint8_t sensor) const {
    const Channel* channel = channelFor(sensor);
    if (channel == nullptr) {
        return {};
    }
    return {channel->held.rbegin(), channel->held.rend()};  // 최신 순
}

bool RtspVideoSource::receiving() const {
    const std::int64_t nowNs = clock_.nowMonotonicNs();
    return std::any_of(channels_.begin(), channels_.end(), [nowNs](const auto& channel) {
        return channel->lastFrameNs && nowNs - *channel->lastFrameNs <= kReceivingNs;
    });
}

bool RtspVideoSource::playing() const {
    return std::any_of(channels_.begin(), channels_.end(), [](const auto& channel) {
        return channel->receiver->state() == rtsp::ReceiverState::Playing;
    });
}

video::VideoCounters RtspVideoSource::counters() const {
    video::VideoCounters out;
    for (const auto& channel : channels_) {
        const rtsp::ReceiverCounters counters = channel->receiver->counters();
        out.rtpPackets += counters.videoRtp.received;
        out.rtpLost += counters.videoRtp.lost;
        out.rtpBytes += counters.bytes;
        out.metadataPackets += counters.metadataRtp.received;
        out.metadataLost += counters.metadataRtp.lost;
        out.frames += counters.frames;
    }
    out.displayed = displayed_;
    out.lastDisplayNs = lastDisplayNs_;
    out.receiving = receiving();
    return out;
}

// 두 트랙의 RTP 순번으로 센 수를 더하고, 손실 기록을 시각 순으로 합친다 (최근 kMaxLossRecords 개)
std::optional<video::VideoLinkStats> RtspVideoSource::linkStats(std::uint8_t sensor) const {
    const Channel* channel = channelFor(sensor);
    if (channel == nullptr) {
        return std::nullopt;
    }
    const rtsp::ReceiverCounters counters = channel->receiver->counters();
    if (counters.videoRtp.received == 0 && counters.metadataRtp.received == 0 && !channel->view) {
        return std::nullopt;
    }
    video::VideoLinkStats out;
    if (channel->view) {
        out.fps = channel->view->fps;
        out.kbPerSec = channel->view->kbPerSec;
    }
    out.tracks = {trackStats(video::VideoTrack::Video, counters.videoRtp),
                  trackStats(video::VideoTrack::Metadata, counters.metadataRtp)};
    for (const video::VideoTrackStats& track : out.tracks) {
        out.received += track.received;
        out.lost += track.lost;
        out.reordered += track.reordered;
        out.duplicated += track.duplicated;
    }
    std::vector<video::VideoLossRecord> losses;
    appendLosses(losses, video::VideoTrack::Video, counters.videoRtp);
    appendLosses(losses, video::VideoTrack::Metadata, counters.metadataRtp);
    std::stable_sort(losses.begin(), losses.end(),
                     [](const auto& a, const auto& b) { return a.detectedNs < b.detectedNs; });
    const std::size_t skip = losses.size() > video::VideoLinkStats::kMaxLossRecords
                                 ? losses.size() - video::VideoLinkStats::kMaxLossRecords
                                 : 0;
    out.losses.assign(std::next(losses.begin(), static_cast<std::ptrdiff_t>(skip)), losses.end());
    return out;
}

}  // namespace argus::gcs::video_rtsp
