// RtspVideoSource: 테스트용 수신기와 FakeClock 으로 짝짓기, 알림 순서, 수신 통계, 재연결 확인
#include "argus/gcs/video_rtsp/rtsp_video_source.hpp"

#include <QSignalSpy>
#include <QStringList>
#include <QtTest>

#include "argus/gcs/core_testing/fake_clock.hpp"
#include "fake_rtsp_receiver.hpp"

namespace argus::gcs::video_rtsp {

namespace {

constexpr std::uint8_t kIr = 1;
constexpr std::uint8_t kEo = 2;
constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kUtcBase = std::int64_t{1792048527321} * kMs;
constexpr QLatin1StringView kIrUrl{"rtsp://192.168.4.1:8554/argus/ir"};
constexpr QLatin1StringView kEoUrl{"rtsp://192.168.4.1:8554/argus/eo"};

rtsp::DecodedFrame frameAt(std::optional<std::int64_t> utcNs) {
    rtsp::DecodedFrame frame;
    frame.width = 4;
    frame.height = 2;
    frame.bytesPerLine = 16;
    frame.rgbx = QByteArray(32, '\x7F');
    frame.utcNs = utcNs;
    return frame;
}

rtsp::FrameMetadata metadataAt(std::int64_t utcNs, std::uint64_t frameId) {
    rtsp::FrameMetadata metadata;
    metadata.utcNs = utcNs;
    metadata.frameId = frameId;
    return metadata;
}

}  // namespace

class RtspVideoSourceTest : public QObject {
    Q_OBJECT

private:
    std::unique_ptr<core_testing::FakeClock> clock_;
    std::vector<FakeRtspReceiver*> receivers_;  // 만든 순서. RtspVideoSource 가 소유한다

    rtsp::RtspReceiverFactory factory() {
        return [this](const QString& url) {
            auto receiver = std::make_unique<FakeRtspReceiver>(url);
            receivers_.push_back(receiver.get());
            return receiver;
        };
    }

private slots:
    void init() {
        clock_ = std::make_unique<core_testing::FakeClock>(1'000 * kMs);
        receivers_.clear();
    }

    void opensOneReceiverPerStream() {
        RtspVideoSource source(factory(), *clock_);
        source.setStreams({{kIr, QString(kIrUrl)}, {kEo, QString(kEoUrl)}});

        QCOMPARE(receivers_.size(), std::size_t{2});
        QCOMPARE(receivers_.at(0)->url(), QString(kIrUrl));
        QCOMPARE(receivers_.at(1)->url(), QString(kEoUrl));
        QCOMPARE(receivers_.at(0)->starts(), std::int32_t{1});
    }

    // 짝지은 메타데이터를 그 프레임의 frameReady 보다 먼저 내고, FrameId 를 frame_id 로 쓴다
    void emitsPairedMetadataBeforeFrame() {
        RtspVideoSource source(factory(), *clock_);
        source.setStreams({{kIr, QString(kIrUrl)}});
        QStringList order;
        QString session;
        connect(&source, &RtspVideoSource::metadataReceived, this,
                [&order, &session](std::uint8_t sensor, const rtsp::FrameMetadata& metadata,
                                   const QString& from) {
                    order << QStringLiteral("metadata %1 %2").arg(sensor).arg(*metadata.frameId);
                    session = from;
                });
        connect(&source, &RtspVideoSource::frameReady, this,
                [&order](std::uint8_t sensor) { order << QStringLiteral("frame %1").arg(sensor); });

        receivers_.front()->deliverMetadata(metadataAt(kUtcBase, 100));
        receivers_.front()->deliverFrame(frameAt(kUtcBase + 400'000));

        QCOMPARE(order, QStringList({QStringLiteral("metadata 1 100"), QStringLiteral("frame 1")}));
        QVERIFY(!session.isEmpty());
        QCOMPARE(source.view(kIr).value_or(video::VideoView{}).frameId, std::uint32_t{100});
        QCOMPARE(source.view(kIr).value_or(video::VideoView{}).image.size(), QSize(4, 2));
        QVERIFY(!source.view(kEo).has_value());
    }

    // UTC 가 없는 프레임(첫 RTCP SR 전)은 짝 없이 표시한다
    void frameWithoutUtcHasNoPair() {
        RtspVideoSource source(factory(), *clock_);
        source.setStreams({{kIr, QString(kIrUrl)}});
        const QSignalSpy metadata(&source, &RtspVideoSource::metadataReceived);
        receivers_.front()->deliverMetadata(metadataAt(kUtcBase, 100));

        receivers_.front()->deliverFrame(frameAt(std::nullopt));

        QCOMPARE(metadata.count(), 0);
        QCOMPARE(source.view(kIr).value_or(video::VideoView{}).frameId, std::uint32_t{0});
    }

    // 크기가 맞지 않는 프레임은 버린다
    void ignoresMalformedFrame() {
        RtspVideoSource source(factory(), *clock_);
        source.setStreams({{kIr, QString(kIrUrl)}});
        rtsp::DecodedFrame frame = frameAt(std::nullopt);
        frame.rgbx.truncate(10);

        receivers_.front()->deliverFrame(frame);

        QVERIFY(!source.view(kIr).has_value());
    }

    void computesFrameRateAndThroughput() {
        RtspVideoSource source(factory(), *clock_);
        source.setStreams({{kIr, QString(kIrUrl)}});
        FakeRtspReceiver& receiver = *receivers_.front();

        for (std::int32_t i = 0; i < 3; ++i) {
            receiver.mutableCounters().bytes += std::uint64_t{10} * 1024;
            receiver.deliverFrame(frameAt(std::nullopt));
            clock_->advanceNs(500 * kMs);
        }

        const video::VideoView view = source.view(kIr).value_or(video::VideoView{});
        QCOMPARE(view.fps, 3.0);        // 0, 500, 1000 ms (1 초 구간 안)
        QCOMPARE(view.kbPerSec, 20.0);  // 1 초에 20 kB
        QCOMPARE(view.receivedNs, std::int64_t{2'000 * kMs});
        QCOMPARE(source.counters().displayed, std::uint64_t{3});
        QCOMPARE(source.counters().lastDisplayNs, std::optional<std::int64_t>(2'000 * kMs));
    }

    // 최근 kHoldNs 안의 프레임만 보관하고 최신 순으로 돌려준다
    void holdsRecentFramesNewestFirst() {
        RtspVideoSource source(factory(), *clock_);
        source.setStreams({{kIr, QString(kIrUrl)}});
        for (const std::int64_t stepMs : {0, 100, 100, 200}) {  // 0, 100, 200, 400 ms
            clock_->advanceNs(stepMs * kMs);
            receivers_.front()->deliverFrame(frameAt(std::nullopt));
        }

        const std::vector<video::VideoFrame> held = source.heldFrames(kIr);

        QCOMPARE(held.size(), std::size_t{3});  // 0 ms 는 400 ms 보다 300 ms 넘게 앞섬
        QCOMPARE(held.front().receivedNs, std::int64_t{1'400 * kMs});
        QCOMPARE(held.back().receivedNs, std::int64_t{1'100 * kMs});
    }

    void receivingAndPlaying() {
        RtspVideoSource source(factory(), *clock_);
        source.setStreams({{kIr, QString(kIrUrl)}});
        QVERIFY(!source.receiving());
        QVERIFY(!source.playing());

        receivers_.front()->setState(rtsp::ReceiverState::Playing);
        receivers_.front()->deliverFrame(frameAt(std::nullopt));
        QVERIFY(source.receiving());
        QVERIFY(source.playing());

        clock_->advanceNs(RtspVideoSource::kReceivingNs + 1);
        QVERIFY(!source.receiving());
        QVERIFY(!source.counters().receiving);
    }

    // 실패한 수신기는 kRetryNs 가 지난 뒤의 tick 에서 다시 연다
    void retriesFailedReceiverAfterDelay() {
        RtspVideoSource source(factory(), *clock_);
        source.setStreams({{kIr, QString(kIrUrl)}});
        FakeRtspReceiver& receiver = *receivers_.front();
        receiver.setState(rtsp::ReceiverState::Failed);

        clock_->advanceNs(RtspVideoSource::kRetryNs - 1);
        source.tick();
        QCOMPARE(receiver.starts(), std::int32_t{1});

        clock_->advanceNs(1);
        source.tick();
        QCOMPARE(receiver.starts(), std::int32_t{2});
        source.tick();  // 다시 실패하기 전에는 열지 않는다
        QCOMPARE(receiver.starts(), std::int32_t{2});
    }

    // 같은 스트림이면 그대로 두고, 바뀌면 다시 열며 session 글이 바뀐다
    void reopensOnlyWhenStreamsChange() {
        RtspVideoSource source(factory(), *clock_);
        QStringList sessions;
        connect(&source, &RtspVideoSource::metadataReceived, this,
                [&sessions](std::uint8_t, const rtsp::FrameMetadata&, const QString& session) {
                    sessions << session;
                });
        source.setStreams({{kIr, QString(kIrUrl)}});
        source.setStreams({{kIr, QString(kIrUrl)}});
        QCOMPARE(receivers_.size(), std::size_t{1});
        receivers_.back()->deliverMetadata(metadataAt(kUtcBase, 1));
        receivers_.back()->deliverFrame(frameAt(kUtcBase));

        source.setStreams({{kIr, QString(kEoUrl)}});
        QCOMPARE(receivers_.size(), std::size_t{2});
        receivers_.back()->deliverMetadata(metadataAt(kUtcBase, 1));
        receivers_.back()->deliverFrame(frameAt(kUtcBase));

        QCOMPARE(sessions.size(), 2);
        QVERIFY(sessions.at(0) != sessions.at(1));

        source.setStreams({});
        QVERIFY(!source.view(kIr).has_value());
    }

    // 두 트랙의 수를 더하고 손실 기록을 시각 순으로 합친다
    void mergesTrackStatistics() {
        RtspVideoSource source(factory(), *clock_);
        source.setStreams({{kIr, QString(kIrUrl)}});
        rtsp::ReceiverCounters& counters = receivers_.front()->mutableCounters();
        counters.videoRtp.received = 100;
        counters.videoRtp.lost = 3;
        counters.videoRtp.losses = {rtsp::RtpLossRecord{30, 10, 14, 3}};
        counters.metadataRtp.received = 40;
        counters.metadataRtp.lost = 1;
        counters.metadataRtp.losses = {rtsp::RtpLossRecord{20, 5, 7, 1}};

        const video::VideoLinkStats stats = source.linkStats(kIr).value_or(video::VideoLinkStats{});

        QCOMPARE(stats.received, std::uint64_t{140});
        QCOMPARE(stats.lost, std::uint64_t{4});
        QCOMPARE(stats.tracks.size(), std::size_t{2});
        QCOMPARE(stats.losses.size(), std::size_t{2});
        QVERIFY(stats.losses.front().track == video::VideoTrack::Metadata);
        QCOMPARE(stats.losses.front().detectedNs, std::int64_t{20});
        QCOMPARE(source.counters().rtpPackets, std::uint64_t{100});
        QCOMPARE(source.counters().metadataPackets, std::uint64_t{40});
        QVERIFY(!source.linkStats(kEo).has_value());
    }
};

}  // namespace argus::gcs::video_rtsp

QTEST_GUILESS_MAIN(argus::gcs::video_rtsp::RtspVideoSourceTest)
#include "rtsp_video_source_test.moc"
