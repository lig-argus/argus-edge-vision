// GstRtspReceiver: 필요한 GStreamer 요소와 연결 실패 처리.
// RTSP 서버 없이 확인할 수 있는 것만 다룬다. 트랙 처리는 track_branch_test 가 확인한다
#include "argus/gcs/rtsp_gstreamer/gst_rtsp_receiver.hpp"

#include <QSignalSpy>
#include <QtTest>

#include "argus/gcs/core_testing/fake_clock.hpp"
#include "track_branch.hpp"

namespace argus::gcs::rtsp_gstreamer {

namespace {

constexpr std::int32_t kFailTimeoutMs = 10'000;

}  // namespace

class GstRtspReceiverTest : public QObject {
    Q_OBJECT

private slots:
    void initTestCase() { ensureGstreamerInitialized(); }

    // 통제기 실행에 필요한 요소 (docs/spec/video.md 6장): plugins-good, gst-libav, plugins-base
    void requiredElementsExist() {
        for (const char* name : {"rtspsrc", "rtph264depay", "avdec_h264", "rtpjpegdepay", "jpegdec",
                                 "videoconvert", "appsink", "fakesink"}) {
            GstElementFactory* factory = gst_element_factory_find(name);
            QVERIFY2(factory != nullptr, name);
            gst_object_unref(factory);
        }
    }

    void startsStopped() {
        const core_testing::FakeClock clock;
        const GstRtspReceiver receiver(QStringLiteral("rtsp://127.0.0.1:9/argus/ir"), clock);

        QVERIFY(receiver.state() == rtsp::ReceiverState::Stopped);
        QCOMPARE(receiver.url(), QStringLiteral("rtsp://127.0.0.1:9/argus/ir"));
    }

    // 서버가 없으면 Connecting 을 거쳐 Failed, 이유가 남는다. stop() 하면 Stopped
    void failsWhenServerIsUnreachable() {
        const core_testing::FakeClock clock;
        GstRtspReceiver receiver(QStringLiteral("rtsp://127.0.0.1:9/argus/ir"), clock);
        const QSignalSpy states(&receiver, &rtsp::IRtspReceiver::stateChanged);

        receiver.start();

        QVERIFY(receiver.state() == rtsp::ReceiverState::Connecting ||
                receiver.state() == rtsp::ReceiverState::Failed);
        QTRY_VERIFY_WITH_TIMEOUT(receiver.state() == rtsp::ReceiverState::Failed, kFailTimeoutMs);
        QVERIFY(!receiver.lastError().isEmpty());
        QCOMPARE(receiver.counters().frames, std::uint64_t{0});

        receiver.stop();
        QVERIFY(receiver.state() == rtsp::ReceiverState::Stopped);
        QVERIFY(receiver.lastError().isEmpty());
        QVERIFY(states.count() >= 3);
    }
};

}  // namespace argus::gcs::rtsp_gstreamer

QTEST_GUILESS_MAIN(argus::gcs::rtsp_gstreamer::GstRtspReceiverTest)
#include "gst_rtsp_receiver_test.moc"
