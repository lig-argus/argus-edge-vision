// 영상 프레임 ↔ 메타데이터 UTC 짝짓기 (docs/spec/video.md 4장, 6장)
#include "argus/gcs/rtsp/frame_synchronizer.hpp"

#include <QtTest>

namespace argus::gcs::rtsp {

namespace {

constexpr std::int64_t kFrameNs = 33'333'333;  // 30 FPS
constexpr std::int64_t kBase = std::int64_t{1792048527321} * 1'000'000;

FrameMetadata metadataAt(std::int64_t utcNs, std::uint64_t frameId) {
    FrameMetadata metadata;
    metadata.utcNs = utcNs;
    metadata.frameId = frameId;
    return metadata;
}

std::optional<std::uint64_t> frameIdOf(const std::optional<FrameMetadata>& metadata) {
    return metadata.value_or(FrameMetadata{}).frameId;
}

}  // namespace

class FrameSynchronizerTest : public QObject {
    Q_OBJECT

private slots:
    // 메타데이터 UtcTime 은 ms 단위라 영상 UTC(ns)와 1 ms 안쪽으로 다르다
    void matchesWithinHalfFrame() {
        FrameSynchronizer sync;
        sync.addMetadata(metadataAt(kBase, 100));
        sync.addMetadata(metadataAt(kBase + kFrameNs, 101));

        QCOMPARE(frameIdOf(sync.takeFor(kBase + 400'000)), std::optional<std::uint64_t>(100));
        QCOMPARE(frameIdOf(sync.takeFor(kBase + kFrameNs - 300'000)),
                 std::optional<std::uint64_t>(101));
        QCOMPARE(sync.matched(), std::uint64_t{2});
    }

    // 메타데이터가 아직 오지 않은 프레임은 짝이 없다
    void frameWithoutMetadataHasNoMatch() {
        FrameSynchronizer sync;

        QVERIFY(!sync.takeFor(kBase).has_value());
        QCOMPARE(sync.unmatchedFrames(), std::uint64_t{1});
    }

    // 허용 오차(±반 프레임) 밖이면 짝이 아니다
    void outsideToleranceIsNoMatch() {
        FrameSynchronizer sync;
        sync.addMetadata(metadataAt(kBase + 20'000'000, 100));  // 20 ms 뒤

        QVERIFY(!sync.takeFor(kBase).has_value());
        QVERIFY(sync.takeFor(kBase + 20'000'000).has_value());  // 그 시각의 프레임이 오면 짝
    }

    // 이미 지나간 프레임의 메타데이터가 늦게 오면 버린다
    void dropsLateMetadata() {
        FrameSynchronizer sync;
        QVERIFY(!sync.takeFor(kBase + 2 * kFrameNs).has_value());
        sync.addMetadata(metadataAt(kBase, 100));  // 두 프레임 전 것

        QCOMPARE(sync.droppedMetadata(), std::uint64_t{1});
        QVERIFY(!sync.takeFor(kBase + 3 * kFrameNs).has_value());
    }

    // 프레임이 빠지면 그 프레임의 메타데이터는 다음 프레임 때 버린다
    void dropsMetadataOfSkippedFrame() {
        FrameSynchronizer sync;
        sync.addMetadata(metadataAt(kBase, 100));
        sync.addMetadata(metadataAt(kBase + kFrameNs, 101));

        // 100 프레임은 오지 않음
        QCOMPARE(frameIdOf(sync.takeFor(kBase + kFrameNs)), std::optional<std::uint64_t>(101));
        QCOMPARE(sync.droppedMetadata(), std::uint64_t{1});
    }

    // UtcTime 역순으로 도착해도 시각으로 짝짓는다
    void outOfOrderArrival() {
        FrameSynchronizer sync;
        sync.addMetadata(metadataAt(kBase + kFrameNs, 101));
        sync.addMetadata(metadataAt(kBase, 100));

        QCOMPARE(frameIdOf(sync.takeFor(kBase)), std::optional<std::uint64_t>(100));
        QCOMPARE(frameIdOf(sync.takeFor(kBase + kFrameNs)), std::optional<std::uint64_t>(101));
    }

    // 짝을 못 찾고 쌓이면 오래된 것부터 버린다
    void boundedHolding() {
        FrameSynchronizer sync;
        for (std::size_t i = 0; i < FrameSynchronizer::kMaxHeld + 5; ++i) {
            sync.addMetadata(metadataAt(kBase + static_cast<std::int64_t>(i) * kFrameNs, i));
        }

        QCOMPARE(sync.droppedMetadata(), std::uint64_t{5});
        QVERIFY(!sync.takeFor(kBase).has_value());  // 0 번은 이미 버렸다
    }
};

}  // namespace argus::gcs::rtsp

QTEST_GUILESS_MAIN(argus::gcs::rtsp::FrameSynchronizerTest)
#include "frame_synchronizer_test.moc"
