#include "argus/gcs/link/detection_store.hpp"

#include <QtTest>
#include <cstdint>

#include "argus/mavlink.h"

namespace argus::gcs::link {
namespace {

constexpr std::uint8_t kIr = ARGUS_SENSOR_IR;
constexpr std::uint8_t kEo = ARGUS_SENSOR_EO;
constexpr std::int64_t kFramePeriodNs = 50 * kNsPerMs;  // 20 fps

DetectionFrame frame(std::uint32_t frameId, std::uint32_t generation = 1,
                     const char* source = "ir0") {
    DetectionFrame f;
    f.frameId = frameId;
    f.generation = generation;
    f.imageWidth = 640;
    f.imageHeight = 480;
    f.source = QString::fromLatin1(source);
    return f;
}

}  // namespace

class DetectionStoreTest : public QObject {
    Q_OBJECT

private slots:
    // 받은 시각을 채우고 센서별로 기록한다
    void recordsPerSensor() {
        DetectionStore store;
        const auto added = store.add(kIr, frame(10), 5);
        QVERIFY(added.has_value());
        QCOMPARE(added.value_or(DetectionFrame{}).receivedNs, std::int64_t{5});
        QVERIFY(store.add(kEo, frame(3, 1, "eo0"), 6));
        QCOMPARE(store.latest(kIr).value_or(DetectionFrame{}).frameId, std::uint32_t{10});
        QCOMPARE(store.latest(kEo).value_or(DetectionFrame{}).frameId, std::uint32_t{3});
        QCOMPARE(store.history(kIr).size(), std::size_t{1});
        QVERIFY(!store.latest(ARGUS_SENSOR_UNKNOWN));
        QVERIFY(!store.add(ARGUS_SENSOR_UNKNOWN, frame(1), 0));
    }

    // 최근 kHistory 개만 남기고 frameId 로 찾는다
    void keepsRecentHistory() {
        DetectionStore store;
        for (std::uint32_t id = 1; id <= DetectionStore::kHistory + 5; ++id) {
            QVERIFY(store.add(kIr, frame(id), id * kFramePeriodNs));
        }
        QCOMPARE(store.history(kIr).size(), DetectionStore::kHistory);
        QCOMPARE(store.history(kIr).front().frameId, std::uint32_t{6});
        QVERIFY(!store.forFrame(kIr, 5));
        QCOMPARE(store.forFrame(kIr, 20).value_or(DetectionFrame{}).frameId, std::uint32_t{20});
        QVERIFY(!store.forFrame(kEo, 20));
    }

    // fps 는 마지막 1 s 안에 받은 결과 수
    void countsFramesInLastSecond() {
        DetectionStore store;
        std::optional<DetectionFrame> last;
        for (std::uint32_t id = 1; id <= 40; ++id) {
            last = store.add(kIr, frame(id), id * kFramePeriodNs);
        }
        QCOMPARE(last.value_or(DetectionFrame{}).fps, 21.0);  // 1 s 구간의 양 끝을 포함
        last = store.add(kIr, frame(41), 40 * kFramePeriodNs + 3 * kNsPerSecond);
        QCOMPARE(last.value_or(DetectionFrame{}).fps, 1.0);
    }

    // 같은 문맥에서 frameId 가 같거나 작으면 버리고, generation 이 작아진 결과도 버린다
    void dropsStaleFrames() {
        DetectionStore store;
        QVERIFY(store.add(kIr, frame(10, 2), 0));
        QVERIFY(!store.add(kIr, frame(10, 2), 1));
        QVERIFY(!store.add(kIr, frame(9, 2), 2));
        QVERIFY(!store.add(kIr, frame(50, 1), 3));
        QCOMPARE(store.history(kIr).size(), std::size_t{1});
        QVERIFY(store.add(kIr, frame(11, 2), 4));
    }

    // generation 이 커지거나 source 가 바뀌면 새 문맥: 기록을 비우고 frameId 가 작아도 받는다
    void startsNewContext() {
        DetectionStore store;
        QVERIFY(store.add(kIr, frame(100, 1), 0));
        QVERIFY(store.add(kIr, frame(101, 1), 1));
        const auto next = store.add(kIr, frame(1, 2), 2);
        QVERIFY(next.has_value());
        QCOMPARE(next.value_or(DetectionFrame{}).fps, 1.0);
        QCOMPARE(store.history(kIr).size(), std::size_t{1});
        QVERIFY(store.add(kIr, frame(1, 1, "ir1"), 3));
        QCOMPARE(store.history(kIr).size(), std::size_t{1});
        QVERIFY(!store.forFrame(kIr, 100));
    }

    // clear 뒤에는 generation 이 처음부터 다시 붙어도 받는다 (송신 쪽 재시작)
    void clearAcceptsRestartedSender() {
        DetectionStore store;
        QVERIFY(store.add(kIr, frame(100, 5), 0));
        QVERIFY(!store.add(kIr, frame(1, 1), 1));
        store.clear(kIr);
        QVERIFY(store.history(kIr).empty());
        QVERIFY(store.add(kIr, frame(1, 1), 2));
    }
};

}  // namespace argus::gcs::link

QTEST_GUILESS_MAIN(argus::gcs::link::DetectionStoreTest)
#include "detection_store_test.moc"
