// STATUSTEXT 조각 모으기 테스트: 한 조각 문구, 나뉜 문구, id 0 으로 나뉜 문구, 끊긴 조각, 심각도
#include "argus/gcs/link/status_text.hpp"

#include <QtTest>
#include <cstdint>
#include <cstring>

namespace argus::gcs::link {
namespace {

// 조각 하나. 50 자를 채우면 NUL 이 없다 (다음 조각이 있다는 뜻)
mavlink_statustext_t chunk(const char* text, std::uint16_t id, std::uint8_t seq,
                           std::uint8_t severity) {
    mavlink_statustext_t out{};
    std::strncpy(static_cast<char*>(out.text), text, sizeof(out.text));
    out.id = id;
    out.chunk_seq = seq;
    out.severity = severity;
    return out;
}

mavlink_statustext_t chunk(const char* text, std::uint16_t id, std::uint8_t seq) {
    return chunk(text, id, seq, MAV_SEVERITY_INFO);
}

constexpr const char* kFull = "12345678901234567890123456789012345678901234567890";  // 50 자
constexpr std::int64_t kMs = kNsPerMs;

}  // namespace

class StatusTextTest : public QObject {
    Q_OBJECT

private slots:
    void singleChunkIsOneLine() {
        StatusTextAssembler assembler;
        const auto lines = assembler.add(chunk(" Takeoff detected ", 0, 0, MAV_SEVERITY_ERROR), 0);
        QCOMPARE(lines.size(), std::size_t{1});
        QCOMPARE(lines.front().text, QStringLiteral("Takeoff detected"));
        QCOMPARE(lines.front().level, LogLevel::Serious);
    }

    void chunksWithSameIdJoin() {
        StatusTextAssembler assembler;
        QVERIFY(assembler.add(chunk(kFull, 7, 0, MAV_SEVERITY_WARNING), 0).empty());
        QVERIFY(assembler.add(chunk(kFull, 7, 1), 10 * kMs).empty());
        const auto lines = assembler.add(chunk("end", 7, 2), 20 * kMs);
        QCOMPARE(lines.size(), std::size_t{1});
        QCOMPARE(lines.front().text,
                 QString::fromLatin1(kFull) + QString::fromLatin1(kFull) + QStringLiteral("end"));
        QCOMPARE(lines.front().level, LogLevel::Caution);  // 첫 조각의 심각도
    }

    // id 0 이라도 50 자를 채운 조각은 다음 조각과 모은다
    void idZeroFullChunkJoinsNext() {
        StatusTextAssembler assembler;
        QVERIFY(assembler.add(chunk(kFull, 0, 0), 0).empty());
        const auto lines = assembler.add(chunk("tail", 0, 1), 10 * kMs);
        QCOMPARE(lines.size(), std::size_t{1});
        QCOMPARE(lines.front().text, QString::fromLatin1(kFull) + QStringLiteral("tail"));
    }

    // 다른 문구가 끼면 모으던 것을 먼저 낸다
    void otherTextFlushesPending() {
        StatusTextAssembler assembler;
        QVERIFY(assembler.add(chunk(kFull, 5, 0), 0).empty());
        const auto lines = assembler.add(chunk("other", 0, 0, MAV_SEVERITY_WARNING), 10 * kMs);
        QCOMPARE(lines.size(), std::size_t{2});
        QCOMPARE(lines.at(0).text, QString::fromLatin1(kFull));
        QCOMPARE(lines.at(1).text, QStringLiteral("other"));
        QCOMPARE(lines.at(1).level, LogLevel::Caution);
    }

    // 순서가 어긋나면(조각을 잃으면) 모으던 것을 내고 새로 모은다
    void sequenceGapFlushesPending() {
        StatusTextAssembler assembler;
        QVERIFY(assembler.add(chunk(kFull, 3, 0), 0).empty());
        const auto lines = assembler.add(chunk("late", 3, 2), 10 * kMs);
        QCOMPARE(lines.size(), std::size_t{2});
        QCOMPARE(lines.at(0).text, QString::fromLatin1(kFull));
        QCOMPARE(lines.at(1).text, QStringLiteral("late"));
    }

    // 마지막 조각이 오지 않으면 마지막 조각 뒤 kTimeoutNs 가 지나야 낸다
    void pendingExpiresAfterTimeout() {
        StatusTextAssembler assembler;
        const std::int64_t start = 100 * kMs;
        QVERIFY(assembler.add(chunk(kFull, 6, 0), start).empty());
        QVERIFY(!assembler.expire(start + StatusTextAssembler::kTimeoutNs));
        const auto line = assembler.expire(start + StatusTextAssembler::kTimeoutNs + 1);
        QCOMPARE(line.value_or(StatusTextLine{}).text, QString::fromLatin1(kFull));
        QVERIFY(!assembler.expire(start + 10 * StatusTextAssembler::kTimeoutNs));
    }

    void severityMapsToLogLevel() {
        QCOMPARE(statusTextLevel(MAV_SEVERITY_EMERGENCY), LogLevel::Critical);
        QCOMPARE(statusTextLevel(MAV_SEVERITY_ALERT), LogLevel::Critical);
        QCOMPARE(statusTextLevel(MAV_SEVERITY_CRITICAL), LogLevel::Critical);
        QCOMPARE(statusTextLevel(MAV_SEVERITY_ERROR), LogLevel::Serious);
        QCOMPARE(statusTextLevel(MAV_SEVERITY_WARNING), LogLevel::Caution);
        QCOMPARE(statusTextLevel(MAV_SEVERITY_NOTICE), LogLevel::Off);
        QCOMPARE(statusTextLevel(MAV_SEVERITY_INFO), LogLevel::Off);
        QCOMPARE(statusTextLevel(MAV_SEVERITY_DEBUG), LogLevel::Off);
    }
};

}  // namespace argus::gcs::link

QTEST_GUILESS_MAIN(argus::gcs::link::StatusTextTest)
#include "status_text_test.moc"
