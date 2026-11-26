// 메타데이터 트랙의 RTP 패킷 → XML 문서 (docs/spec/video.md 3.1)
#include "argus/gcs/rtsp/metadata_reassembler.hpp"

#include <QtTest>

namespace argus::gcs::rtsp {

namespace {

constexpr std::uint8_t kPayloadType = 107;

// RTP 고정 헤더 12 바이트 (PT 107) + payload
QByteArray rtp(std::uint16_t seq, std::uint32_t timestamp, bool marker, const QByteArray& payload) {
    QByteArray packet(12, '\0');
    packet[0] = static_cast<char>(0x80U);  // V=2
    packet[1] = static_cast<char>((marker ? 0x80U : 0x00U) | kPayloadType);
    packet[2] = static_cast<char>(seq >> 8U);
    packet[3] = static_cast<char>(seq & 0xFFU);
    for (std::uint32_t i = 0; i < 4U; ++i) {
        packet[static_cast<qsizetype>(i) + 4] =
            static_cast<char>((timestamp >> (24U - 8U * i)) & 0xFFU);
    }
    return packet + payload;
}

QByteArray xmlOf(const std::optional<MetadataDocument>& document) {
    return document.value_or(MetadataDocument{}).xml;
}

}  // namespace

class MetadataReassemblerTest : public QObject {
    Q_OBJECT

private slots:
    void singlePacketDocument() {
        MetadataReassembler reassembler;
        const std::optional<MetadataDocument> document =
            reassembler.push(rtp(1, 9000, true, "<a/>"));

        QVERIFY(document.has_value());
        QCOMPARE(xmlOf(document), QByteArray("<a/>"));
        QCOMPARE(document.value_or(MetadataDocument{}).rtpTimestamp, std::uint32_t{9000});
    }

    // MTU 를 넘는 문서는 여러 패킷, 마지막에 marker
    void joinsPacketsUntilMarker() {
        MetadataReassembler reassembler;
        QVERIFY(!reassembler.push(rtp(10, 100, false, "<tt:")).has_value());
        QVERIFY(!reassembler.push(rtp(11, 100, false, "Frame")).has_value());
        const std::optional<MetadataDocument> document = reassembler.push(rtp(12, 100, true, "/>"));

        QCOMPARE(xmlOf(document), QByteArray("<tt:Frame/>"));
        QCOMPARE(reassembler.droppedDocuments(), std::uint64_t{0});
    }

    // 가운데 패킷이 빠지면 그 문서는 버리고, 다음 문서는 정상으로 받는다
    void dropsDocumentWithMissingPacket() {
        MetadataReassembler reassembler;
        QVERIFY(!reassembler.push(rtp(1, 100, false, "<a>")).has_value());
        QVERIFY(!reassembler.push(rtp(3, 100, true, "</a>")).has_value());  // 2 가 빠짐
        QCOMPARE(reassembler.droppedDocuments(), std::uint64_t{1});

        QCOMPARE(xmlOf(reassembler.push(rtp(4, 200, true, "<b/>"))), QByteArray("<b/>"));
    }

    // marker 패킷이 빠지면 순번이 끊기므로 다음 문서는 marker 까지 버린다
    void dropsDocumentWithMissingMarker() {
        MetadataReassembler reassembler;
        QVERIFY(!reassembler.push(rtp(1, 100, false, "<a>")).has_value());
        QVERIFY(!reassembler.push(rtp(3, 200, true, "<b/>")).has_value());  // 2 (marker)가 빠짐
        QCOMPARE(reassembler.droppedDocuments(), std::uint64_t{1});

        QCOMPARE(xmlOf(reassembler.push(rtp(4, 300, true, "<c/>"))), QByteArray("<c/>"));
    }

    // 순번이 이어진 채로 타임스탬프만 바뀌면 (marker 없이 다음 문서) 앞 문서를 버린다
    void timestampChangeWithoutMarkerDropsPartial() {
        MetadataReassembler reassembler;
        QVERIFY(!reassembler.push(rtp(1, 100, false, "<a>")).has_value());

        QCOMPARE(xmlOf(reassembler.push(rtp(2, 200, true, "<b/>"))), QByteArray("<b/>"));
        QCOMPARE(reassembler.droppedDocuments(), std::uint64_t{1});
    }

    // 순번은 16 비트라 65535 다음은 0
    void sequenceWrapsAround() {
        MetadataReassembler reassembler;
        QVERIFY(!reassembler.push(rtp(65535, 100, false, "<a>")).has_value());

        QCOMPARE(xmlOf(reassembler.push(rtp(0, 100, true, "</a>"))), QByteArray("<a></a>"));
    }

    void rejectsNonRtp() {
        MetadataReassembler reassembler;
        QVERIFY(!reassembler.push(QByteArray("short")).has_value());
        QByteArray wrongVersion = rtp(1, 100, true, "<a/>");
        wrongVersion[0] = 0x40;  // V=1
        QVERIFY(!reassembler.push(wrongVersion).has_value());

        QCOMPARE(reassembler.badPackets(), std::uint64_t{2});
    }

    // 최대 크기를 넘는 문서는 marker 에서 버린다
    void dropsOversizedDocument() {
        MetadataReassembler reassembler;
        const QByteArray half(MetadataReassembler::kMaxDocumentBytes / 2 + 1, 'x');
        QVERIFY(!reassembler.push(rtp(1, 100, false, half)).has_value());
        QVERIFY(!reassembler.push(rtp(2, 100, true, half)).has_value());

        QCOMPARE(reassembler.droppedDocuments(), std::uint64_t{1});
    }

    // CSRC 목록, 헤더 확장, 패딩은 건너뛴다
    void skipsCsrcExtensionAndPadding() {
        QByteArray packet = rtp(1, 100, true, QByteArray());
        packet[0] = static_cast<char>(0x80U | 0x20U | 0x10U | 0x01U);  // P=1, X=1, CC=1
        packet += QByteArray(4, 'c');                                  // CSRC 1 개
        packet += QByteArray("\xBE\xDE\x00\x01", 4);                   // 확장 머리: 길이 1 워드
        packet += QByteArray(4, 'e');                                  // 확장 1 워드
        packet += "<x/>";
        packet += QByteArray("\x00\x00\x03", 3);  // 패딩 3 바이트 (마지막 바이트가 길이)
        MetadataReassembler reassembler;

        QCOMPARE(xmlOf(reassembler.push(packet)), QByteArray("<x/>"));
    }
};

}  // namespace argus::gcs::rtsp

QTEST_GUILESS_MAIN(argus::gcs::rtsp::MetadataReassemblerTest)
#include "metadata_reassembler_test.moc"
