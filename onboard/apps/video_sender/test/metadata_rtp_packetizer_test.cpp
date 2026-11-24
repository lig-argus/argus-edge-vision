#include "argus/video_sender/metadata_rtp_packetizer.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace argus::video_sender {
namespace {

using Packet = MetadataRtpPacketizer::Packet;

constexpr std::size_t kHeaderBytes = MetadataRtpPacketizer::kHeaderBytes;
constexpr std::uint32_t kTimestamp = 0x01020304;
constexpr std::uint8_t kPayloadType = 107;
constexpr std::uint32_t kSsrc = 0xA1B2C3D4;

bool marker_of(const Packet& packet) { return (packet.at(1) & 0x80U) != 0; }

std::uint8_t payload_type_of(const Packet& packet) {
    return static_cast<std::uint8_t>(packet.at(1) & 0x7FU);
}

std::uint16_t sequence_number_of(const Packet& packet) {
    return static_cast<std::uint16_t>((static_cast<std::uint32_t>(packet.at(2)) << 8U) |
                                      static_cast<std::uint32_t>(packet.at(3)));
}

std::uint32_t u32_at(const Packet& packet, std::size_t offset) {
    return (static_cast<std::uint32_t>(packet.at(offset)) << 24U) |
           (static_cast<std::uint32_t>(packet.at(offset + 1)) << 16U) |
           (static_cast<std::uint32_t>(packet.at(offset + 2)) << 8U) |
           static_cast<std::uint32_t>(packet.at(offset + 3));
}

std::uint32_t timestamp_of(const Packet& packet) { return u32_at(packet, 4); }
std::uint32_t ssrc_of(const Packet& packet) { return u32_at(packet, 8); }

std::string payload_of(const Packet& packet) {
    return {packet.begin() + static_cast<std::ptrdiff_t>(kHeaderBytes), packet.end()};
}

MetadataRtpConfig config_with_max_packet_bytes(std::size_t max_packet_bytes) {
    MetadataRtpConfig config{kPayloadType, kSsrc};
    config.max_packet_bytes = max_packet_bytes;
    return config;
}

TEST(MetadataRtpPacketizerTest, WritesHeaderFields) {
    MetadataRtpConfig config{kPayloadType, kSsrc};
    config.first_sequence_number = 0x1234;
    MetadataRtpPacketizer packetizer{config};

    const std::vector<Packet> packets = packetizer.packetize("<a/>", kTimestamp);

    ASSERT_EQ(packets.size(), 1U);
    const Packet& packet = packets[0];
    ASSERT_EQ(packet.size(), kHeaderBytes + 4);
    EXPECT_EQ(packet[0], 0x80);  // V=2, padding 없음, 확장 없음, CSRC 0개
    EXPECT_EQ(payload_type_of(packet), kPayloadType);
    EXPECT_EQ(sequence_number_of(packet), 0x1234);
    EXPECT_EQ(timestamp_of(packet), kTimestamp);
    EXPECT_EQ(ssrc_of(packet), kSsrc);
    EXPECT_EQ(payload_of(packet), "<a/>");
}

TEST(MetadataRtpPacketizerTest, SinglePacketHasMarker) {
    MetadataRtpPacketizer packetizer{MetadataRtpConfig{kPayloadType, kSsrc}};

    const std::vector<Packet> packets = packetizer.packetize("<a/>", kTimestamp);

    ASSERT_EQ(packets.size(), 1U);
    EXPECT_TRUE(marker_of(packets[0]));
}

TEST(MetadataRtpPacketizerTest, DocumentOfExactlyMaxPayloadFitsInOnePacket) {
    MetadataRtpPacketizer packetizer{config_with_max_packet_bytes(kHeaderBytes + 10)};

    const std::vector<Packet> packets = packetizer.packetize(std::string(10, 'x'), kTimestamp);

    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(packets[0].size(), kHeaderBytes + 10);
}

TEST(MetadataRtpPacketizerTest, SplitsDocumentLargerThanMaxPayload) {
    MetadataRtpPacketizer packetizer{config_with_max_packet_bytes(kHeaderBytes + 10)};
    const std::string xml = "0123456789abcdefghijKLMNO";  // 25 바이트 → 10, 10, 5

    const std::vector<Packet> packets = packetizer.packetize(xml, kTimestamp);

    ASSERT_EQ(packets.size(), 3U);
    EXPECT_EQ(payload_of(packets[0]), "0123456789");
    EXPECT_EQ(payload_of(packets[1]), "abcdefghij");
    EXPECT_EQ(payload_of(packets[2]), "KLMNO");
    EXPECT_FALSE(marker_of(packets[0]));
    EXPECT_FALSE(marker_of(packets[1]));
    EXPECT_TRUE(marker_of(packets[2]));
}

TEST(MetadataRtpPacketizerTest, PacketsOfOneDocumentShareTimestampAndSsrc) {
    const MetadataRtpConfig config = config_with_max_packet_bytes(kHeaderBytes + 10);
    MetadataRtpPacketizer packetizer{config};

    const std::vector<Packet> packets = packetizer.packetize(std::string(25, 'x'), kTimestamp);

    ASSERT_EQ(packets.size(), 3U);
    for (const Packet& packet : packets) {
        EXPECT_EQ(timestamp_of(packet), kTimestamp);
        EXPECT_EQ(ssrc_of(packet), kSsrc);
    }
}

TEST(MetadataRtpPacketizerTest, DefaultMaxPacketBytesIs1400) {
    MetadataRtpPacketizer packetizer{MetadataRtpConfig{kPayloadType, kSsrc}};

    const std::vector<Packet> packets = packetizer.packetize(std::string(1389, 'x'), kTimestamp);

    ASSERT_EQ(packets.size(), 2U);
    EXPECT_EQ(packets[0].size(), 1400U);
    EXPECT_EQ(packets[1].size(), kHeaderBytes + 1);
}

TEST(MetadataRtpPacketizerTest, SequenceNumberContinuesAcrossDocuments) {
    MetadataRtpConfig config = config_with_max_packet_bytes(kHeaderBytes + 10);
    config.first_sequence_number = 100;
    MetadataRtpPacketizer packetizer{config};

    const std::vector<Packet> first = packetizer.packetize(std::string(15, 'x'), kTimestamp);
    const std::vector<Packet> second = packetizer.packetize("<a/>", kTimestamp + 3000);

    ASSERT_EQ(first.size(), 2U);
    ASSERT_EQ(second.size(), 1U);
    EXPECT_EQ(sequence_number_of(first[0]), 100);
    EXPECT_EQ(sequence_number_of(first[1]), 101);
    EXPECT_EQ(sequence_number_of(second[0]), 102);
    EXPECT_EQ(timestamp_of(second[0]), kTimestamp + 3000);
}

TEST(MetadataRtpPacketizerTest, SequenceNumberWrapsAfter65535) {
    MetadataRtpConfig config = config_with_max_packet_bytes(kHeaderBytes + 10);
    config.first_sequence_number = 65535;
    MetadataRtpPacketizer packetizer{config};

    const std::vector<Packet> packets = packetizer.packetize(std::string(15, 'x'), kTimestamp);

    ASSERT_EQ(packets.size(), 2U);
    EXPECT_EQ(sequence_number_of(packets[0]), 65535);
    EXPECT_EQ(sequence_number_of(packets[1]), 0);
}

TEST(MetadataRtpPacketizerTest, EmptyDocumentMakesNoPacket) {
    MetadataRtpConfig config{kPayloadType, kSsrc};
    config.first_sequence_number = 5;
    MetadataRtpPacketizer packetizer{config};

    EXPECT_TRUE(packetizer.packetize("", kTimestamp).empty());

    // 빈 문서는 sequence number 를 사용하지 않는다
    const std::vector<Packet> packets = packetizer.packetize("<a/>", kTimestamp);
    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(sequence_number_of(packets[0]), 5);
}

TEST(MetadataRtpPacketizerTest, PayloadTypeUsesLower7Bits) {
    MetadataRtpConfig config = config_with_max_packet_bytes(kHeaderBytes + 2);
    config.payload_type = 0xFF;
    MetadataRtpPacketizer packetizer{config};

    const std::vector<Packet> packets = packetizer.packetize("<a/>", kTimestamp);

    // payload_type 의 최상위 비트가 marker 로 출력되지 않는다
    ASSERT_EQ(packets.size(), 2U);
    EXPECT_EQ(payload_type_of(packets[0]), 0x7F);
    EXPECT_FALSE(marker_of(packets[0]));
    EXPECT_TRUE(marker_of(packets[1]));
}

TEST(MetadataRtpPacketizerTest, MaxPacketBytesBelowHeaderIsHandledAsOnePayloadByte) {
    MetadataRtpPacketizer packetizer{config_with_max_packet_bytes(0)};

    const std::vector<Packet> packets = packetizer.packetize("abc", kTimestamp);

    ASSERT_EQ(packets.size(), 3U);
    EXPECT_EQ(packets[0].size(), kHeaderBytes + 1);
    EXPECT_TRUE(marker_of(packets[2]));
}

TEST(MetadataRtpPacketizerTest, JoinedPayloadsEqualDocument) {
    MetadataRtpPacketizer packetizer{config_with_max_packet_bytes(kHeaderBytes + 7)};
    const std::string xml = R"(<tt:MetadataStream xmlns:tt="http://www.onvif.org/ver10/schema"/>)";

    std::string joined;
    for (const Packet& packet : packetizer.packetize(xml, kTimestamp)) {
        joined += payload_of(packet);
    }

    EXPECT_EQ(joined, xml);
}

}  // namespace
}  // namespace argus::video_sender
