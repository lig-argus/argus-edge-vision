#include "argus/video_sender/metadata_rtp_packetizer.hpp"

#include <algorithm>

namespace argus::video_sender {
namespace {

constexpr std::uint8_t kVersion2 = 0x80;  // V=2, padding 없음, 확장 없음, CSRC 0개
constexpr std::uint8_t kMarkerBit = 0x80;
constexpr std::uint8_t kPayloadTypeMask = 0x7F;

// 네트워크 바이트 순서(big-endian)로 추가한다
void append_u16(MetadataRtpPacketizer::Packet& packet, std::uint16_t value) {
    packet.push_back(static_cast<std::uint8_t>(value >> 8U));
    packet.push_back(static_cast<std::uint8_t>(value & 0xFFU));
}

void append_u32(MetadataRtpPacketizer::Packet& packet, std::uint32_t value) {
    append_u16(packet, static_cast<std::uint16_t>(value >> 16U));
    append_u16(packet, static_cast<std::uint16_t>(value & 0xFFFFU));
}

}  // namespace

MetadataRtpPacketizer::MetadataRtpPacketizer(const MetadataRtpConfig& config)
    : payload_type_(config.payload_type & kPayloadTypeMask),
      ssrc_(config.ssrc),
      next_sequence_number_(config.first_sequence_number),
      max_payload_bytes_(std::max(config.max_packet_bytes, kHeaderBytes + 1) - kHeaderBytes) {}

std::vector<MetadataRtpPacketizer::Packet> MetadataRtpPacketizer::packetize(
    std::string_view xml, std::uint32_t rtp_timestamp) {
    std::vector<Packet> packets;
    packets.reserve((xml.size() + max_payload_bytes_ - 1) / max_payload_bytes_);
    while (!xml.empty()) {
        const std::string_view payload = xml.substr(0, max_payload_bytes_);
        xml.remove_prefix(payload.size());
        const bool is_last = xml.empty();
        packets.push_back(make_packet(payload, rtp_timestamp, is_last));
    }
    return packets;
}

MetadataRtpPacketizer::Packet MetadataRtpPacketizer::make_packet(std::string_view payload,
                                                                 std::uint32_t rtp_timestamp,
                                                                 bool marker) {
    Packet packet;
    packet.reserve(kHeaderBytes + payload.size());
    packet.push_back(kVersion2);
    packet.push_back(marker ? static_cast<std::uint8_t>(kMarkerBit | payload_type_)
                            : payload_type_);
    append_u16(packet, next_sequence_number_++);
    append_u32(packet, rtp_timestamp);
    append_u32(packet, ssrc_);
    packet.insert(packet.end(), payload.begin(), payload.end());
    return packet;
}

}  // namespace argus::video_sender
