#include "argus/gcs/rtsp/metadata_reassembler.hpp"

namespace argus::gcs::rtsp {

namespace {

constexpr qsizetype kRtpHeaderBytes = 12;
constexpr std::uint8_t kRtpVersion = 2;
constexpr qsizetype kExtensionHeaderBytes = 4;
constexpr qsizetype kWordBytes = 4;

struct RtpView {
    std::uint16_t seq{0};
    std::uint32_t timestamp{0};
    bool marker{false};
    qsizetype payloadBegin{0};
    qsizetype payloadEnd{0};
};

std::uint8_t byteAt(const QByteArray& bytes, qsizetype at) {
    return static_cast<std::uint8_t>(bytes.at(at));
}

std::uint16_t readU16(const QByteArray& bytes, qsizetype at) {
    return static_cast<std::uint16_t>((static_cast<std::uint32_t>(byteAt(bytes, at)) << 8U) |
                                      byteAt(bytes, at + 1));
}

std::uint32_t readU32(const QByteArray& bytes, qsizetype at) {
    return (static_cast<std::uint32_t>(readU16(bytes, at)) << 16U) | readU16(bytes, at + 2);
}

// RFC 3550 5.1 고정 헤더, CSRC 목록, 헤더 확장, 패딩을 건너뛴 payload 범위
std::optional<RtpView> parseRtp(const QByteArray& packet) {
    const qsizetype size = packet.size();
    if (size < kRtpHeaderBytes) {
        return std::nullopt;
    }
    const std::uint8_t first = byteAt(packet, 0);
    if ((first >> 6U) != kRtpVersion) {
        return std::nullopt;
    }
    const bool padding = (first & 0x20U) != 0;
    const bool extension = (first & 0x10U) != 0;
    const auto csrcCount = static_cast<qsizetype>(first & 0x0FU);
    RtpView view;
    view.marker = (byteAt(packet, 1) & 0x80U) != 0;
    view.seq = readU16(packet, 2);
    view.timestamp = readU32(packet, 4);
    qsizetype begin = kRtpHeaderBytes + kWordBytes * csrcCount;
    if (extension) {
        if (begin + kExtensionHeaderBytes > size) {
            return std::nullopt;
        }
        begin +=
            kExtensionHeaderBytes + kWordBytes * static_cast<qsizetype>(readU16(packet, begin + 2));
    }
    qsizetype end = size;
    if (padding) {
        end -= byteAt(packet, size - 1);
    }
    if (begin > end) {
        return std::nullopt;
    }
    view.payloadBegin = begin;
    view.payloadEnd = end;
    return view;
}

}  // namespace

std::optional<MetadataDocument> MetadataReassembler::push(const QByteArray& rtpPacket) {
    const std::optional<RtpView> rtp = parseRtp(rtpPacket);
    if (!rtp) {
        ++badPackets_;
        return std::nullopt;
    }
    // 앞 패킷이 빠졌으면 지금 패킷이 어느 문서의 중간일 수 있다: 다음 marker 까지 버린다
    const bool gap = lastSeq_ && static_cast<std::uint16_t>(*lastSeq_ + 1U) != rtp->seq;
    lastSeq_ = rtp->seq;
    if (gap && !broken_) {
        discardPartial();  // 이 문서는 marker 에서 버린 것으로 센다
        broken_ = true;
    }
    // 문서 중간에 타임스탬프가 바뀌었다: 앞 문서의 marker 패킷이 빠졌다
    if (!broken_ && partialTimestamp_ && *partialTimestamp_ != rtp->timestamp) {
        discardPartial();
        ++droppedDocuments_;
    }
    if (!broken_) {
        partialTimestamp_ = rtp->timestamp;
        partial_.append(rtpPacket.constData() + rtp->payloadBegin,
                        rtp->payloadEnd - rtp->payloadBegin);
        if (partial_.size() > kMaxDocumentBytes) {
            discardPartial();
            broken_ = true;
        }
    }
    if (!rtp->marker) {
        return std::nullopt;
    }
    // 문서 끝
    if (broken_) {
        broken_ = false;
        ++droppedDocuments_;
        return std::nullopt;
    }
    MetadataDocument document{rtp->timestamp, partial_};
    discardPartial();
    return document;
}

void MetadataReassembler::discardPartial() {
    partial_.clear();
    partialTimestamp_.reset();
}

}  // namespace argus::gcs::rtsp
