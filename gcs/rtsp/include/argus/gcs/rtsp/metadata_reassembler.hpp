// RTSP 메타데이터 트랙의 RTP 패킷을 XML 문서로 결합한다 (docs/spec/video.md 3.1).
// 문서 하나가 MTU 를 넘으면 여러 패킷으로 나뉘고 마지막 패킷에 marker = 1 이 붙는다.
// 패킷이 빠지면(순번이 건너뛰거나 문서 중간에 타임스탬프가 바뀌면) 그 문서는 버린다
#ifndef ARGUS_GCS_RTSP_METADATA_REASSEMBLER_HPP_
#define ARGUS_GCS_RTSP_METADATA_REASSEMBLER_HPP_

#include <QByteArray>
#include <cstdint>
#include <optional>

namespace argus::gcs::rtsp {

struct MetadataDocument {
    // 프레임 unix_ns 의 90 kHz 값 (짝짓기에는 쓰지 않는다, video.md 3.1)
    std::uint32_t rtpTimestamp{0};

    QByteArray xml;
};

class MetadataReassembler {
public:
    // 문서 하나의 최대 크기. 넘으면 깨진 문서로 보고 버린다 (프레임 하나의 XML 은 수 KB)
    static constexpr qsizetype kMaxDocumentBytes = qsizetype{256} * 1024;

    // RTP 패킷 하나(헤더 포함)를 넣는다. 문서가 끝나면 그 문서를 돌려준다
    std::optional<MetadataDocument> push(const QByteArray& rtpPacket);

    // 문서가 끝나기 전에 패킷이 빠져 버린 문서 수
    std::uint64_t droppedDocuments() const { return droppedDocuments_; }
    // RTP 형식이 아닌 패킷 수
    std::uint64_t badPackets() const { return badPackets_; }

private:
    void discardPartial();  // 모으던 조각을 버린다 (세지는 않는다)

    QByteArray partial_;
    std::optional<std::uint32_t> partialTimestamp_;
    std::optional<std::uint16_t> lastSeq_;
    bool broken_{false};  // 지금 문서에 빠진 패킷이 있음 (marker 까지 버린다)
    std::uint64_t droppedDocuments_{0};
    std::uint64_t badPackets_{0};
};

}  // namespace argus::gcs::rtsp

#endif  // ARGUS_GCS_RTSP_METADATA_REASSEMBLER_HPP_
