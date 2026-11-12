// 메타데이터 트랙의 XML 문서 하나를 RTP 패킷으로 나눈다
// 규약: docs/spec/video.md 3.1 (RTP 규칙), 5장 (송신부가 만든 RTP 패킷을 appsrc 에 넣는다)
// 입력: OnvifMetadataSerializer 가 반환한 XML 문자열
// 출력: RTP 패킷(헤더 12 바이트 + XML 조각)의 목록. RtspServer 의 입력
//
// 규칙
// - 패킷 하나의 크기는 헤더를 포함해 max_packet_bytes 이하다. 문서가 한 패킷에 들어가지 않으면
//   순서대로 여러 패킷으로 나눈다
// - 문서의 마지막 패킷만 marker 가 1 이다
// - 한 문서의 모든 패킷은 타임스탬프가 같다. 값은 호출하는 코드가 정한다
// - sequence number 는 패킷마다 1 증가하며 65535 다음은 0 이다. 문서가 바뀌어도 이어진다
// - 빈 문서에 대해서는 패킷을 반환하지 않는다
#ifndef ARGUS_VIDEO_SENDER_METADATA_RTP_PACKETIZER_HPP_
#define ARGUS_VIDEO_SENDER_METADATA_RTP_PACKETIZER_HPP_

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

namespace argus::video_sender {

struct MetadataRtpConfig {
    // payload_type 과 ssrc 는 기본값이 없다. RtspServer 의 메타데이터 트랙 설정과 같은 값을
    // 이 설정을 생성하는 코드가 지정한다
    MetadataRtpConfig(std::uint8_t payload_type_value, std::uint32_t ssrc_value)
        : payload_type(payload_type_value), ssrc(ssrc_value) {}

    // RTP 헤더의 payload type. 수신기가 이 번호와 SDP 의 a=rtpmap 줄로 패킷의 형식
    // (vnd.onvif.metadata)을 확인한다. 형식이 미리 정해지지 않은 동적 번호(96~127) 중에서 정하며,
    // 영상 트랙의 번호(26)와 달라야 한다. 하위 7 비트만 사용한다
    std::uint8_t payload_type;

    // RTP 헤더의 SSRC. RFC 3550 은 무작위 값으로 정하도록 한다
    std::uint32_t ssrc;

    // 최초 패킷의 sequence number. RFC 3550 은 무작위 값으로 정하도록 한다
    std::uint16_t first_sequence_number = 0;

    // 헤더 12 바이트를 포함한 패킷 최대 크기. GStreamer RTP 페이로더의 기본 MTU 와 같은 값이다.
    // 13 미만이면 13 으로 처리한다
    std::size_t max_packet_bytes = 1400;
};

class MetadataRtpPacketizer {
public:
    using Packet = std::vector<std::uint8_t>;

    static constexpr std::size_t kHeaderBytes = 12;

    explicit MetadataRtpPacketizer(const MetadataRtpConfig& config);

    // rtp_timestamp: 90 kHz 클럭의 RTP 타임스탬프 (video.md 3.1)
    std::vector<Packet> packetize(std::string_view xml, std::uint32_t rtp_timestamp);

private:
    Packet make_packet(std::string_view payload, std::uint32_t rtp_timestamp, bool marker);

    std::uint8_t payload_type_;
    std::uint32_t ssrc_;
    std::uint16_t next_sequence_number_;
    std::size_t max_payload_bytes_;
};

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_METADATA_RTP_PACKETIZER_HPP_
