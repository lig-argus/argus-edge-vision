// 프레임 하나의 영상과 메타데이터를 통제기로 송신하는 인터페이스
// 규약: docs/spec/video.md 5장
// 구현: RtspServer (RTSP 세션의 영상 트랙과 메타데이터 트랙), 테스트용 FakeStreamSink
//      (apps/video_sender/test)
// 사용: VideoSender
#ifndef ARGUS_VIDEO_SENDER_STREAM_SINK_HPP_
#define ARGUS_VIDEO_SENDER_STREAM_SINK_HPP_

#include <string_view>

#include "argus/perception_api/types.hpp"

namespace argus::video_sender {

class IStreamSink {
public:
    IStreamSink() = default;
    virtual ~IStreamSink() = default;
    IStreamSink(const IStreamSink&) = delete;
    IStreamSink& operator=(const IStreamSink&) = delete;
    IStreamSink(IStreamSink&&) = delete;
    IStreamSink& operator=(IStreamSink&&) = delete;

    // 프레임 하나의 영상과 그 프레임의 메타데이터 문서(UTF-8 XML)를 송신한다.
    // 한 호출로 받는 이유: 영상과 메타데이터가 같은 프레임의 것이어야 하고, 구현이 두 트랙에 같은
    // 시각을 부여해야 하기 때문이다. 전송 형식(RTP 패킷 분할 등)은 구현이 정한다.
    // 송신하지 못한 경우(수신하는 클라이언트가 없는 경우 등)에는 폐기하며 호출한 코드에 알리지
    // 않는다. 폐기 횟수는 구현이 제공한다 (RtspServer::stats())
    virtual void push(const perception_api::Frame& frame, std::string_view metadata_document) = 0;
};

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_STREAM_SINK_HPP_
