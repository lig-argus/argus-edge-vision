// CollectedFrame 하나를 메타데이터 문서로 변환해 영상과 함께 IStreamSink 로 송신한다
// 규약: docs/spec/video.md 5장 (송신부 동작), docs/design/uml/class_video.md 1절
//
// 규칙
// - send() 한 번마다 IStreamSink::push() 를 한 번 호출한다
// - 추적 결과가 없는 프레임도 송신한다. 그 메타데이터는 객체 없는 tt:Frame 이다 (video.md 5장)
//
// 사용: FrameCollector::subscribe() 의 handler 에서 send() 를 호출한다 (src/main.cpp)
// 수명: 생성자에 넘긴 IStreamSink 는 VideoSender 보다 오래 유지되어야 한다
#ifndef ARGUS_VIDEO_SENDER_VIDEO_SENDER_HPP_
#define ARGUS_VIDEO_SENDER_VIDEO_SENDER_HPP_

#include "argus/video_sender/collected_frame.hpp"
#include "argus/video_sender/onvif_metadata_serializer.hpp"
#include "argus/video_sender/stream_sink.hpp"

namespace argus::video_sender {

class VideoSender {
public:
    VideoSender(OnvifMetadataSerializer serializer, IStreamSink& sink);

    void send(const CollectedFrame& collected);

private:
    OnvifMetadataSerializer serializer_;
    IStreamSink& sink_;
};

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_VIDEO_SENDER_HPP_
