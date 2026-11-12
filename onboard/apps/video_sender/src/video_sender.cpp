#include "argus/video_sender/video_sender.hpp"

#include <utility>

namespace argus::video_sender {

VideoSender::VideoSender(OnvifMetadataSerializer serializer, IStreamSink& sink)
    : serializer_(std::move(serializer)), sink_(sink) {}

void VideoSender::send(const CollectedFrame& collected) {
    sink_.push(collected.frame, serializer_.serialize(collected));
}

}  // namespace argus::video_sender
