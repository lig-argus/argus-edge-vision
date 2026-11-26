// VIDEO_STREAM_INFORMATION (269) 을 만드는 함수. RequestMessageHandler 에 269 로 등록한다
// 규약: docs/spec/video.md 7장 (필드 값), docs/spec/process_interfaces.md 5.1.6
//
// 규칙
// - 값은 영상 송신부가 발행한 video/stream_info 의 최신 값으로 채운다. 받은 적이 없거나 스트림이
//   없으면 DENIED
// - param2(stream id)가 0 이면 모든 스트림, 그 밖에는 그 stream_id 의 스트림 하나. 정수가 아니거나
//   없는 stream_id 면 DENIED
// - uri 의 호스트는 요청한 통제기에 보낼 때 쓰는 자기 주소(IGcsLink::local_address()). 구하지
//   못하거나 uri 가 메시지 필드(160 바이트)에 들어가지 않으면 FAILED
#ifndef ARGUS_ONBOARD_VIDEO_STREAM_INFORMATION_HPP_
#define ARGUS_ONBOARD_VIDEO_STREAM_INFORMATION_HPP_

#include "argus/mavlink_gateway/gcs_link.hpp"
#include "argus/mavlink_gateway/mavlink_command_handler.hpp"
#include "argus/mavlink_gateway/request_message_handler.hpp"
#include "argus/video_api/video_stream_info_source.hpp"

namespace argus::mavlink_gateway {

RequestedMessages build_video_stream_information(const CommandRequest& request,
                                                 const video_api::IVideoStreamInfoSource& source,
                                                 const IGcsLink& link);

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_VIDEO_STREAM_INFORMATION_HPP_
