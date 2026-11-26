// video/stream_info IDL 타입과 video_api 값 타입 사이의 변환.
// 생성된 IDL 타입(argus_msg/video_stream_info.idl 의 argus::idl::VideoStreamInfo)을 사용하는 코드는
// 이 파일의 함수뿐이다. 라이브러리 밖에서는 include 하지 않는다
#ifndef ARGUS_VIDEO_API_BUS_SRC_VIDEO_STREAM_INFO_IDL_HPP_
#define ARGUS_VIDEO_API_BUS_SRC_VIDEO_STREAM_INFO_IDL_HPP_

#include "argus/video_api/types.hpp"
#include "argus_msg/video_stream_info.hpp"

namespace argus::video_api_bus {

void to_idl_video_stream_info(const video_api::VideoStreamInfo& info, idl::VideoStreamInfo& sample);
// 항상 true. enum 의 UNSPECIFIED 와 이 코드에 정의되지 않은 값은 Unknown 으로 바꾼다
bool from_idl_video_stream_info(const idl::VideoStreamInfo& sample,
                                video_api::VideoStreamInfo& info);

}  // namespace argus::video_api_bus

#endif  // ARGUS_VIDEO_API_BUS_SRC_VIDEO_STREAM_INFO_IDL_HPP_
