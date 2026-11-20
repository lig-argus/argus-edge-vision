// CollectedFrame 을 메타데이터 트랙의 ONVIF XML 문서(tt:MetadataStream) 하나로 변환한다
// 규약: docs/spec/video.md 3.2 (XML 구조와 필드 변환), 3.3 (좌표 변환)
// 입력: FrameCollector 가 출력한 CollectedFrame
// 출력: UTF-8 XML 문자열. MetadataRtpPacketizer 의 입력
//
// 규칙
// - 프레임마다 문서 하나를 반환한다. tracks 가 없거나 트랙이 0개이면 tt:Object 가 없는 tt:Frame 을
//   반환한다
// - tt:Frame 의 UtcTime 은 Frame.unix_ns 를 ISO 8601 (ms 단위, Z) 로 변환한 값이다. ms 미만은
//   절사한다
// - 박스 좌표는 영상 픽셀 좌표(원점 왼쪽 위, y 아래 방향)에서 ONVIF 프레임 좌표(원점 왼쪽 아래,
//   y 위 방향)로 변환한다: top = image_height - y1, bottom = image_height - y2
// - 영상 크기는 tracks 가 있으면 Tracks 의 image_width, image_height 를, 없으면 Frame 의 width,
//   height 를 사용한다 (박스 좌표의 기준이 Tracks 의 영상 크기이기 때문)
// - 트랙의 state 가 TrackState 에 정의되지 않은 값이면 argus:State 요소 없이 출력한다
// - 숫자는 지수 표기 없이 소수점을 포함해 출력한다. 좌표는 소수 1자리, 그 밖의 값은 소수 8자리까지
//   출력하고 끝의 0 을 삭제한다. 소수점 문자는 프로세스의 locale 과 무관하게 '.' 이다
// - 문자열(source, label, ONVIF 분류명)은 UTF-8 이어야 한다. XML 특수 문자 5개(& < > " ')는
//   entity 로, 탭과 줄바꿈과 CR 은 문자 참조로 변환하고, 그 밖의 제어 문자(0x20 미만)는 XML 1.0
//   문서에 포함할 수 없으므로 출력하지 않는다
#ifndef ARGUS_VIDEO_SENDER_ONVIF_METADATA_SERIALIZER_HPP_
#define ARGUS_VIDEO_SENDER_ONVIF_METADATA_SERIALIZER_HPP_

#include <cstdint>
#include <map>
#include <string>

#include "argus/video_sender/collected_frame.hpp"

namespace argus::video_sender {

class OnvifMetadataSerializer {
public:
    // class_id 를 tt:Class/tt:Type 에 넣을 ONVIF 분류명(Vehicle 등)으로 변환하는 표 (video.md 8장).
    // 표에 없는 class_id 의 트랙은 tt:Class 요소 없이 출력한다. argus:ClassId 와 argus:Label 은
    // 표와 무관하게 항상 출력한다
    using OnvifTypeByClassId = std::map<std::int32_t, std::string>;

    explicit OnvifMetadataSerializer(OnvifTypeByClassId onvif_type_by_class_id = {});

    std::string serialize(const CollectedFrame& collected) const;

private:
    OnvifTypeByClassId onvif_type_by_class_id_;
};

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_ONVIF_METADATA_SERIALIZER_HPP_
