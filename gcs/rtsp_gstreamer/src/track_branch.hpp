// rtspsrc 의 트랙 출력 단자 하나에 그 트랙의 처리 부분(bin)을 붙인다. 수신기(GstRtspReceiver)
// 내부용.
//   영상 트랙:        <해독 요소> ! videoconvert ! RGBx ! appsink   (+ 해독 전 RTP 패킷을 보는
//   probe) 메타데이터 트랙:  appsink (RTP 패킷 그대로) 그 밖의 트랙:     fakesink (받고 버린다)
// 콜백(ITrackSink)은 GStreamer 스레드에서 불린다
#ifndef ARGUS_GCS_RTSP_GSTREAMER_TRACK_BRANCH_HPP_
#define ARGUS_GCS_RTSP_GSTREAMER_TRACK_BRANCH_HPP_

#include <gst/gst.h>

#include <QString>
#include <cstdint>
#include <optional>

#include "argus/gcs/rtsp/rtsp_receiver.hpp"

namespace argus::gcs::rtsp_gstreamer {

class ITrackSink {
public:
    ITrackSink() = default;
    virtual ~ITrackSink() = default;
    ITrackSink(const ITrackSink&) = delete;
    ITrackSink& operator=(const ITrackSink&) = delete;
    ITrackSink(ITrackSink&&) = delete;
    ITrackSink& operator=(ITrackSink&&) = delete;

    // 해독 전 영상 RTP 패킷 하나
    virtual void onVideoRtpPacket(GstBuffer* buffer) = 0;
    // 해독한 영상 프레임 (utcNs 는 비어 있다). pts 는 버퍼의 표시 시각, 없으면 없음
    virtual void onDecodedFrame(rtsp::DecodedFrame frame, std::optional<std::uint64_t> pts) = 0;
    // 메타데이터 RTP 패킷 하나 (헤더 포함)
    virtual void onMetadataRtpPacket(GstBuffer* buffer) = 0;
};

// gst_init 을 한 번만 부른다 (여러 스레드에서 불러도 된다)
void ensureGstreamerInitialized();

// pad 의 caps(media, encoding-name)를 보고 bin 을 만들어 pipeline 에 넣고 pad 에 연결한다.
// sink 는 pipeline 이 멈출 때까지 살아 있어야 한다. 실패하면 오류 글
std::optional<QString> attachTrack(GstBin* pipeline, GstPad* pad, ITrackSink& sink);

}  // namespace argus::gcs::rtsp_gstreamer

#endif  // ARGUS_GCS_RTSP_GSTREAMER_TRACK_BRANCH_HPP_
