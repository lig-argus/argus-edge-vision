// 비압축 프레임을 공급하는 인터페이스
// 규약: docs/spec/process_interfaces.md 5.1 vision/frame/gray8, 5.3
// 구현: (다음 PR) ZmqFrameSource (vision/frame/gray8 구독), FileFrameSource (녹화 파일 재생),
//      테스트용 FakeFrameSource (apps/video_sender/test)
// 사용: FrameCollector
#ifndef ARGUS_VIDEO_SENDER_FRAME_SOURCE_HPP_
#define ARGUS_VIDEO_SENDER_FRAME_SOURCE_HPP_

#include <functional>

#include "argus/core/subscription.hpp"
#include "argus/perception_api/types.hpp"

namespace argus::video_sender {

class IFrameSource {
public:
    // 프레임은 참조로 넘긴다 (픽셀 버퍼 복사를 피하기 위해). 보관하려면 handler 가 복사한다
    using FrameHandler = std::function<void(const perception_api::Frame&)>;

    IFrameSource() = default;
    virtual ~IFrameSource() = default;
    IFrameSource(const IFrameSource&) = delete;
    IFrameSource& operator=(const IFrameSource&) = delete;
    IFrameSource(IFrameSource&&) = delete;
    IFrameSource& operator=(IFrameSource&&) = delete;

    // 프레임이 올 때마다 handler 를 부른다. 여러 번 등록하면 모두 부른다.
    // handler 는 core::IPoller::process_events() 안에서, 그 함수를 부른 스레드에서 불린다.
    // 반환된 Subscription 이 사라지면 그 handler 는 더 불리지 않는다.
    [[nodiscard]] virtual core::Subscription subscribe(FrameHandler handler) = 0;
};

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_FRAME_SOURCE_HPP_
