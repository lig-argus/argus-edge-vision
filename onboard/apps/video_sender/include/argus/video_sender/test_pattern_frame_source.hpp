// 카메라와 탐지 프로세스 없이 영상 송신을 확인하기 위한 시험용 IFrameSource.
// 영상의 내용은 test_pattern_scene.hpp 가 정한다
// 사용: argus_video_sender 를 --test-pattern 옵션으로 실행한 경우 (src/main.cpp)
#ifndef ARGUS_VIDEO_SENDER_TEST_PATTERN_FRAME_SOURCE_HPP_
#define ARGUS_VIDEO_SENDER_TEST_PATTERN_FRAME_SOURCE_HPP_

#include <cstdint>
#include <functional>

#include "argus/core/clock.hpp"
#include "argus/core/handler_list.hpp"
#include "argus/core/subscription.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/video_sender/frame_source.hpp"
#include "argus/video_sender/test_pattern_scene.hpp"

namespace argus::video_sender {

class TestPatternFrameSource : public IFrameSource {
public:
    // wall clock(CLOCK_REALTIME) 의 현재 시각(ns)을 반환하는 함수. Frame.unix_ns 에 사용한다.
    // core::IClock 은 monotonic clock 만 제공하므로 함수로 받는다
    using UnixClock = std::function<std::int64_t()>;

    TestPatternFrameSource(const core::IClock& clock, UnixClock unix_clock,
                           TestPatternConfig config = {});

    [[nodiscard]] core::Subscription subscribe(FrameHandler handler) override;

    // 프레임 하나를 생성해 등록된 handler 를 호출한다. frame_id 는 0 부터 1 씩 증가한다.
    // 30 FPS 로 송신하려면 core::EventLoop::add_periodic() 으로 33 ms 마다 호출한다
    void emit_frame();

private:
    perception_api::Frame make_frame(std::uint64_t frame_id) const;

    const core::IClock& clock_;
    UnixClock unix_clock_;
    TestPatternConfig config_;
    std::uint64_t next_frame_id_ = 0;
    core::HandlerList<const perception_api::Frame&> handlers_;
};

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_TEST_PATTERN_FRAME_SOURCE_HPP_
