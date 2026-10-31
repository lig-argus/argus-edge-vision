// 테스트용 IFrameSource. 테스트가 simulate_frame_received() 로 등록한 프레임을 FakePoller 를 거쳐
// process_events() 안에서 등록된 handler 에게 전달한다. 실제 구현과 같이 handler 는
// process_events() 안에서만 불린다.
#ifndef ARGUS_VIDEO_SENDER_TEST_FAKE_FRAME_SOURCE_HPP_
#define ARGUS_VIDEO_SENDER_TEST_FAKE_FRAME_SOURCE_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <utility>

#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/video_sender/frame_source.hpp"

namespace argus::video_sender {

class FakeFrameSource : public IFrameSource {
public:
    explicit FakeFrameSource(core_testing::FakePoller& poller)
        : poller_(poller), handlers_(std::make_shared<Handlers>()) {}

    [[nodiscard]] core::Subscription subscribe(FrameHandler handler) override {
        const std::uint64_t id = handlers_->next_id++;
        handlers_->by_id[id] = std::move(handler);
        const std::weak_ptr<Handlers> weak = handlers_;
        return core::Subscription([weak, id] {
            if (const auto handlers = weak.lock()) {
                handlers->by_id.erase(id);
            }
        });
    }

    // 테스트용: 프레임 하나를 도착한 것으로 등록한다. 다음 process_events() 에서 handler 에
    // 전달되며, 전달 시점에 등록돼 있는 handler 에게만 간다
    void simulate_frame_received(perception_api::Frame frame) {
        const auto shared_frame = std::make_shared<const perception_api::Frame>(std::move(frame));
        const std::weak_ptr<Handlers> weak = handlers_;
        poller_.post([weak, shared_frame] {
            const auto handlers = weak.lock();
            if (!handlers) {
                return;
            }
            const auto snapshot = handlers->by_id;
            for (const auto& [id, handler] : snapshot) {
                if (handlers->by_id.count(id) != 0) {
                    handler(*shared_frame);
                }
            }
        });
    }

private:
    struct Handlers {
        std::map<std::uint64_t, FrameHandler> by_id;
        std::uint64_t next_id = 1;
    };

    core_testing::FakePoller& poller_;
    std::shared_ptr<Handlers> handlers_;
};

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_TEST_FAKE_FRAME_SOURCE_HPP_
