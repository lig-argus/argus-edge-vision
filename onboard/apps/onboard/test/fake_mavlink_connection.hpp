// 테스트용 IMavlinkConnection. 보낸 메시지를 기록하고, 테스트가 넣은 메시지는 FakePoller 를 거쳐
// process_events() 안에서 등록된 handler 에게 간다.
#ifndef ARGUS_ONBOARD_TEST_FAKE_MAVLINK_CONNECTION_HPP_
#define ARGUS_ONBOARD_TEST_FAKE_MAVLINK_CONNECTION_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/mavlink.h"
#include "argus/onboard/mavlink_connection.hpp"

namespace argus::onboard {

class FakeMavlinkConnection : public IMavlinkConnection {
public:
    explicit FakeMavlinkConnection(core_testing::FakePoller& poller)
        : poller_(poller), handlers_(std::make_shared<Handlers>()) {}

    bool send(const mavlink_message_t& message) override {
        sent_.push_back(message);
        return send_result_;
    }

    [[nodiscard]] core::Subscription subscribe(std::uint32_t msgid, Handler handler) override {
        const std::uint64_t id = handlers_->next_id++;
        handlers_->by_msgid[msgid][id] = std::move(handler);
        const std::weak_ptr<Handlers> weak = handlers_;
        return core::Subscription([weak, msgid, id] {
            if (const auto handlers = weak.lock()) {
                handlers->by_msgid[msgid].erase(id);
            }
        });
    }

    // 테스트용: FC 나 통제기가 보낸 메시지 하나를 도착한 것으로 등록한다 (다음 process_events()
    // 에서 전달)
    void simulate_message_received(const mavlink_message_t& message) {
        const std::weak_ptr<Handlers> weak = handlers_;
        const auto shared_message = std::make_shared<const mavlink_message_t>(message);
        poller_.post([weak, shared_message] {
            const auto handlers = weak.lock();
            if (!handlers) {
                return;
            }
            const auto found = handlers->by_msgid.find(shared_message->msgid);
            if (found == handlers->by_msgid.end()) {
                return;
            }
            const auto copy = found->second;  // handler 가 등록을 풀어도 안전하게
            for (const auto& [id, handler] : copy) {
                handler(*shared_message);
            }
        });
    }

    // 테스트용: send() 로 보낸 메시지 기록, send() 의 반환값 지정
    const std::vector<mavlink_message_t>& sent() const { return sent_; }
    void set_send_result(bool result) { send_result_ = result; }

private:
    struct Handlers {
        std::uint64_t next_id = 1;
        std::map<std::uint32_t, std::map<std::uint64_t, Handler>> by_msgid;
    };

    core_testing::FakePoller& poller_;
    std::shared_ptr<Handlers> handlers_;
    std::vector<mavlink_message_t> sent_;
    bool send_result_ = true;
};

}  // namespace argus::onboard

#endif  // ARGUS_ONBOARD_TEST_FAKE_MAVLINK_CONNECTION_HPP_
