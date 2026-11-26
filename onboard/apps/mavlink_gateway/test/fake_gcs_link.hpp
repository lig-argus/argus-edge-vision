// 테스트용 IGcsLink. 보낸 메시지를 대상과 함께 기록하고, 테스트가 넣은 메시지를 등록된 handler 에
// 바로 전달한다 (실제 구현은 process_events() 안에서 전달)
#ifndef ARGUS_ONBOARD_TEST_FAKE_GCS_LINK_HPP_
#define ARGUS_ONBOARD_TEST_FAKE_GCS_LINK_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/gcs_handler_registry.hpp"
#include "argus/mavlink_gateway/gcs_link.hpp"

namespace argus::mavlink_gateway {

class FakeGcsLink : public IGcsLink {
public:
    // 보낸 메시지 하나. to 가 nullopt 이면 broadcast
    struct Sent {
        std::optional<GcsPeer> to;
        mavlink_message_t message{};
    };

    bool broadcast(const mavlink_message_t& message) override {
        sent_.push_back(Sent{std::nullopt, message});
        return true;
    }

    bool send_to(const GcsPeer& peer, const mavlink_message_t& message) override {
        sent_.push_back(Sent{peer, message});
        return true;
    }

    [[nodiscard]] core::Subscription subscribe(std::uint32_t msgid, Handler handler) override {
        return handlers_.add(msgid, std::move(handler));
    }

    std::optional<std::string> local_address(const GcsPeer& peer) const override {
        const auto found = local_addresses_.find(peer.address);
        if (found == local_addresses_.end()) {
            return std::nullopt;
        }
        return found->second;
    }

    // 테스트용 조작
    void simulate_message_received(const mavlink_message_t& message, const GcsPeer& sender) {
        handlers_.dispatch(message, sender);
    }
    void set_local_address(const std::string& peer_address, const std::string& local) {
        local_addresses_[peer_address] = local;
    }
    const std::vector<Sent>& sent() const { return sent_; }
    void clear_sent() { sent_.clear(); }

private:
    GcsHandlerRegistry handlers_;
    std::vector<Sent> sent_;
    std::map<std::string, std::string> local_addresses_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_TEST_FAKE_GCS_LINK_HPP_
