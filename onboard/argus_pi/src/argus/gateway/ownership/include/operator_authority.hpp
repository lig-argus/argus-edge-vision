#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "argus/gateway/common/include/python_compat.hpp"

namespace argus::gateway {
class OperatorAuthority {
public:
    std::recursive_mutex& mutex() { return mutex_; }
    bool emergency() const { return emergency_.load(); }
    void set_emergency(bool enabled) { emergency_.store(enabled); }
    std::optional<std::string> owner() const { return std::atomic_load(&view_)->owner; }
    nlohmann::json snapshot() const {
        const auto view = std::atomic_load(&view_);
        return {{"supported", true},
                {"owner", view->owner ? nlohmann::json(*view->owner) : nlohmann::json()},
                {"revision", view->revision},
                {"emergency_latched", emergency()}};
    }
    std::uint64_t admit(const std::string& client, bool emergency_request = false) {
        validate(client, "관제 세션 ID 오류");
        if (emergency_request) {
            return std::atomic_load(&view_)->revision;
        }
        const std::scoped_lock lock(mutex_);
        if (emergency()) {
            throw ValueError("긴급 착륙 후 지상·무장 해제를 확인하고 제어권을 다시 가져오세요");
        }
        const auto previous = owner();
        if (previous && previous != client) {
            // 2026-11-19 사용자 지시(메인 GUI 우선): 제어권을 가진 GUI와 가져오는 방법을
            // 알린다.
            if (previous->rfind("main:", 0) == 0) {
                throw ValueError(
                    "메인 GUI가 제어권을 사용 중입니다. "
                    "정비 GUI에서 \"이 GUI로 제어권 가져오기\"를 누르세요");
            }
            throw ValueError("다른 GUI가 제어권을 사용 중입니다. 명시적으로 인계하세요");
        }
        if (!previous) {
            publish(client);
        }
        return std::atomic_load(&view_)->revision;
    }
    void release(const std::optional<std::string>& client, bool inactive) {
        const std::scoped_lock lock(mutex_);
        if (owner() != client) {
            throw ValueError("현재 GUI의 제어권이 아닙니다");
        }
        require_inactive(inactive);
        publish(std::nullopt);
    }
    std::optional<std::string> maintenance_claim(const std::string& client) {
        validate(client, "정비 GUI 세션 ID 오류");
        if (client.rfind("main:", 0) == 0) {
            throw ValueError("정비 GUI 세션 ID 오류");
        }
        const std::scoped_lock lock(mutex_);
        require_no_emergency();
        const auto previous = owner();
        if (previous != client) {
            publish(client);
        }
        return previous;
    }
    std::optional<std::string> main_gui_claim(const std::string& client, bool owner_live,
                                              bool follow_active, bool fc_busy,
                                              bool ground_disarmed) {
        validate(client, "메인 GUI 세션 ID 오류");
        if (client.rfind("main:", 0) != 0) {
            throw ValueError("메인 GUI 세션 ID 오류");
        }
        const std::scoped_lock lock(mutex_);
        if (fc_busy) {
            throw ValueError("FC 명령 처리 중: 완료 후 다시 제어권을 가져오세요");
        }
        if (follow_active) {
            throw ValueError("추종·정비 동작 중에는 제어권을 옮기지 않습니다. 먼저 중단하세요");
        }
        if (emergency()) {
            if (!ground_disarmed) {
                throw ValueError("긴급 착륙 잠금: 지상·무장 해제 후 다시 가져오세요");
            }
            set_emergency(false);
        }
        // 2026-11-16 사용자 지시: 다른 메인 GUI가 연결돼 있어도(비행 중 포함) 명시적 "제어권
        // 가져오기"로 가져온다. 추종·FC 명령 처리 중·긴급 잠금 거절은 그대로 둔다. owner_live는
        // 기록용으로만 받는다.
        (void)owner_live;
        const auto previous = owner();
        if (previous != client) {
            publish(client);
        }
        return previous;
    }
    void transfer(const std::string& client, bool inactive) {
        validate(client, "관제 세션 ID 오류");
        const std::scoped_lock lock(mutex_);
        require_no_emergency();
        require_inactive(inactive);
        publish(client);
    }

private:
    struct View {
        std::optional<std::string> owner;
        std::uint64_t revision{0};
    };
    static void validate(const std::string& client, const char* reason) {
        const auto size = utf8_length(client);
        if (size == 0 || size > 80) {
            throw ValueError(reason);
        }
    }
    static void require_inactive(bool inactive) {
        if (!inactive) {
            throw ValueError("실제 추종·명령·외부제어 중단 확인이 필요합니다");
        }
    }
    void require_no_emergency() const {
        if (emergency()) {
            throw ValueError("긴급 착륙 잠금이 설정되어 있습니다");
        }
    }
    void publish(std::optional<std::string> client) {
        const auto revision = std::atomic_load(&view_)->revision + 1;
        std::atomic_store(&view_, std::make_shared<const View>(View{std::move(client), revision}));
    }
    std::recursive_mutex mutex_;
    std::shared_ptr<const View> view_{std::make_shared<const View>()};
    std::atomic<bool> emergency_{false};
};
}  // namespace argus::gateway
