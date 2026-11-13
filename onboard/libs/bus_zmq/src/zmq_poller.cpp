#include "argus/bus_zmq/zmq_poller.hpp"

#include <algorithm>
#include <cerrno>
#include <limits>
#include <string>
#include <utility>

#include "detail.hpp"

namespace argus::bus_zmq {
namespace {

using detail::kNsPerMs;
using detail::log_error;

}  // namespace

ZmqPoller::ZmqPoller(const core::IClock& clock)
    : clock_(clock), registry_(std::make_shared<Registry>()) {}

core::Subscription ZmqPoller::add_socket(zmq::socket_ref socket, Handler handler) {
    if (!socket || !handler) {
        log_error("ZmqPoller::add_socket: 빈 소켓 또는 빈 handler");
        return {};
    }
    Entry entry;
    entry.socket = socket.handle();
    entry.handler = std::move(handler);
    return add(std::move(entry));
}

core::Subscription ZmqPoller::add_fd(int fd, Handler handler) {
    if (fd < 0 || !handler) {
        log_error("ZmqPoller::add_fd: 음수 fd 또는 빈 handler");
        return {};
    }
    Entry entry;
    entry.fd = fd;
    entry.handler = std::move(handler);
    return add(std::move(entry));
}

core::Subscription ZmqPoller::add(Entry entry) {
    const std::uint64_t id = registry_->next_id++;
    registry_->by_id.emplace(id, std::move(entry));
    registry_->changed = true;
    std::weak_ptr<Registry> weak = registry_;
    return core::Subscription([weak, id] {
        if (const auto registry = weak.lock()) {
            registry->by_id.erase(id);
            registry->changed = true;
        }
    });
}

core::Subscription ZmqPoller::add_timer_at(std::int64_t deadline_ns, Handler handler) {
    if (!handler) {
        log_error("ZmqPoller::add_timer_at: 빈 handler");
        return {};
    }
    const std::uint64_t id = registry_->next_id++;
    registry_->timers_by_id.emplace(id, Timer{deadline_ns, std::move(handler)});
    std::weak_ptr<Registry> weak = registry_;
    return core::Subscription([weak, id] {
        if (const auto registry = weak.lock()) {
            registry->timers_by_id.erase(id);
        }
    });
}

void ZmqPoller::rebuild_poll_items() {
    poll_items_.clear();
    poll_item_ids_.clear();
    for (const auto& [id, entry] : registry_->by_id) {
        zmq::pollitem_t item{};
        item.socket = entry.socket;
        item.fd = entry.fd;
        item.events = ZMQ_POLLIN;
        poll_items_.push_back(item);
        poll_item_ids_.push_back(id);
    }
    registry_->changed = false;
}

std::chrono::milliseconds ZmqPoller::limit_to_next_timer(std::chrono::milliseconds wait) const {
    if (registry_->timers_by_id.empty()) {
        return wait;
    }
    std::int64_t earliest_ns = std::numeric_limits<std::int64_t>::max();
    for (const auto& [id, timer] : registry_->timers_by_id) {
        earliest_ns = std::min(earliest_ns, timer.deadline_ns);
    }
    const std::int64_t remaining_ns = earliest_ns - clock_.now_monotonic_ns();
    if (remaining_ns <= 0) {
        return std::chrono::milliseconds{0};
    }
    // 올림한다. 내리면 시각 직전에 깨어나 타이머를 부르지 못하고 한 번 더 돌게 된다
    const std::chrono::milliseconds remaining{(remaining_ns + kNsPerMs - 1) / kNsPerMs};
    return std::min(wait, remaining);
}

void ZmqPoller::run_due_timers() {
    const std::int64_t now_ns = clock_.now_monotonic_ns();
    std::vector<std::pair<std::int64_t, std::uint64_t>> due;  // (시각, 등록 번호)
    for (const auto& [id, timer] : registry_->timers_by_id) {
        if (timer.deadline_ns <= now_ns) {
            due.emplace_back(timer.deadline_ns, id);
        }
    }
    std::sort(due.begin(), due.end());
    for (const auto& [deadline_ns, id] : due) {
        // 앞 handler 가 이 타이머를 해제했을 수 있다
        const auto found = registry_->timers_by_id.find(id);
        if (found == registry_->timers_by_id.end()) {
            continue;
        }
        // 한 번만 부르도록 먼저 지우고 부른다
        const Handler handler = std::move(found->second.handler);
        registry_->timers_by_id.erase(found);
        handler();
    }
}

void ZmqPoller::process_events(std::chrono::milliseconds wait, std::chrono::milliseconds budget) {
    dispatch_events(limit_to_next_timer(std::max(wait, std::chrono::milliseconds{0})), budget);
    run_due_timers();
}

void ZmqPoller::dispatch_events(std::chrono::milliseconds wait, std::chrono::milliseconds budget) {
    const std::int64_t budget_ns = std::max<std::int64_t>(budget.count(), 0) * kNsPerMs;
    std::chrono::milliseconds timeout = wait;
    std::int64_t started_ns = 0;
    bool called = false;

    for (;;) {
        // handler 가 등록이나 해제를 했으면 다음 poll 전에 목록을 다시 만든다
        if (registry_->changed) {
            rebuild_poll_items();
        }

        int ready_count = 0;
        try {
            ready_count = zmq::poll(poll_items_.data(), poll_items_.size(), timeout);
        } catch (const zmq::error_t& error) {
            // 신호(SIGINT, SIGTERM)로 깨어난 경우는 오류가 아니다. EventLoop 가 정지 여부를 본다
            if (error.num() != EINTR) {
                log_error(std::string("ZmqPoller: zmq_poll 실패: ") + error.what());
            }
            return;
        }
        if (ready_count <= 0) {
            return;
        }
        if (!called) {
            started_ns = clock_.now_monotonic_ns();  // 기다린 시간은 budget 에 넣지 않는다
        }

        ready_ids_.clear();
        for (std::size_t i = 0; i < poll_items_.size(); ++i) {
            if (poll_items_[i].revents != 0) {
                ready_ids_.push_back(poll_item_ids_[i]);
            }
        }
        // 등록 번호 순서인 목록을 마지막으로 부른 번호의 다음부터 시작하도록 돌린다
        std::rotate(ready_ids_.begin(),
                    std::upper_bound(ready_ids_.begin(), ready_ids_.end(), last_served_id_),
                    ready_ids_.end());

        for (const std::uint64_t id : ready_ids_) {
            if (called && clock_.now_monotonic_ns() - started_ns >= budget_ns) {
                return;
            }
            // 앞 handler 가 이 등록을 해제했을 수 있다
            const auto found = registry_->by_id.find(id);
            if (found == registry_->by_id.end()) {
                continue;
            }
            // handler 가 자기 등록을 해제해도 안전하도록 복사해서 부른다
            const Handler handler = found->second.handler;
            last_served_id_ = id;
            called = true;
            handler();
        }

        timeout = std::chrono::milliseconds{0};  // 남은 이벤트만 확인하고 더 기다리지 않는다
    }
}

}  // namespace argus::bus_zmq
