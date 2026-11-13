// 테스트용 IFdWatcher. poll(2) 로 등록된 file descriptor 를 실제로 기다린다
#ifndef ARGUS_ONBOARD_TEST_POLL_FD_WATCHER_HPP_
#define ARGUS_ONBOARD_TEST_POLL_FD_WATCHER_HPP_

#include <poll.h>

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "argus/core/fd_watcher.hpp"
#include "argus/core/subscription.hpp"

namespace argus::onboard {

class PollFdWatcher : public core::IFdWatcher {
public:
    PollFdWatcher() : entries_(std::make_shared<Entries>()) {}

    [[nodiscard]] core::Subscription add_fd(int fd, Handler on_readable) override {
        const std::uint64_t id = entries_->next_id++;
        entries_->by_id[id] = Entry{fd, std::move(on_readable)};
        const std::weak_ptr<Entries> weak = entries_;
        return core::Subscription([weak, id] {
            if (const auto entries = weak.lock()) {
                entries->by_id.erase(id);
            }
        });
    }

    // 등록된 file descriptor 중 하나에 읽을 데이터가 생길 때까지 최대 wait 동안 기다리고, 읽을
    // 데이터가 있는 file descriptor 의 handler 를 호출한다. 호출한 handler 수를 반환한다
    int wait_and_dispatch(std::chrono::milliseconds wait) {
        std::vector<pollfd> fds;
        std::vector<Handler> handlers;
        for (const auto& [id, entry] : entries_->by_id) {
            fds.push_back(pollfd{entry.fd, POLLIN, 0});
            handlers.push_back(entry.on_readable);
        }
        if (poll(fds.data(), static_cast<nfds_t>(fds.size()), static_cast<int>(wait.count())) <=
            0) {
            return 0;
        }
        int called = 0;
        for (std::size_t index = 0; index < fds.size(); ++index) {
            if ((static_cast<unsigned int>(fds[index].revents) &
                 static_cast<unsigned int>(POLLIN)) != 0U) {
                handlers[index]();
                ++called;
            }
        }
        return called;
    }

    std::size_t registered_count() const { return entries_->by_id.size(); }

private:
    struct Entry {
        int fd = -1;
        Handler on_readable;
    };
    struct Entries {
        std::uint64_t next_id = 1;
        std::map<std::uint64_t, Entry> by_id;
    };

    std::shared_ptr<Entries> entries_;
};

}  // namespace argus::onboard

#endif  // ARGUS_ONBOARD_TEST_POLL_FD_WATCHER_HPP_
