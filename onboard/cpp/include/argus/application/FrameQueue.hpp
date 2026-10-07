#pragma once
#include "argus/contracts/Frame.hpp"
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
namespace argus {
class FrameQueue {
    std::size_t capacity_;
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<Frame> frames_;
    bool closed_{};
    std::uint64_t dropped_{};

  public:
    explicit FrameQueue(std::size_t capacity = 2) : capacity_(capacity) {
        if (!capacity)
            throw std::invalid_argument("frame queue capacity must be positive");
    }
    void put(Frame frame) {
        std::lock_guard<std::mutex> guard(mutex_);
        if (closed_)
            return;
        if (frames_.size() == capacity_) {
            frames_.pop_front();
            ++dropped_;
        }
        frames_.push_back(std::move(frame));
        ready_.notify_one();
    }
    bool take(Frame &frame, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> guard(mutex_);
        ready_.wait_for(guard, timeout, [&] { return closed_ || !frames_.empty(); });
        if (frames_.empty())
            return false;
        frame = std::move(frames_.front());
        frames_.pop_front();
        return true;
    }
    void close() {
        std::lock_guard<std::mutex> guard(mutex_);
        closed_ = true;
        ready_.notify_all();
    }
    bool drained() const {
        std::lock_guard<std::mutex> guard(mutex_);
        return closed_ && frames_.empty();
    }
    std::uint64_t dropped() const {
        std::lock_guard<std::mutex> guard(mutex_);
        return dropped_;
    }
};
} // namespace argus
