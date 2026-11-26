// Bounded, non-blocking post-flight decision logging. Never affects control decisions.
#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif
#include "argus/common/serialization/include/native_json_io.hpp"

namespace follow {
class DecisionLog final {
public:
    explicit DecisionLog(std::filesystem::path path) : path_(std::move(path)) {
        try {
            if (!path_.empty()) {
                worker_ = std::thread([this] { run(); });
            }
        } catch (...) {
            path_.clear();
            healthy_.store(false, std::memory_order_relaxed);
            std::cerr << "decision diagnostics unavailable: worker start failed\n";
        }
    }
    ~DecisionLog() {
        if (!worker_.joinable()) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        condition_.notify_one();
        worker_.join();
    }
    DecisionLog(const DecisionLog&) = delete;
    DecisionLog& operator=(const DecisionLog&) = delete;
    bool healthy() const noexcept { return healthy_.load(std::memory_order_relaxed); }
    void note_dropped() noexcept { dropped_.fetch_add(1, std::memory_order_relaxed); }
    void note_critical_dropped(std::uint64_t count) noexcept {
        dropped_.fetch_add(count, std::memory_order_relaxed);
        dropped_critical_.fetch_add(count, std::memory_order_relaxed);
    }

    bool try_record(std::function<NativeJson()> make_record, bool flush_now = false,
                    bool count_on_reject = true) noexcept {
        if (path_.empty()) {
            return false;
        }
        if (!healthy_.load(std::memory_order_relaxed)) {
            if (count_on_reject) {
                dropped_.fetch_add(1, std::memory_order_relaxed);
                if (flush_now) {
                    dropped_critical_.fetch_add(1, std::memory_order_relaxed);
                }
            }
            return false;
        }
        try {
            std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
            if (!lock.owns_lock()) {
                if (count_on_reject) {
                    dropped_.fetch_add(1, std::memory_order_relaxed);
                    if (flush_now) {
                        dropped_critical_.fetch_add(1, std::memory_order_relaxed);
                    }
                }
                return false;
            }
            auto& queue = flush_now ? critical_queue_ : queue_;
            if (queue.size() >= (flush_now ? kCriticalQueueLimit : kQueueLimit)) {
                if (count_on_reject) {
                    dropped_.fetch_add(1, std::memory_order_relaxed);
                    if (flush_now) {
                        dropped_critical_.fetch_add(1, std::memory_order_relaxed);
                    }
                }
                return false;
            }
            queue.push_back({std::move(make_record), flush_now});
            lock.unlock();
            condition_.notify_one();
            return true;
        } catch (...) {
            if (count_on_reject) {
                dropped_.fetch_add(1, std::memory_order_relaxed);
                if (flush_now) {
                    dropped_critical_.fetch_add(1, std::memory_order_relaxed);
                }
            }
            return false;
        }
    }

private:
    static constexpr std::size_t kQueueLimit = 256;
    static constexpr std::size_t kCriticalQueueLimit = 32;
    static constexpr std::uintmax_t kMaxFileBytes = 8 * 1024 * 1024;
    static constexpr int kBackups = 3;
    std::filesystem::path path_;
    std::mutex mutex_;
    std::condition_variable condition_;
    struct Entry {
        std::function<NativeJson()> make_record;
        bool flush_now{false};
    };
    std::deque<Entry> queue_;
    std::deque<Entry> critical_queue_;
    std::atomic<std::uint64_t> dropped_{0};
    std::atomic<std::uint64_t> dropped_critical_{0};
    std::atomic<bool> healthy_{true};
    bool stopping_{false};
    std::thread worker_;

    void run() noexcept {
        try {
            std::ofstream file;
            std::uintmax_t bytes = 0;
            const auto open = [&] {
                std::error_code ec;
                bytes =
                    std::filesystem::exists(path_, ec) ? std::filesystem::file_size(path_, ec) : 0;
                if (ec) {
                    bytes = 0;
                }
                file.open(path_, std::ios::out | std::ios::app | std::ios::binary);
                return file.is_open();
            };
            if (!open()) {
                healthy_.store(false, std::memory_order_relaxed);
                std::cerr << "decision diagnostics unavailable: cannot open output\n";
                return;
            }
            while (true) {
                Entry entry;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    condition_.wait(lock, [&] {
                        return stopping_ || !critical_queue_.empty() || !queue_.empty();
                    });
                    if (queue_.empty() && critical_queue_.empty() && stopping_) {
                        break;
                    }
                    auto& queue = critical_queue_.empty() ? queue_ : critical_queue_;
                    entry = std::move(queue.front());
                    queue.pop_front();
                }
                NativeJson record;
                try {
                    record = entry.make_record();
                } catch (...) {
                    dropped_.fetch_add(1, std::memory_order_relaxed);
                    if (entry.flush_now) {
                        dropped_critical_.fetch_add(1, std::memory_order_relaxed);
                    }
                    continue;
                }
                record["dropped_since_previous_record"] =
                    dropped_.exchange(0, std::memory_order_relaxed);
                record["dropped_critical_since_previous_record"] =
                    dropped_critical_.exchange(0, std::memory_order_relaxed);
                const auto line = record.dump() + "\n";
                if (bytes + line.size() + 256 > kMaxFileBytes) {
                    file.close();
                    std::error_code ec;
                    std::filesystem::remove(path_.string() + "." + std::to_string(kBackups), ec);
                    for (int i = kBackups - 1; i >= 1; --i) {
                        const auto from = path_.string() + "." + std::to_string(i);
                        const auto to = path_.string() + "." + std::to_string(i + 1);
                        ec.clear();
                        if (std::filesystem::exists(from, ec)) {
                            ec.clear();
                            std::filesystem::rename(from, to, ec);
                            if (ec) {
                                throw std::runtime_error("decision diagnostics rotate failed");
                            }
                        }
                    }
                    ec.clear();
                    std::filesystem::rename(path_, path_.string() + ".1", ec);
                    if (ec || !open()) {
                        throw std::runtime_error("decision diagnostics rotate failed");
                    }
                }
                file << line;
                if (!file) {
                    throw std::runtime_error("decision diagnostics write failed");
                }
                bytes += line.size();
                const bool periodic_flush = queue_flush_counter_++ % 20 == 0;
                if (entry.flush_now || periodic_flush) {
                    file.flush();
                    if (!file) {
                        throw std::runtime_error("decision diagnostics flush failed");
                    }
                    if (entry.flush_now && !sync_file(path_)) {
                        throw std::runtime_error("decision diagnostics durable sync failed");
                    }
                }
            }
            const auto dropped = dropped_.exchange(0, std::memory_order_relaxed);
            const auto critical_dropped = dropped_critical_.exchange(0, std::memory_order_relaxed);
            if (dropped || critical_dropped) {
                file << NativeJson{{"schema", "argus.follow.decision.loss.v1"},
                                   {"dropped_since_previous_record", dropped},
                                   {"dropped_critical_since_previous_record", critical_dropped}}
                            .dump()
                     << '\n';
            }
            file.flush();
            if (!file) {
                throw std::runtime_error("decision diagnostics final flush failed");
            }
        } catch (const std::exception& e) {
            healthy_.store(false, std::memory_order_relaxed);
            std::cerr << "decision diagnostics unavailable: " << e.what() << '\n';
        } catch (...) {
            healthy_.store(false, std::memory_order_relaxed);
            std::cerr << "decision diagnostics unavailable\n";
        }
    }
    // Runs only on the diagnostic worker. Never waits for storage on the control thread.
    static bool sync_file(const std::filesystem::path& path) noexcept {
#if defined(_WIN32)
        HANDLE handle = CreateFileW(path.c_str(), GENERIC_WRITE,
                                    FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                                    OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (handle == INVALID_HANDLE_VALUE) {
            return false;
        }
        const bool ok = FlushFileBuffers(handle) != 0;
        CloseHandle(handle);
        return ok;
#else
        const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
        if (fd < 0) {
            return false;
        }
        const bool ok = ::fsync(fd) == 0;
        ::close(fd);
        return ok;
#endif
    }
    std::uint64_t queue_flush_counter_{0};
};
}  // namespace follow
