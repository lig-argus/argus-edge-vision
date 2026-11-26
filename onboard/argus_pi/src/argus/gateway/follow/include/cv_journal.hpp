// CV·송신부 진단 기록 (Pi tools/real_fc/cv_journal.py CvJournal과 같은 규칙).
// 비동기·유한 큐·최대 8MB. 제어 승인이나 송신을 막지 않는다 (실패해도 기록만 멈춘다).
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "argus/gateway/common/include/msgpack_codec.hpp"

namespace argus::gateway {

class CvJournal {
public:
    // directory가 없으면 기록하지 않는다 (원본 directory=None)
    explicit CvJournal(std::optional<std::filesystem::path> directory,
                       std::size_t queue_size = 512);
    CvJournal(const CvJournal&) = delete;
    CvJournal& operator=(const CvJournal&) = delete;
    CvJournal(CvJournal&&) = delete;
    CvJournal& operator=(CvJournal&&) = delete;
    ~CvJournal();

    // fields는 순서를 지킨 사전. schema·event·queued_monotonic_ns 뒤에 붙는다.
    void record(const std::string& event, const Wire& fields);
    void close();
    std::optional<std::filesystem::path> path() const { return path_; }

private:
    void run();

    std::size_t queue_size_;
    std::optional<std::filesystem::path> path_;
    std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<Wire> pending_;
    std::size_t dropped_{0};
    std::atomic<bool> closed_{false};
    std::thread thread_;
};

}  // namespace argus::gateway
