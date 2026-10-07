#pragma once
#include "argus/contracts/Observation.hpp"
#include <condition_variable>
#include <deque>
#include <exception>
#include <fstream>
#include <mutex>
#include <thread>
namespace argus {
class Recorder {
    std::ofstream output_;
    std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<std::string> rows_;
    std::thread thread_;
    std::exception_ptr error_;
    bool stopping_{};
    std::uint64_t dropped_{};
    void run();

  public:
    explicit Recorder(const std::string &path);
    ~Recorder();
    void record(const Observation &, std::uint64_t queue_dropped);
    void check();
    void close();
};
} // namespace argus
