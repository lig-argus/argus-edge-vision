#include "argus/application/Recorder.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>
namespace argus {
Recorder::Recorder(const std::string &path) : output_(path) {
    if (!output_)
        throw std::runtime_error("cannot open latency CSV: " + path);
    output_ << "seq,received_monotonic_ns,published_monotonic_ns,queue_dwell_ms,preprocess_ms,infer_total_ms,"
               "ipc_ms,total_ms,n_detections,queue_dropped\n";
    thread_ = std::thread(&Recorder::run, this);
}
void Recorder::record(const Observation &o, std::uint64_t dropped) {
    check();
    std::ostringstream s;
    s << std::fixed << std::setprecision(6);
    s << o.frame_id << ',' << o.timing.received_monotonic_ns << ',' << o.published_monotonic_ns << ','
      << o.queue_dwell_ms << ',' << o.preprocess_ms << ',' << o.inference_ms << ',' << o.ipc_ms << ','
      << (o.published_monotonic_ns - o.timing.received_monotonic_ns) / 1000000.0 << ',' << o.full_boxes.size()
      << ',' << dropped << '\n';
    std::lock_guard<std::mutex> lock(mutex_);
    if (rows_.size() == 128) {
        rows_.pop_front();
        ++dropped_;
    }
    rows_.push_back(s.str());
    ready_.notify_one();
}
void Recorder::run() {
    try {
        for (;;) {
            std::string row;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                ready_.wait(lock, [&] { return stopping_ || !rows_.empty(); });
                if (rows_.empty() && stopping_)
                    break;
                row = std::move(rows_.front());
                rows_.pop_front();
            }
            output_ << row;
            if (!output_)
                throw std::runtime_error("CSV write failed");
        }
        output_.flush();
        if (!output_)
            throw std::runtime_error("CSV flush failed");
    } catch (...) {
        std::lock_guard<std::mutex> lock(mutex_);
        error_ = std::current_exception();
    }
}
void Recorder::check() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (error_)
        std::rethrow_exception(error_);
}
void Recorder::close() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
        ready_.notify_all();
    }
    if (thread_.joinable())
        thread_.join();
    if (dropped_)
        std::cerr << "Recorder dropped rows=" << dropped_ << '\n';
    check();
}
Recorder::~Recorder() {
    try {
        close();
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
    }
}
} // namespace argus
