// tracks bus 대기·디코딩을 독립 스레드에 두고 selector에는 검증된 표본만 전달한다.
#include "argus/transport/zeromq/include/target_tracks_worker.hpp"

#include <exception>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <utility>

namespace follow {

TargetTracksWorker::TargetTracksWorker(ZeroMqContext& context, TargetSelector& selector,
                                       std::string endpoint,
                                       std::chrono::milliseconds poll_interval,
                                       BusSocketEndpoint endpoint_action,
                                       std::string diagnostics_csv, UpdateCallback update)
    : context_(context),
      selector_(selector),
      endpoint_(std::move(endpoint)),
      poll_interval_(poll_interval),
      endpoint_action_(endpoint_action),
      diagnostics_csv_(std::move(diagnostics_csv)),
      update_(std::move(update)) {
    if (endpoint_.empty() || poll_interval_.count() <= 0 ||
        poll_interval_ > std::chrono::seconds(1)) {
        throw std::invalid_argument("invalid tracks subscriber worker configuration");
    }
}
TargetTracksWorker::~TargetTracksWorker() { stop(); }
void TargetTracksWorker::start() {
    std::promise<void> initialized;
    auto ready = initialized.get_future();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (thread_.joinable() || stats_.running) {
            throw std::logic_error("tracks worker already started");
        }
        stop_requested_.store(false);
        thread_ = std::thread([this, initialized = std::move(initialized)]() mutable {
            run(std::move(initialized));
        });
    }
    try {
        ready.get();
    } catch (...) {
        stop();
        throw;
    }
}
void TargetTracksWorker::stop() {
    stop_requested_.store(true);
    if (thread_.joinable()) {
        thread_.join();
    }
}
TargetTracksWorkerStats TargetTracksWorker::stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}
void TargetTracksWorker::run(std::promise<void> initialized) {
    try {
        // SUB socket의 생성·수신·종료를 한 worker 스레드가 소유한다.
        ZeroMqTracksSubscriber subscriber(context_, endpoint_, endpoint_action_);
        std::ofstream timing;
        if (!diagnostics_csv_.empty()) {
            timing.open(diagnostics_csv_);
            if (!timing) {
                throw std::runtime_error("cannot create target ingestion timing CSV");
            }
            timing << "sample_ns,frame_id,accepted,target_ingestion_ms\n" << std::setprecision(12);
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stats_.running = true;
        }
        initialized.set_value();
        while (!stop_requested_.load()) {
            const auto received = subscriber.receive(poll_interval_);
            std::lock_guard<std::mutex> lock(mutex_);
            if (received.state == TracksBusReceive::State::Timeout) {
                ++stats_.empty_polls;
                continue;
            }
            if (received.state == TracksBusReceive::State::TransportError ||
                received.state == TracksBusReceive::State::Invalid) {
                ++stats_.invalid_messages;
                stats_.last_error = received.error;
                continue;
            }
            ++stats_.messages_received;
            const auto begin = Clock::now();
            const bool accepted = update_ ? update_(received.message, begin)
                                          : selector_.update(received.message, begin);
            const auto done = Clock::now();
            if (!accepted) {
                ++stats_.rejected_updates;
            }
            if (timing.is_open()) {
                timing << received.message.sample_monotonic_ns << "," << received.message.frame_id
                       << "," << accepted << ","
                       << std::chrono::duration<double, std::milli>(done - begin).count() << "\n";
                timing.flush();
            }
        }
    } catch (const std::exception& error) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.invalid_messages;
        stats_.last_error = error.what();
        if (!stats_.running) {
            initialized.set_exception(std::current_exception());
        }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.running = false;
}

}  // namespace follow
