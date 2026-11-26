// 버스 wait는 이 worker에만 둔다. timeout마다 제어 측이 호출할 수 있도록 바로 반환한다.
#include "argus/transport/zeromq/include/observation_bus_worker.hpp"

#include <exception>
#include <stdexcept>
#include <utility>

namespace follow {

ObservationBusWorker::ObservationBusWorker(ZeroMqContext& context, ObservationInbox& inbox,
                                           std::string endpoint,
                                           std::chrono::milliseconds poll_interval)
    : context_(context),
      inbox_(inbox),
      endpoint_(std::move(endpoint)),
      poll_interval_(poll_interval) {
    if (endpoint_.empty() || poll_interval_.count() <= 0 ||
        poll_interval_ > std::chrono::seconds(1)) {
        throw std::invalid_argument("invalid observation subscription worker configuration");
    }
}

ObservationBusWorker::~ObservationBusWorker() { stop(); }

void ObservationBusWorker::start() {
    std::promise<void> initialized;
    auto ready = initialized.get_future();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (thread_.joinable() || stats_.running) {
            throw std::logic_error("observation bus worker already started");
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

void ObservationBusWorker::stop() {
    stop_requested_.store(true);
    if (thread_.joinable()) {
        thread_.join();
    }
}

ObservationBusWorkerStats ObservationBusWorker::stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

void ObservationBusWorker::run(std::promise<void> initialized) {
    try {
        // SUB socket은 run thread에서 만들고 닫아 ZeroMQ 소유 스레드 규칙을 지킨다.
        ZeroMqObservationSubscriber subscriber(context_, endpoint_);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stats_.running = true;
        }
        initialized.set_value();
        while (!stop_requested_.load()) {
            const auto result = subscriber.receive(poll_interval_);
            std::lock_guard<std::mutex> lock(mutex_);
            if (result.state == ObservationBusReceive::State::Timeout) {
                ++stats_.empty_polls;
                continue;
            }
            if (result.state == ObservationBusReceive::State::TransportError) {
                ++stats_.transport_errors;
                stats_.last_error = result.error;
                continue;
            }
            if (result.state == ObservationBusReceive::State::Invalid) {
                ++stats_.malformed_messages;
                stats_.last_error = result.error;
                continue;
            }
            ++stats_.messages_received;
            // local receive time만으로 age를 잰다. producer monotonic 값은 노출/제어 연령으로
            // 복사하지 않는다.
            inbox_.offer(result.message, Clock::now());
        }
    } catch (const std::exception& error) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.transport_errors;
        stats_.last_error = error.what();
        if (!stats_.running) {
            initialized.set_exception(std::current_exception());
        }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.running = false;
}

}  // namespace follow
