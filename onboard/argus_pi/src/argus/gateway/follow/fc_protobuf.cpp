// 게이트웨이의 FC 텔레메트리 6370 (2026-11-25 batch 6a). 규칙은 include/fc_protobuf.hpp.
#include "argus/gateway/follow/include/fc_protobuf.hpp"

#include <zmq.h>

#include <condition_variable>
#include <deque>
#include <exception>
#include <iostream>
#include <map>
#include <mutex>
#include <string_view>
#include <thread>
#include <utility>
#include <zmq.hpp>

#include "argus/transport/team_wire/include/pi_topics.hpp"
#include "argus/transport/team_wire/include/team_wire.hpp"

namespace argus::gateway {

namespace {

bool fc_topic(std::string_view name) {
    return name == argus::team_wire::kFcState || name == argus::team_wire::kFcAttitude ||
           name == argus::team_wire::kFcModeEvent;
}

std::string_view text(const std::vector<std::uint8_t>& bytes) {
    return {reinterpret_cast<const char*>(bytes.data()), bytes.size()};
}

bool power_of_two(std::uint64_t count) { return count != 0 && (count & (count - 1)) == 0; }

struct Item {
    std::string name;
    std::uint64_t sequence = 0;
    std::vector<std::uint8_t> packed;
};

}  // namespace

struct FcTelemetryWire::Impl {
    mutable std::mutex mutex;
    std::condition_variable wake;
    std::deque<Item> queue;
    std::size_t queue_depth = kQueueDepth;
    std::map<std::string, std::uint64_t, std::less<>> sequences;  // 팀 seq (publish 쪽)
    FcWireStats stats;
    std::string send_error;  // 송신 스레드의 zmq_send 실패 (다음 publish 가 한 번 알린다)
    bool started = false;
    bool stopping = false;
    void* socket = nullptr;  // start 뒤로 송신 스레드만 쓴다
    argus::team_wire::UnknownKeyHook on_unknown = argus::team_wire::default_unknown_key_hook();
    std::thread sender;

    void run() {
        std::unique_lock<std::mutex> lock(mutex);
        for (;;) {
            wake.wait(lock, [this] { return stopping || !queue.empty(); });
            if (queue.empty()) {
                return;  // stopping 이고 대기열을 모두 보냈다
            }
            Item item = std::move(queue.front());
            queue.pop_front();
            lock.unlock();
            std::string error;
            const bool encoded = send_one(item, error);
            lock.lock();
            if (!encoded) {
                ++stats.encode_failures;
                if (power_of_two(stats.encode_failures)) {
                    std::cerr << "fc 6370 변환 실패 (누적 " << stats.encode_failures
                              << "): " << error << '\n';
                }
            } else if (!error.empty()) {
                ++stats.send_failures;
                send_error = error;
            } else {
                ++stats.sent;
            }
        }
    }

    // 변환하지 못하면 false (error 에 까닭). 변환은 했으나 보내지 못하면 true 이고 error 에 까닭
    bool send_one(const Item& item, std::string& error) const {
        follow::NativeJson value;
        try {
            value = follow::NativeJson::from_msgpack(item.packed.begin(), item.packed.end());
        } catch (const std::exception& failure) {
            error = item.name + ": msgpack 해석 실패: " + failure.what();
            return false;
        }
        const auto* spec = argus::team_wire::find_wire_spec(item.name);
        std::vector<std::uint8_t> body;
        std::vector<std::string> unknown;
        if (spec == nullptr || !argus::team_wire::encode_body(
                                   item.name, false, value, item.sequence, body, unknown, error)) {
            error = item.name + ": " + error;
            return false;
        }
        if (body.size() > spec->max_body_bytes) {
            error = item.name + ": 본문 상한 초과";
            return false;
        }
        if (!unknown.empty() && on_unknown) {
            on_unknown(item.name, unknown);
        }
        if (!argus::team_wire::send_raw(zmq::socket_ref(zmq::from_handle, socket), item.name, body,
                                        nullptr)) {
            error = item.name + ": 보내기 실패: " + zmq_strerror(zmq_errno());
        }
        return true;
    }
};

FcTelemetryWire::FcTelemetryWire() : impl_(std::make_unique<Impl>()) {}

FcTelemetryWire::~FcTelemetryWire() { stop(); }

void FcTelemetryWire::start(void* socket, std::size_t queue_depth) {
    const std::scoped_lock lock(impl_->mutex);
    if (impl_->started) {
        return;
    }
    impl_->socket = socket;
    impl_->queue_depth = queue_depth == 0 ? 1 : queue_depth;
    impl_->started = true;
    impl_->sender = std::thread([this] { impl_->run(); });
}

FcPublish FcTelemetryWire::publish(const std::vector<std::uint8_t>& topic,
                                   const std::vector<std::uint8_t>& packed, std::string& error) {
    error.clear();
    const std::string_view name = text(topic);
    if (!fc_topic(name)) {
        error = "6370 토픽이 아님: " + std::string(name);
        return FcPublish::NotFcTopic;
    }
    {
        const std::scoped_lock lock(impl_->mutex);
        if (!impl_->started || impl_->stopping) {
            error = std::string(name) + ": 송신 스레드가 없음";
            return FcPublish::SendFailed;
        }
        if (!impl_->send_error.empty()) {
            error = std::move(impl_->send_error);
            impl_->send_error.clear();
            return FcPublish::SendFailed;
        }
        auto found = impl_->sequences.find(name);
        if (found == impl_->sequences.end()) {
            found = impl_->sequences.emplace(std::string(name), 0).first;
        }
        const std::uint64_t sequence = ++found->second;  // 보내지 못해도 번호를 쓴다 (팀 ZmqBus)
        if (impl_->queue.size() >= impl_->queue_depth) {
            ++impl_->stats.queue_drops;
            error = std::string(name) + ": 송신 대기열 가득 참 (누적 " +
                    std::to_string(impl_->stats.queue_drops) + ")";
            return FcPublish::QueueFull;
        }
        impl_->queue.push_back(Item{std::string(name), sequence, packed});
        ++impl_->stats.queued;
    }
    impl_->wake.notify_one();
    return FcPublish::Queued;
}

void FcTelemetryWire::stop() {
    {
        const std::scoped_lock lock(impl_->mutex);
        if (!impl_->started || impl_->stopping) {
            return;
        }
        impl_->stopping = true;
    }
    impl_->wake.notify_one();
    if (impl_->sender.joinable()) {
        impl_->sender.join();
    }
}

FcWireStats FcTelemetryWire::stats() const {
    const std::scoped_lock lock(impl_->mutex);
    return impl_->stats;
}

std::uint64_t FcTelemetryWire::last_sequence(const std::string& topic) const {
    const std::scoped_lock lock(impl_->mutex);
    const auto found = impl_->sequences.find(topic);
    return found == impl_->sequences.end() ? 0 : found->second;
}

FcStateFrame fc_state_msgpack(const std::vector<std::vector<std::uint8_t>>& parts,
                              std::vector<std::uint8_t>& packed, std::string& error) {
    packed.clear();
    error.clear();
    if (parts.size() != 2 || text(parts.front()) != argus::team_wire::kFcState) {
        return FcStateFrame::NotFcState;
    }
    const auto& body = parts.back();
    const auto* spec = argus::team_wire::find_wire_spec(argus::team_wire::kFcState);
    if (spec == nullptr || body.size() > spec->max_body_bytes) {
        error = "fc/state 본문 상한 초과";
        return FcStateFrame::Invalid;
    }
    follow::NativeJson value;
    std::uint64_t sequence = 0;
    if (!argus::team_wire::decode_body(argus::team_wire::kFcState, false, body, value, sequence,
                                       error)) {
        return FcStateFrame::Invalid;
    }
    try {
        packed = follow::NativeJson::to_msgpack(value);
    } catch (const std::exception& failure) {
        error = std::string("fc/state msgpack: ") + failure.what();
        return FcStateFrame::Invalid;
    }
    return FcStateFrame::Ok;
}

}  // namespace argus::gateway
