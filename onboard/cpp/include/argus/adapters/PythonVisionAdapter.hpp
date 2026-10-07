#pragma once
#include "argus/adapters/MessageBus.hpp"
#include "argus/adapters/OwnedProcess.hpp"
#include "argus/application/EnvironmentProfile.hpp"
#include "argus/ports/IDetector.hpp"
#include <atomic>
#include <mutex>
#include <thread>
namespace argus {
class PythonVisionAdapter final : public IDetector {
    zmq::context_t &context_;
    zmq::socket_t request_;
    OwnedProcess process_;
    std::string directory_, session_, source_, publish_endpoint_;
    unsigned timeout_ms_;
    std::atomic<bool> stopping_{};
    std::thread preview_relay_;
    std::exception_ptr relay_error_;
    std::mutex relay_mutex_;
    msgpack::object_handle exchange(const std::string &header, const Frame *frame, unsigned timeout);
    void relay();

  public:
    PythonVisionAdapter(zmq::context_t &, const RuntimeOptions &, const std::string &source,
                        const std::string &session);
    ~PythonVisionAdapter();
    DetectionResult infer(const Frame &) override;
    void stop() noexcept;
};
} // namespace argus
