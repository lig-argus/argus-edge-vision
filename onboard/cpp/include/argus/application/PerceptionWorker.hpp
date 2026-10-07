#pragma once
#include "argus/adapters/MessageBus.hpp"
#include "argus/application/CompositionRoot.hpp"
#include <csignal>
namespace argus {
class PerceptionWorker {
    CompositionRoot &root_;
    const IClock &clock_;
    zmq::context_t &context_;
    MessageBus &bus_;
    std::string session_;

  public:
    PerceptionWorker(CompositionRoot &root, const IClock &clock, zmq::context_t &context, MessageBus &bus,
                     std::string session)
        : root_(root), clock_(clock), context_(context), bus_(bus), session_(std::move(session)) {}
    void run(const volatile std::sig_atomic_t &interrupted);
};
} // namespace argus
