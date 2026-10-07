#pragma once
#include "argus/contracts/Observation.hpp"
#include <msgpack.hpp>
#include <string>
#include <zmq.hpp>
namespace argus {
class MessageBus {
    zmq::socket_t socket_;

  public:
    MessageBus(zmq::context_t &, const std::string &endpoint);
    void publish(const std::string &topic, const std::string &header, const std::string *payload = nullptr);
    void publish_observation(const Observation &);
    void diagnostic(const std::string &source, const std::string &state, const std::string &detail,
                    const ClockStamp &);
};
std::string observation_message(const Observation &);
} // namespace argus
