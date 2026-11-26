// tracking/tracks 토픽의 ZeroMQ SUB 경계를 정의한다.
// 2026-11-25 batch 2: 팀 규약 [이름][protobuf] (zeromq_tracks_bus.cpp). 예전 msgpack PUB 은 지웠다.
#pragma once

#include <chrono>
#include <string>

#include "argus/tracking/contracts/include/tracks_v1.hpp"
#include "argus/transport/zeromq/include/zeromq_observation_bus.hpp"

namespace follow {

struct TracksBusReceive {
    enum class State { Message, Timeout, Invalid, TransportError } state{State::Timeout};
    TracksV1 message;
    std::string error;
};

class ZeroMqTracksSubscriber final {
public:
    ZeroMqTracksSubscriber(ZeroMqContext&, const std::string& endpoint,
                           BusSocketEndpoint action = BusSocketEndpoint::Connect,
                           int receive_hwm = 2);
    ~ZeroMqTracksSubscriber();
    ZeroMqTracksSubscriber(const ZeroMqTracksSubscriber&) = delete;
    ZeroMqTracksSubscriber& operator=(const ZeroMqTracksSubscriber&) = delete;
    TracksBusReceive receive(std::chrono::milliseconds timeout);

private:
    void* socket_{nullptr};
};

inline constexpr char kTracksTopic[] = "tracking/tracks";

}  // namespace follow
