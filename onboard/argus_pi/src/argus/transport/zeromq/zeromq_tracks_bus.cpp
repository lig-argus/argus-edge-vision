// ZeroMQ socket, multipart, protobuf, MessagePack 오류를 추적 도메인 밖으로 내보내지 않는다.
// 2026-11-25 batch 2: tracking/tracks 는 팀 규약 [이름][protobuf] (6375). 받기는 team_wire
// (이름 정확 일치, 프레임 수, 크기 상한), 본문은 tracks_body 로 Pi 값을 되살린 뒤 예전과 같은
// TracksV1Codec::decode 로 검사한다. 예전 msgpack PUB(ZeroMqTracksPublisher)은 쓰는 곳이 없어
// 지웠다.
#include "argus/transport/zeromq/include/zeromq_tracks_bus.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>
#include <zmq.hpp>

#include "argus/tracking/contracts/include/tracks_v1_codec.hpp"
#include "argus/transport/team_wire/include/team_wire.hpp"
#include "argus/transport/team_wire/include/tracks_body.hpp"
#include "argus/transport/zeromq/include/zeromq_operations.hpp"

namespace follow {

ZeroMqTracksSubscriber::ZeroMqTracksSubscriber(ZeroMqContext& context, const std::string& address,
                                               BusSocketEndpoint action, int receive_hwm) {
    if (receive_hwm < 1 || address.empty()) {
        throw std::invalid_argument("invalid tracks subscriber config");
    }
    auto socket = zmq_open_socket(context, ZMQ_SUB, "tracks SUB socket");
    const int linger = 0;
    zmq_set_option(socket.get(), ZMQ_LINGER, &linger, sizeof(linger), "tracks SUB linger");
    zmq_set_option(socket.get(), ZMQ_RCVHWM, &receive_hwm, sizeof(receive_hwm), "tracks SUB HWM");
    // 2026-11-25 batch 6c: ZMQ_MAXMSGSIZE 없음 (libzmq 4.3.5 는 넘는 프레임을 보낸 연결을 끊고
    // 다시 맺지 않는다). 본문 상한 1 MiB 는 receive 의 receive_raw (TooLarge -> Invalid) 가 본다
    zmq_set_option(socket.get(), ZMQ_SUBSCRIBE, kTracksTopic, sizeof(kTracksTopic) - 1,
                   "tracks SUB topic");
    zmq_apply_endpoint(socket.get(), address, action, "tracks endpoint");
    socket_ = socket.release();
}
ZeroMqTracksSubscriber::~ZeroMqTracksSubscriber() {
    if (socket_) {
        zmq_close(socket_);
    }
}

TracksBusReceive ZeroMqTracksSubscriber::receive(std::chrono::milliseconds timeout) {
    TracksBusReceive result;
    zmq_pollitem_t item{socket_, 0, ZMQ_POLLIN, 0};
    const auto ms = std::clamp<std::int64_t>(timeout.count(), 0, 60000);
    if (zmq_poll(&item, 1, static_cast<long>(ms)) < 0) {
        if (zmq_errno() == EINTR) {
            return result;
        }
        result.state = TracksBusReceive::State::TransportError;
        result.error = zmq_error("tracks poll");
        return result;
    }
    if (!(item.revents & ZMQ_POLLIN)) {
        return result;
    }
    argus::team_wire::RawMessage raw;
    const std::vector<std::string_view> accepted{argus::team_wire::kTrackingTracks};
    const auto received =
        argus::team_wire::receive_raw(zmq::socket_ref(zmq::from_handle, socket_), accepted, raw);
    if (received == argus::team_wire::ReceiveStatus::NoMessage) {
        return result;
    }
    if (received == argus::team_wire::ReceiveStatus::SocketError) {
        result.state = TracksBusReceive::State::TransportError;
        result.error = zmq_error("tracks message receive");
        return result;
    }
    if (received != argus::team_wire::ReceiveStatus::Ok) {
        result.state = TracksBusReceive::State::Invalid;
        result.error = std::string("unexpected tracks topic, multipart shape or size: ") +
                       argus::team_wire::to_string(received);
        return result;
    }
    std::vector<std::uint8_t> packed;
    std::string error;
    if (argus::team_wire::tracks_body_to_msgpack(raw.body.data(), raw.body.size(), packed, error) !=
        argus::team_wire::TracksBodyStatus::Ok) {
        result.state = TracksBusReceive::State::Invalid;
        result.error = std::move(error);
        return result;
    }
    auto decoded = TracksV1Codec::decode(
        std::string_view(reinterpret_cast<const char*>(packed.data()), packed.size()));
    if (!decoded.valid) {
        result.state = TracksBusReceive::State::Invalid;
        result.error = std::move(decoded.error);
        return result;
    }
    result.state = TracksBusReceive::State::Message;
    result.message = std::move(decoded.message);
    return result;
}
}  // namespace follow
