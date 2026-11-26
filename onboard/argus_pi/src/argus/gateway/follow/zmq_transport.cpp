#include "argus/gateway/follow/include/zmq_transport.hpp"

#include <zmq.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "argus/gateway/follow/include/control_protobuf.hpp"
#include "argus/gateway/follow/include/mode_rpc_protobuf.hpp"

namespace argus::gateway {

namespace {

// 2026-11-25 batch 6c: 6375 추적 메시지 부분 하나의 상한 (team_wire kMaxHeaderBytes, 예전
// MAXMSGSIZE 1 MiB). 받은 뒤 보고 넘는 메시지는 돌려주지 않는다
constexpr std::size_t kMaxTrackPartBytes = std::size_t{1} << 20U;
// 한 번의 track_receive 가 건너뛸 수 있는 상한 초과 메시지 수 (FC 소유 스레드를 오래 잡지 않게)
constexpr int kMaxTrackSkips = 8;

void set_int(void* socket, int option, int value) {
    zmq_setsockopt(socket, option, &value, sizeof(value));
}

void common_options(void* socket) {
    set_int(socket, ZMQ_LINGER, 0);
    set_int(socket, ZMQ_RCVHWM, 64);
    set_int(socket, ZMQ_SNDHWM, 64);
}

void subscribe(void* socket, const std::string& topic) {
    zmq_setsockopt(socket, ZMQ_SUBSCRIBE, topic.data(), topic.size());
}

bool readable(void* socket) {
    zmq_pollitem_t item{socket, 0, ZMQ_POLLIN, 0};
    return zmq_poll(&item, 1, 0) > 0 && (item.revents & ZMQ_POLLIN) != 0;
}

// poll(0)이 읽을 수 있다고 한 뒤의 recv_multipart
std::optional<Parts> receive(void* socket) {
    if (socket == nullptr || !readable(socket)) {
        return std::nullopt;
    }
    Parts parts;
    for (;;) {
        zmq_msg_t message;
        zmq_msg_init(&message);
        if (zmq_msg_recv(&message, socket, 0) < 0) {
            zmq_msg_close(&message);
            throw std::runtime_error(zmq_strerror(zmq_errno()));
        }
        const auto* data = static_cast<const std::uint8_t*>(zmq_msg_data(&message));
        parts.emplace_back(data,
                           std::next(data, static_cast<std::ptrdiff_t>(zmq_msg_size(&message))));
        const bool more = zmq_msg_more(&message) != 0;
        zmq_msg_close(&message);
        if (!more) {
            return parts;
        }
    }
}

void send(void* socket, const Parts& parts) {
    for (std::size_t i = 0; i < parts.size(); ++i) {
        const int flags = i + 1 < parts.size() ? ZMQ_SNDMORE : 0;
        if (zmq_send(socket, parts.at(i).data(), parts.at(i).size(), flags) < 0) {
            throw std::runtime_error(zmq_strerror(zmq_errno()));
        }
    }
}

}  // namespace

ZmqFollowTransport::ZmqFollowTransport(const FollowEndpoints& endpoints) : context_(zmq_ctx_new()) {
    try {
        pub_ = zmq_socket(context_, ZMQ_PUB);
        rpc_ = zmq_socket(context_, ZMQ_REP);
        sub_ = zmq_socket(context_, ZMQ_SUB);
        for (void* socket : {pub_, rpc_, sub_}) {
            if (socket == nullptr) {
                throw std::runtime_error("ZMQ 소켓 생성 실패");
            }
            common_options(socket);
        }
        if (zmq_bind(pub_, endpoints.state.c_str()) != 0 ||
            zmq_bind(rpc_, endpoints.rpc.c_str()) != 0) {
            throw std::runtime_error(std::string("ZMQ 연결 실패: ") + zmq_strerror(zmq_errno()));
        }
        // 2026-11-25 batch 6b: 6372 는 팀 [이름][protobuf]. 2026-11-25 batch 6c: ZMQ_MAXMSGSIZE 는
        // 두지 않는다 (libzmq 4.3.5 는 넘는 프레임을 보낸 연결을 끊고 다시 맺지 않는다). 본문
        // 상한은 command_receive 의 control_msgpack 이 보고 그 메시지만 버린다
        // (command_decode_failures 가 센다)
        subscribe(sub_, "control/setpoint");
        subscribe(sub_, "follow/status");
        if (zmq_connect(sub_, endpoints.command.c_str()) != 0) {
            throw std::runtime_error(std::string("ZMQ 연결 실패: ") + zmq_strerror(zmq_errno()));
        }
        if (endpoints.tracks) {
            track_ = zmq_socket(context_, ZMQ_SUB);
            set_int(track_, ZMQ_LINGER, 0);
            set_int(track_, ZMQ_RCVHWM, 64);
            // 2026-11-25 batch 6c: ZMQ_MAXMSGSIZE 없음. 본문 상한 (1 MiB) 은 FollowBOwner 의
            // tracks_msgpack (team_wire tracks_body) 이 보고 그 메시지만 버린다
            subscribe(track_, "tracking/tracks");
            if (zmq_connect(track_, endpoints.tracks->c_str()) != 0) {
                throw std::runtime_error(std::string("ZMQ 추적 연결 실패: ") +
                                         zmq_strerror(zmq_errno()));
            }
        }
        fc_.start(pub_);  // 2026-11-25 6a 리뷰: 이 뒤로 pub_ 는 6370 송신 스레드만 쓴다
    } catch (...) {
        close();
        throw;
    }
}

ZmqFollowTransport::~ZmqFollowTransport() { close(); }

void ZmqFollowTransport::publish(const Parts& parts) {
    // 2026-11-25 batch 6a: [이름][msgpack] -> 팀 [이름][protobuf]. 6370 이 아닌 이름과 소켓 오류는
    // 예전 send 처럼 예외다. 2026-11-25 6a 리뷰: 변환과 send 는 송신 스레드가 하므로 (FC 링크 잠금
    // 밖) 소켓 오류는 다음 발행에서 예외가 된다. 변환 실패는 송신 스레드가 세고 남긴다. 대기열이
    // 가득 차 버린 메시지는 FC 소유 스레드를 멈추지 않도록 세고 1, 2, 4 ... 번째만 남긴다 (PUB HWM
    // 유실과 같다. 받는 쪽은 신선도가 끊기면 무효가 된다)
    if (parts.size() != 2) {
        throw std::runtime_error("6370 발행은 [이름][본문] 두 프레임이어야 한다");
    }
    std::string error;
    switch (fc_.publish(parts.front(), parts.back(), error)) {
        case FcPublish::Queued:
            return;
        case FcPublish::QueueFull: {
            const std::uint64_t count = fc_.stats().queue_drops;
            if (count != 0 && (count & (count - 1)) == 0) {
                std::cerr << "fc 6370 " << error << '\n';
            }
            return;
        }
        case FcPublish::NotFcTopic:
        case FcPublish::SendFailed:
            break;
    }
    throw std::runtime_error(error);
}

std::optional<Parts> ZmqFollowTransport::rpc_receive() {
    // 2026-11-25 batch 7: [control/mode_request][protobuf] -> [control/mode_request][msgpack].
    // 해석하지 못한 요청은 여기서 빈 본문으로 답하고 (REP 상태를 지킨다) 다음 요청을 본다. 한 번에
    // 건너뛰는 수는 FollowBOwner 의 tick 당 상한 (16) 과 같다
    const Bytes name(mode_rpc_name().begin(), mode_rpc_name().end());
    for (int i = 0; i < 16; ++i) {
        std::optional<Parts> parts = receive(rpc_);
        if (!parts) {
            return std::nullopt;
        }
        Bytes packed;
        std::string error;
        if (mode_rpc_request_msgpack(*parts, packed, error) == ModeRpcFrame::Ok) {
            return Parts{name, std::move(packed)};
        }
        const std::uint64_t count = ++rpc_decode_failures_;
        if ((count & (count - 1)) == 0) {
            std::cerr << "follow 6371 요청 해석 실패 (누적 " << count << "): " << error << '\n';
        }
        send(rpc_, {name, Bytes{}});
    }
    return std::nullopt;
}

void ZmqFollowTransport::rpc_send(const Parts& parts) {
    // 2026-11-25 batch 7: [control/mode_request][msgpack] -> [control/mode_request][protobuf].
    // 첫 프레임은 늘 요청 이름이다 (팀 replier 와 같다). 변환하지 못한 응답은 빈 본문
    const Bytes name(mode_rpc_name().begin(), mode_rpc_name().end());
    Bytes body;
    std::vector<std::string> unknown;
    std::string error;
    if (parts.size() != 2 || parts.front() != name) {
        error = "6371 응답은 [control/mode_request][msgpack] 이어야 한다";
    } else if (mode_rpc_reply_body(parts.back(), body, unknown, error)) {
        for (const auto& key : unknown) {
            if (rpc_unknown_seen_.insert(key).second) {
                std::cerr << "PB_UNKNOWN_FIELD " << key << " (proto에 없어 전달하지 않음)\n";
            }
        }
        send(rpc_, {name, body});
        return;
    }
    const std::uint64_t count = ++rpc_encode_failures_;
    if ((count & (count - 1)) == 0) {
        std::cerr << "follow 6371 응답 변환 실패 (누적 " << count << "): " << error << '\n';
    }
    send(rpc_, {name, Bytes{}});
}

std::optional<Parts> ZmqFollowTransport::command_receive() {
    // 2026-11-25 batch 6b: [이름][protobuf] -> [이름][msgpack]. 두 프레임이 아닌 메시지는 예전처럼
    // 그대로 돌려준다 (FollowBOwner 는 두 프레임만 본다)
    std::optional<Parts> parts = receive(sub_);
    if (!parts || parts->size() != 2) {
        return parts;
    }
    const std::string name(parts->front().begin(), parts->front().end());
    Bytes packed;
    std::string error;
    switch (control_msgpack(name, parts->back(), packed, error)) {
        case ControlFrame::Ok:
            parts->back() = std::move(packed);
            break;
        case ControlFrame::OtherName:
            parts->back() = Bytes{0xc0};  // msgpack nil
            break;
        case ControlFrame::Invalid: {
            // 빈 본문: msgpack 해석이 실패해 예전처럼 invalid_setpoint_wire 가 된다
            parts->back().clear();
            const std::uint64_t count = ++command_decode_failures_;
            if ((count & (count - 1)) == 0) {
                std::cerr << "follow 6372 해석 실패 (누적 " << count << "): " << error << '\n';
            }
            break;
        }
    }
    return parts;
}

std::optional<Parts> ZmqFollowTransport::track_receive() {
    // 2026-11-25 batch 6c: ZMQ_MAXMSGSIZE 대신 받은 뒤 본다. 부분 하나가 1 MiB 를 넘는 메시지는
    // 예전처럼 FollowBOwner 에 돌려주지 않고 세며 (track_oversized) 다음 메시지를 본다. 연결은
    // 그대로다
    for (int skipped = 0; skipped < kMaxTrackSkips; ++skipped) {
        std::optional<Parts> parts = receive(track_);
        if (!parts) {
            return std::nullopt;
        }
        const bool oversized = std::any_of(parts->begin(), parts->end(), [](const Bytes& part) {
            return part.size() > kMaxTrackPartBytes;
        });
        if (!oversized) {
            return parts;
        }
        ++track_oversized_;
    }
    return std::nullopt;
}

void ZmqFollowTransport::close() {
    fc_.stop();  // 2026-11-25 6a 리뷰: 대기열을 보내고 송신 스레드를 끝낸 뒤 pub_ 를 닫는다
    for (void** socket : {&track_, &pub_, &rpc_, &sub_}) {
        if (*socket != nullptr) {
            zmq_close(*socket);
            *socket = nullptr;
        }
    }
    if (context_ != nullptr) {
        zmq_ctx_term(context_);
        context_ = nullptr;
    }
}

}  // namespace argus::gateway
