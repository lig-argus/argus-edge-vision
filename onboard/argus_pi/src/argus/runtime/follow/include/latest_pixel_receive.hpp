// 추적기 입력에서 가장 최신 관측 하나만 꺼낸다 (2026-11-09 사용자 요청).
// 실기: 입력이 약 6프레임(200ms) 밀린 채 한 프레임 처리가 프레임 간격(33ms)과 같아 줄지 않았고,
// 모든 프레임이 카메라 전용 250ms 신선도 기준을 넘겨 버려졌다. 대기 중인 메시지를 모두 받아
// 마지막 정상 메시지만 처리한다. 메시지가 topic·payload 두 부분이라 ZMQ_CONFLATE는 쓸 수 없다.
#pragma once

#include <zmq.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace follow {

enum class PixelReceive { Received, Empty, Invalid };

// (topic, payload) 메시지 하나를 기다리지 않고 받는다. 형식이 틀리면 남은 부분까지 비운다.
// 받을 메시지가 없으면 Empty. 전송 오류는 예외로 알린다.
inline PixelReceive receive_pixel_message(void* socket, const std::string& topic,
                                          std::size_t maximum, std::string& payload,
                                          std::string& error) {
    constexpr std::size_t kMaximumTopic = 128;
    payload.clear();
    error.clear();
    int part_index = 0;
    bool more = true;
    while (more) {
        zmq_msg_t part;
        if (zmq_msg_init(&part) != 0) {
            throw std::runtime_error(std::string("pixel init: ") + zmq_strerror(zmq_errno()));
        }
        // 첫 부분만 기다리지 않는다. 나머지 부분은 첫 부분과 함께 이미 도착해 있다.
        const int received = zmq_msg_recv(&part, socket, part_index == 0 ? ZMQ_DONTWAIT : 0);
        if (received < 0) {
            const int code = zmq_errno();
            zmq_msg_close(&part);
            if (part_index == 0 && code == EAGAIN) {
                return PixelReceive::Empty;
            }
            throw std::runtime_error(std::string("pixel recv: ") + zmq_strerror(code));
        }
        const auto size = zmq_msg_size(&part);
        more = zmq_msg_more(&part) != 0;
        const auto* data = static_cast<const char*>(zmq_msg_data(&part));
        if (error.empty()) {
            if (part_index == 0) {
                if (!more || size > kMaximumTopic || std::string(data, size) != topic) {
                    error = "pixel multipart topic invalid";
                }
            } else if (part_index == 1 && !more && size <= maximum) {
                payload.assign(data, size);
            } else {
                error = "pixel multipart size invalid";
            }
        }
        zmq_msg_close(&part);
        ++part_index;
    }
    if (error.empty() && part_index != 2) {
        error = "pixel multipart size invalid";
    }
    if (!error.empty()) {
        payload.clear();
        return PixelReceive::Invalid;
    }
    return PixelReceive::Received;
}

struct LatestPixel {
    std::string payload;
    // 정상 메시지가 하나도 없을 때의 첫 오류
    std::string error;
    bool valid{false};
    // 더 새 메시지가 있어 처리하지 않고 넘긴 정상 메시지 수
    std::uint64_t skipped{0};
    // 형식이 틀려 넘긴 메시지 수
    std::uint64_t invalid{0};
};

// 대기 중인 메시지를 모두(최대 max_messages) 받고 마지막 정상 메시지만 남긴다. 가장 새 메시지가
// 깨졌으면 그 앞의 정상 메시지를 쓴다. 정상 메시지가 없으면 valid=false와 오류를 돌려준다.
// 2026-11-25 batch 4: 형식(이름, 2 부분, 크기)이 맞는 메시지마다 accept(payload, error) 로 본문을
// 해석한다 (6374 protobuf). accept 가 false 이면 형식이 틀린 메시지와 같이 invalid 로 세고 그 앞의
// 정상 메시지를 쓴다. 최신 하나만 처리하는 규칙과 skipped, invalid 수는 예전과 같다.
template <class Accept>
LatestPixel receive_latest_accepted(void* socket, const std::string& topic, std::size_t maximum,
                                    Accept&& accept, std::size_t max_messages = 64) {
    LatestPixel latest;
    std::string payload;
    std::string error;
    for (std::size_t count = 0; count < max_messages; ++count) {
        auto result = receive_pixel_message(socket, topic, maximum, payload, error);
        if (result == PixelReceive::Empty) {
            break;
        }
        if (result == PixelReceive::Received && !accept(payload, error)) {
            result = PixelReceive::Invalid;
        }
        if (result == PixelReceive::Invalid) {
            ++latest.invalid;
            if (!latest.valid && latest.error.empty()) {
                latest.error = error;
            }
            continue;
        }
        if (latest.valid) {
            ++latest.skipped;
        }
        latest.payload.swap(payload);
        latest.valid = true;
        latest.error.clear();
    }
    if (!latest.valid && latest.error.empty()) {
        latest.error = "pixel queue empty after poll";
    }
    return latest;
}

inline LatestPixel receive_latest_pixel(void* socket, const std::string& topic, std::size_t maximum,
                                        std::size_t max_messages = 64) {
    return receive_latest_accepted(
        socket, topic, maximum,
        [](const std::string& /*payload*/, std::string& /*error*/) { return true; }, max_messages);
}

}  // namespace follow
