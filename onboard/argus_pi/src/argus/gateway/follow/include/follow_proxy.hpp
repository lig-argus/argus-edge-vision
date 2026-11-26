// Pi 내부 B 추종기 요청 연결부 (Pi tools/real_fc/follow_proxy.py와 같은 규칙).
// 세션과 시각은 Windows에서 오지 않는다. 결과를 모르면 자동 재전송하지 않는다.
#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "argus/gateway/common/include/msgpack_codec.hpp"

namespace argus::gateway {

// 대상 식별값 검사 (원본 selection_identity)
Wire selection_identity(const Wire& body, bool allow_icd = false);
// B 추종기 STATUS 응답 검사 (원본 validate_status / validate_observation_status)
void validate_follow_status(const Wire& status, const std::string& uid);
void validate_observation_status(const Wire& status, const std::string& uid);

class FollowProxy {
public:
    // ZMQ REQ 송수신 한 번: 보낸 멀티파트에 대한 응답 멀티파트. 전송 실패는 TransportError.
    using Transport =
        std::function<std::vector<Bytes>(const std::vector<Bytes>& parts, int timeout_ms)>;
    using ClockNs = std::function<std::int64_t()>;
    using IdSource = std::function<std::string()>;

    // 기본: endpoint로 ZMQ REQ, 시계는 steady_clock, ID는 무작위 32자 16진수
    explicit FollowProxy(std::string endpoint, Transport transport = {}, ClockNs clock = {},
                         IdSource new_id = {});

    // 원본 forward_vmy(body, expected_uid, timeout_ms)
    Wire forward(const Wire& body, const std::string& expected_uid, int timeout_ms = 1200);

    const std::string& source() const { return source_; }

private:
    Wire exchange(const Wire& request, int timeout_ms);

    std::string endpoint_;
    Transport transport_;
    ClockNs clock_;
    IdSource new_id_;
    std::string source_;
    std::int64_t seq_{0};
    std::mutex mutex_;
};

// 원본 zmq.ZMQError에 해당 (연결·시간 초과)
class TransportError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// 실제 ZMQ REQ 송수신 한 번 (LINGER 0, 송수신 제한 시간 timeout_ms)
std::vector<Bytes> zmq_request(const std::string& endpoint, const std::vector<Bytes>& parts,
                               int timeout_ms);

}  // namespace argus::gateway
