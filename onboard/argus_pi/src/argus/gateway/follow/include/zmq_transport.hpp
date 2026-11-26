// B 추종 송신부의 실제 ZMQ 전송 (원본 FollowBOwner.__init__의 소켓 구성과 같다).
// PUB bind fc_state, REP bind mode_rpc, SUB connect control(control/setpoint·follow/status),
// cv_gap일 때만 SUB connect tracks(tracking/tracks, 최대 1MB). 모두 LINGER 0, HWM 64.
// 2026-11-25 batch 6a: PUB (6370) 은 팀 규약 [이름][protobuf] 로 보낸다 (fc_protobuf.hpp). publish
// 에 들어오는 [이름][msgpack] 은 예전과 같고, 이 전송이 바꾼다. SNDHWM 은 예전 값 64 를 그대로
// 둔다. 2026-11-25 6a 리뷰: 변환과 send 는 FcTelemetryWire 의 송신 스레드가 한다 (FC 링크 잠금
// 밖). 생성자 뒤로 PUB 소켓은 그 스레드만 쓰고, close 가 대기열을 보낸 뒤 스레드를 끝낸다.
// 2026-11-25 batch 6b: SUB (6372) 는 팀 규약 [이름][protobuf] 를 받는다 (control_protobuf.hpp).
// command_receive 는 예전처럼 [이름][msgpack] 을 돌려준다 (FollowBOwner 그대로). 해석하지 못한
// 본문은 빈 본문으로 돌려주어 FollowBOwner 가 예전 msgpack 실패처럼 invalid_setpoint_wire 로
// 남긴다. 다른 이름은 msgpack nil 본문 (FollowBOwner 는 이름이 달라 건너뛴다). RCVHWM 은 예전 값
// 64.
// 2026-11-25 batch 6c: ZMQ_MAXMSGSIZE 는 두지 않는다. 6372 본문 상한 (1 MiB) 은 받은 뒤
// control_msgpack 이 보고 그 메시지만 해석 실패로 센다. 추적 1MB 는 track_receive 가 받은 뒤
// 본다 (연결은 그대로).
// 2026-11-25 batch 7: REP (6371) 는 팀 규약 [control/mode_request][ControlModeRequest] 를 받고
// [control/mode_request][ControlModeReply] 로 답한다 (mode_rpc_protobuf.hpp). rpc_receive 는
// [control/mode_request][msgpack] 을 돌려주고 rpc_send 는 [control/mode_request][msgpack] 을 받아
// 바꾼다. 해석하지 못한 요청은 FollowBOwner 에 넘기지 않고 이 전송이 [control/mode_request][빈
// 본문] 으로 답한다 (팀 replier 의 처리 실패 응답 모양). 변환하지 못한 응답도 빈 본문으로 보낸다.
#pragma once

#include <cstdint>
#include <optional>
#include <set>
#include <string>

#include "argus/gateway/follow/include/fc_protobuf.hpp"
#include "argus/gateway/follow/include/follow_owner.hpp"

namespace argus::gateway {

struct FollowEndpoints {
    std::string state;                  // fc_state (PUB bind)
    std::string rpc;                    // mode_rpc (REP bind)
    std::string command;                // control (SUB connect)
    std::optional<std::string> tracks;  // tracks (SUB connect, cv_gap일 때만)
};

class ZmqFollowTransport final : public IFollowTransport {
public:
    // 묶기·연결에 실패하면 이미 연 소켓을 닫고 std::runtime_error를 던진다 (원본 close 후 raise)
    explicit ZmqFollowTransport(const FollowEndpoints& endpoints);
    ZmqFollowTransport(const ZmqFollowTransport&) = delete;
    ZmqFollowTransport& operator=(const ZmqFollowTransport&) = delete;
    ZmqFollowTransport(ZmqFollowTransport&&) = delete;
    ZmqFollowTransport& operator=(ZmqFollowTransport&&) = delete;
    ~ZmqFollowTransport() override;

    void publish(const Parts& parts) override;
    std::optional<Parts> rpc_receive() override;
    void rpc_send(const Parts& parts) override;
    std::optional<Parts> command_receive() override;
    bool has_tracks() const override { return track_ != nullptr; }
    std::optional<Parts> track_receive() override;
    void close() override;
    // 6372 해석 실패 수 (본문 상한 초과 포함). command_receive 를 부르는 스레드에서만 읽는다
    std::uint64_t command_decode_failures() const { return command_decode_failures_; }
    // 2026-11-25 batch 6c: 1 MiB 를 넘어 track_receive 가 버린 추적 메시지 수 (같은 스레드에서만)
    std::uint64_t track_oversized() const { return track_oversized_; }
    // 2026-11-25 batch 7: 6371 요청을 해석하지 못해 빈 본문으로 답한 수, 응답을 변환하지 못해 빈
    // 본문으로 답한 수 (rpc_receive, rpc_send 를 부르는 스레드에서만 읽는다)
    std::uint64_t rpc_decode_failures() const { return rpc_decode_failures_; }
    std::uint64_t rpc_encode_failures() const { return rpc_encode_failures_; }

private:
    void* context_{nullptr};
    void* pub_{nullptr};
    void* rpc_{nullptr};
    void* sub_{nullptr};
    void* track_{nullptr};
    std::uint64_t track_oversized_{0};
    std::uint64_t command_decode_failures_{0};  // 6372 본문 해석 실패 (command_receive 쪽 스레드)
    std::uint64_t rpc_decode_failures_{0};      // 6371 요청 해석 실패 (rpc_receive 쪽 스레드)
    std::uint64_t rpc_encode_failures_{0};      // 6371 응답 변환 실패 (같은 스레드)
    std::set<std::string> rpc_unknown_seen_;    // 처음 한 번 남긴 응답의 proto 에 없는 키
    FcTelemetryWire fc_;  // 6370 토픽별 팀 seq, 송신 대기열과 송신 스레드 (pub_ 는 그 스레드만)
};

}  // namespace argus::gateway
