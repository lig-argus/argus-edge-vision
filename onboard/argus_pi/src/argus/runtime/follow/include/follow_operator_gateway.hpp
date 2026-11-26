// 제어 thread에서만 운용자 요청을 실행한다. SDK/B 어댑터 모두 자동 arm/takeoff는 없다.
//
// 2026-11-25 batch 8 (6373 팀 전환): B 추종 (allow_follow_commands) 은 팀 규약
// [follow/command][FollowCommand] 만 받고 [follow/command][FollowResult] 로 답한다 (응답 첫
// 프레임은 요청 이름, 팀 ZmqReplier 와 같다). 본문 변환은 team_wire (follow_command_pb_codec) 이고
// 받은 뒤의 검사 (admission, permit, seq, 재전송 cache, 응답 꾸밈) 는 예전 그대로다. 해석하지
// 못한 요청 (프레임 수, 이름, 256 KiB, 빈 본문, protobuf) 은 [follow/command][빈 본문] 으로
// 답한다 (REP 상태 유지). 예전 [follow/command][msgpack] 과 한 프레임 argus.vmy.request.v1 은
// B 에서 받지 않는다. B 가 아닌 6363 경로 (allow_follow_commands false) 는 예전 msgpack 그대로다.
#pragma once
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>

#include "argus/common/serialization/include/native_json_io.hpp"
#include "argus/control/contracts/include/i_follow_command.hpp"
#include "argus/transport/zeromq/include/zeromq_operations.hpp"
namespace argus::team_wire {
class TeamWire;
}  // namespace argus::team_wire
namespace follow {
class VMyOperatorGateway {
public:
    VMyOperatorGateway(ZeroMqContext&, const std::string& endpoint,
                       bool allow_follow_commands = false,
                       std::function<bool(const NativeJson&)> permit = {},
                       std::map<std::uint32_t, std::string> sensor_sources = {});
    ~VMyOperatorGateway();
    VMyOperatorGateway(const VMyOperatorGateway&) = delete;
    VMyOperatorGateway& operator=(const VMyOperatorGateway&) = delete;
    VMyOperatorGateway(VMyOperatorGateway&&) = delete;
    VMyOperatorGateway& operator=(VMyOperatorGateway&&) = delete;
    void poll(const std::function<NativeJson(const NativeJson&)>& handler);
    void poll(IFollowCommand& handler);

private:
    // 받은 요청 하나의 응답 (follow_wire 이면 follow/result 꾸밈까지). failure 가 비어 있지 않으면
    // 그 까닭의 거부 응답
    NativeJson respond(const NativeJson& input, bool follow_wire, IFollowCommand& handler,
                       const std::string& failure);
    // B: 팀 [follow/command][protobuf] 요청 하나 (있으면) 를 받아 답한다
    void serve_team(IFollowCommand& handler);

    ZmqSocket socket_;
    bool allow_follow_commands_{false};
    std::function<bool(const NativeJson&)> permit_;
    std::map<std::uint32_t, std::string> sensor_sources_;
    std::map<std::string, std::pair<std::string, NativeJson>> previous_;
    using SequenceKey = std::tuple<std::string, std::string, std::string, std::uint64_t>;
    std::map<SequenceKey, std::pair<std::uint64_t, std::string>> sequences_;
    std::uint64_t reply_sequence_{0};
    // B 만. 해석하지 못한 요청, 변환하지 못한 응답, respond 밖으로 나온 예외의 수
    // (1, 2, 4 ... 번째만 남긴다)
    std::unique_ptr<argus::team_wire::TeamWire> team_wire_;
    std::uint64_t team_rejects_{0};
    std::uint64_t team_encode_failures_{0};
    std::uint64_t team_handler_failures_{0};
};
}  // namespace follow
