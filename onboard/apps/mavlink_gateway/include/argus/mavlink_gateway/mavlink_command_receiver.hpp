// 통제기가 보낸 명령(COMMAND_LONG)을 받아 공통 규칙을 적용하고 handler 에 넘긴다
// 규약: docs/spec/icd.md 4.1 (대상, 명령 번호, 재전송, 동시 처리, IN_PROGRESS 재송신, STATUSTEXT),
//      5.1 (제어권이 없는 통제기의 명령), 5.3, 5.4 (착륙 잠금, 조종사 개입 중의 명령),
//      docs/design/uml/class_mavlink_gateway.md 3절
// 사용: mavlink_gateway 의 main 이 하나를 생성하여 handler 를 등록하고 tick() 을 1 초 주기 작업으로
//      등록
#ifndef ARGUS_ONBOARD_MAVLINK_COMMAND_RECEIVER_HPP_
#define ARGUS_ONBOARD_MAVLINK_COMMAND_RECEIVER_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <tuple>
#include <vector>

#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/command_activity.hpp"
#include "argus/mavlink_gateway/control_authority.hpp"
#include "argus/mavlink_gateway/gcs_link.hpp"
#include "argus/mavlink_gateway/gcs_message_source.hpp"
#include "argus/mavlink_gateway/landing_lock_state.hpp"
#include "argus/mavlink_gateway/mavlink_command_handler.hpp"
#include "argus/mavlink_gateway/rc_override_state.hpp"

namespace argus::mavlink_gateway {

// tick() 의 호출 주기. 진행 중인 명령의 IN_PROGRESS 재송신 주기와 같다 (icd 4.1)
inline constexpr std::chrono::milliseconds kCommandReceiverTickPeriod{1000};

// 명령 번호 기록을 지우는 시간. 마지막 명령 뒤 이 시간이 지난 기록은 지운다 (icd 4.1)
inline constexpr std::int64_t kCommandRecordLifetimeNs = 60'000'000'000;

// 동시 처리 규칙에서의 명령 종류 (icd 4.1)
enum class CommandKind : std::uint8_t {
    Start,  // 시작 명령 (TRACK_START, SCAN_START)
    Stop,   // 정지 명령 (TRACK_STOP). 진행 중인 시작 명령을 CANCELLED 로 끝내고 처리한다
    Other,  // 그 밖의 명령. 같은 MAV_CMD 가 진행 중일 때만 TEMPORARILY_REJECTED
};

// handler 를 등록할 때 정하는 공통 규칙의 적용 방법
struct CommandRules {
    bool numbered = true;          // param7 의 명령 번호로 재전송을 판단 (argus 명령, icd 4.1)
    bool requires_control = true;  // 제어권이 없는 통제기의 명령을 거부 (icd 4.1, 5.1)
    CommandKind kind = CommandKind::Other;

    // 착륙 잠금이나 조종사 개입 중에도 받는다 (icd 5.3, 5.4). 45010(해제가 목적),
    // MAV_CMD_REQUEST_MESSAGE(기체를 움직이지 않는 조회), 45011(복귀가 목적)
    bool allowed_while_locked = false;
};

// 명령 하나를 받으면 다음 순서로 처리한다.
// 1. target_system 이 1 이 아니거나 target_component 가 195 가 아니면 응답 없이 폐기
// 2. 등록된 handler 가 없으면 UNSUPPORTED
// 3. 명령 번호 규칙: param7 이 0~16,777,215 의 정수가 아니면 DENIED. (sysid, compid, MAV_CMD) 의
//    보관 번호와 같으면 재전송이므로 진행 중이면 IN_PROGRESS, 끝났으면 보관한 최종 결과를 다시
//    보내고, 보관 번호보다 오래된 번호면 응답 없이 폐기 (24 비트 순환 비교)
// 4. 제어권이 필요한 명령을 제어권이 없는 통제기가 보내면 TEMPORARILY_REJECTED 와 STATUSTEXT
//    "제어권 없음"
// 5. allowed_while_locked 가 아닌 명령을 착륙 잠금 중에 받으면 TEMPORARILY_REJECTED 와 STATUSTEXT
//    "착륙 잠금", 조종사 개입 중에 받으면 "조종사 개입 중"
// 6. 동시 처리: 진행 중인 명령이 있을 때 같은 MAV_CMD 나 시작 명령은 TEMPORARILY_REJECTED, 정지
//    명령은 진행 중인 시작 명령을 CANCELLED 로 끝낸 뒤 handler 에 넘김
// 7. handler 에 넘김
// COMMAND_INT 로 온 명령은 handler 가 있으면 COMMAND_LONG_ONLY, 없으면 UNSUPPORTED 로 답한다.
// 응답은 명령을 보낸 통제기에만 보내며, CommandAck::reason 이 비어 있지 않으면 STATUSTEXT 로 함께
// 보낸다
class MavlinkCommandReceiver {
public:
    // 인자로 받은 객체들은 이 객체보다 오래 유지되어야 한다
    MavlinkCommandReceiver(IGcsMessageSource& messages, IGcsLink& link, const core::IClock& clock,
                           const IControlAuthority& authority, CommandActivity& activity,
                           const ILandingLockState& landing_lock,
                           const IRcOverrideState& rc_override);

    MavlinkCommandReceiver(const MavlinkCommandReceiver&) = delete;
    MavlinkCommandReceiver& operator=(const MavlinkCommandReceiver&) = delete;
    MavlinkCommandReceiver(MavlinkCommandReceiver&&) = delete;
    MavlinkCommandReceiver& operator=(MavlinkCommandReceiver&&) = delete;
    ~MavlinkCommandReceiver() = default;

    // handler 를 등록한다. handler 는 이 객체보다 오래 유지되어야 한다.
    // 같은 MAV_CMD 의 handler 가 이미 있으면 등록하지 않고 false
    bool add_handler(IMavlinkCommandHandler& handler, CommandRules rules);

    // 진행 중인 명령의 마지막 IN_PROGRESS 를 다시 보내고 오래된 명령 번호 기록을 지운다.
    // kCommandReceiverTickPeriod 마다 호출한다
    void tick();

    // 최종 결과를 보내지 않은 명령을 모두 CANCELLED 로 끝낸다. 착륙 잠금이 시작될 때 부른다
    // (icd 5.3). 그 뒤 handler 가 보내는 결과는 무시된다
    void cancel_in_progress();

private:
    // 명령 하나의 실행. handler 가 가진 CommandResponder 와 이 객체가 함께 참조한다
    struct Execution {
        GcsPeer sender;
        std::uint16_t command = 0;
        CommandKind kind = CommandKind::Other;
        bool completed = false;
        std::optional<CommandAck> last_ack;  // 마지막으로 보낸 응답
    };

    // CommandResponder 가 이 객체보다 오래 남아도 안전하도록 응답 송신에 필요한 것을 공유 객체에
    // 두고, CommandResponder 의 송신 함수는 weak_ptr 로 참조한다
    struct Shared {
        IGcsLink& link;
        CommandActivity& activity;
        std::vector<std::shared_ptr<Execution>> active;  // 최종 결과를 보내지 않은 실행
    };

    struct Registration {
        IMavlinkCommandHandler* handler = nullptr;
        CommandRules rules;
    };

    // (sysid, compid, MAV_CMD) 의 마지막 명령 번호와 그 실행
    struct NumberRecord {
        std::uint32_t number = 0;
        std::shared_ptr<Execution> execution;
        std::int64_t last_received_ns = 0;
    };
    using RecordKey = std::tuple<std::uint8_t, std::uint8_t, std::uint16_t>;

    void on_command_long(const mavlink_message_t& message, const GcsPeer& sender);
    void on_command_int(const mavlink_message_t& message, const GcsPeer& sender);
    // 진행 중인 실행 때문에 바로 답해야 하면 그 결과. 정지 명령이면 진행 중인 시작 명령을 끝낸다
    std::optional<MAV_RESULT> apply_concurrency(std::uint16_t command, CommandKind kind);

    // 실행을 만들어 active 에 넣는다
    std::shared_ptr<Execution> start_execution(const GcsPeer& sender, std::uint16_t command,
                                               CommandKind kind);
    CommandResponder make_responder(const std::shared_ptr<Execution>& execution) const;

    static void send_ack(Shared& shared, Execution& execution, const CommandAck& ack);
    static void update_activity(Shared& shared);
    static void transmit(IGcsLink& link, const Execution& execution, const CommandAck& ack);

    const core::IClock& clock_;
    const IControlAuthority& authority_;
    const ILandingLockState& landing_lock_;
    const IRcOverrideState& rc_override_;
    std::shared_ptr<Shared> shared_;
    std::map<std::uint16_t, Registration> registrations_;
    std::map<RecordKey, NumberRecord> records_;

    // 멤버 중 가장 먼저 소멸하도록 마지막에 선언한다
    core::Subscription command_long_subscription_;
    core::Subscription command_int_subscription_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_MAVLINK_COMMAND_RECEIVER_HPP_
