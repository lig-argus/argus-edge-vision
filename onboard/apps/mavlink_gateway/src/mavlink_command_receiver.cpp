#include "argus/mavlink_gateway/mavlink_command_receiver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include "argus/mavlink_gateway/mavlink_ids.hpp"
#include "gcs_message_encoding.hpp"

namespace argus::mavlink_gateway {
namespace {

// 명령 번호는 24 비트 (icd 4.1)
constexpr std::uint32_t kNumberModulo = 16'777'216;
constexpr std::uint32_t kNumberHalf = kNumberModulo / 2;

// param7 의 명령 번호. 0~16,777,215 의 정수가 아니면 nullopt
std::optional<std::uint32_t> command_number(float param7) {
    if (!std::isfinite(param7) || param7 < 0.0F || param7 >= static_cast<float>(kNumberModulo) ||
        std::floor(param7) != param7) {
        return std::nullopt;
    }
    return static_cast<std::uint32_t>(param7);
}

// 24 비트 순환 비교로 received 가 stored 보다 새 번호인지 (icd 4.1)
bool is_newer(std::uint32_t received, std::uint32_t stored) {
    const std::uint32_t difference = (received + kNumberModulo - stored) % kNumberModulo;
    return difference != 0 && difference < kNumberHalf;
}

// 제어권이 없는 통제기의 명령에 보내는 STATUSTEXT (icd 4.1)
constexpr const char* kNotInControlText = "제어권 없음";

// 착륙 잠금, 조종사 개입 중에 거부하는 명령에 보내는 STATUSTEXT (icd 5.3, 5.4)
constexpr const char* kLandingLockedText = "착륙 잠금";
constexpr const char* kPilotInControlText = "조종사 개입 중";

bool is_for_onboard(std::uint8_t target_system, std::uint8_t target_component) {
    return target_system == kVehicleSysid && target_component == kOnboardCommandCompid;
}

}  // namespace

MavlinkCommandReceiver::MavlinkCommandReceiver(IGcsMessageSource& messages, IGcsLink& link,
                                               const core::IClock& clock,
                                               const IControlAuthority& authority,
                                               CommandActivity& activity,
                                               const ILandingLockState& landing_lock,
                                               const IRcOverrideState& rc_override)
    : clock_(clock),
      authority_(authority),
      landing_lock_(landing_lock),
      rc_override_(rc_override),
      shared_(std::make_shared<Shared>(Shared{link, activity, {}})) {
    command_long_subscription_ =
        messages.subscribe(MAVLINK_MSG_ID_COMMAND_LONG,
                           [this](const mavlink_message_t& message, const GcsPeer& sender) {
                               on_command_long(message, sender);
                           });
    command_int_subscription_ =
        messages.subscribe(MAVLINK_MSG_ID_COMMAND_INT,
                           [this](const mavlink_message_t& message, const GcsPeer& sender) {
                               on_command_int(message, sender);
                           });
}

bool MavlinkCommandReceiver::add_handler(IMavlinkCommandHandler& handler, CommandRules rules) {
    return registrations_.emplace(handler.command(), Registration{&handler, rules}).second;
}

void MavlinkCommandReceiver::tick() {
    for (const std::shared_ptr<Execution>& execution : shared_->active) {
        if (!execution->completed && execution->last_ack.has_value() &&
            execution->last_ack->result == MAV_RESULT_IN_PROGRESS) {
            transmit(shared_->link, *execution, *execution->last_ack);
        }
    }
    const std::int64_t now_ns = clock_.now_monotonic_ns();
    for (auto it = records_.begin(); it != records_.end();) {
        const bool expired = now_ns - it->second.last_received_ns > kCommandRecordLifetimeNs;
        const bool running = it->second.execution && !it->second.execution->completed;
        it = expired && !running ? records_.erase(it) : std::next(it);
    }
}

void MavlinkCommandReceiver::cancel_in_progress() {
    // send_ack 가 active 를 바꾸므로 복사본을 돈다
    const std::vector<std::shared_ptr<Execution>> running = shared_->active;
    for (const std::shared_ptr<Execution>& execution : running) {
        send_ack(*shared_, *execution, CommandAck{MAV_RESULT_CANCELLED, kProgressUnknown, {}});
    }
}

void MavlinkCommandReceiver::on_command_long(const mavlink_message_t& message,
                                             const GcsPeer& sender) {
    mavlink_command_long_t command{};
    mavlink_msg_command_long_decode(&message, &command);
    if (!is_for_onboard(command.target_system, command.target_component)) {
        return;
    }
    const auto found = registrations_.find(command.command);
    if (found == registrations_.end()) {
        const Execution once{sender, command.command, CommandKind::Other, true, std::nullopt};
        transmit(shared_->link, once, CommandAck{MAV_RESULT_UNSUPPORTED, kProgressUnknown, {}});
        return;
    }
    const Registration& registration = found->second;
    const CommandRules& rules = registration.rules;

    NumberRecord* record = nullptr;
    if (rules.numbered) {
        const std::optional<std::uint32_t> number = command_number(command.param7);
        if (!number.has_value()) {
            const Execution once{sender, command.command, rules.kind, true, std::nullopt};
            transmit(shared_->link, once,
                     CommandAck{MAV_RESULT_DENIED, kProgressUnknown, "invalid command number"});
            return;
        }
        const RecordKey key{sender.sysid, sender.compid, command.command};
        const auto existing = records_.find(key);
        if (existing != records_.end() && existing->second.execution) {
            NumberRecord& stored = existing->second;
            if (*number == stored.number) {
                // 재전송: handler 를 부르지 않고 마지막 응답을 다시 보낸다
                stored.last_received_ns = clock_.now_monotonic_ns();
                const Execution& execution = *stored.execution;
                const CommandAck again =
                    execution.completed || execution.last_ack.has_value()
                        ? execution.last_ack.value_or(CommandAck{})
                        : CommandAck{MAV_RESULT_IN_PROGRESS, kProgressUnknown, {}};
                transmit(shared_->link, execution, again);
                return;
            }
            if (!is_newer(*number, stored.number)) {
                return;  // 늦게 도착한 재전송
            }
        }
        record = &records_[key];
        record->number = *number;
        record->last_received_ns = clock_.now_monotonic_ns();
    }

    std::optional<CommandAck> immediate;
    if (rules.requires_control && !authority_.is_controller(sender.sysid)) {
        immediate =
            CommandAck{MAV_RESULT_TEMPORARILY_REJECTED, kProgressUnknown, kNotInControlText};
    } else if (!rules.allowed_while_locked && landing_lock_.active()) {
        immediate =
            CommandAck{MAV_RESULT_TEMPORARILY_REJECTED, kProgressUnknown, kLandingLockedText};
    } else if (!rules.allowed_while_locked && rc_override_.pilot_in_control()) {
        immediate =
            CommandAck{MAV_RESULT_TEMPORARILY_REJECTED, kProgressUnknown, kPilotInControlText};
    } else if (const std::optional<MAV_RESULT> busy =
                   apply_concurrency(command.command, rules.kind)) {
        immediate = CommandAck{*busy, kProgressUnknown, {}};
    }

    const std::shared_ptr<Execution> execution =
        start_execution(sender, command.command, rules.kind);
    if (record != nullptr) {
        record->execution = execution;
    }
    if (immediate.has_value()) {
        send_ack(*shared_, *execution, *immediate);
        return;
    }
    // 진행 중인 명령 여부(CommandActivity)는 handler 가 돌아온 뒤 반영한다. handler 안의
    // 판단(45010)이 이 명령 자신 때문에 거부되지 않게 한다
    registration.handler->handle(CommandRequest{sender, command}, make_responder(execution));
    update_activity(*shared_);
}

void MavlinkCommandReceiver::on_command_int(const mavlink_message_t& message,
                                            const GcsPeer& sender) {
    mavlink_command_int_t command{};
    mavlink_msg_command_int_decode(&message, &command);
    if (!is_for_onboard(command.target_system, command.target_component)) {
        return;
    }
    const MAV_RESULT result = registrations_.count(command.command) != 0
                                  ? MAV_RESULT_COMMAND_LONG_ONLY
                                  : MAV_RESULT_UNSUPPORTED;
    const Execution once{sender, command.command, CommandKind::Other, true, std::nullopt};
    transmit(shared_->link, once, CommandAck{result, kProgressUnknown, {}});
}

std::optional<MAV_RESULT> MavlinkCommandReceiver::apply_concurrency(std::uint16_t command,
                                                                    CommandKind kind) {
    bool busy = false;
    for (const std::shared_ptr<Execution>& running : shared_->active) {
        if (running->command == command || kind == CommandKind::Start) {
            busy = true;
        }
    }
    if (busy) {
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }
    if (kind == CommandKind::Stop) {
        // send_ack 가 active 를 바꾸므로 복사본을 돈다
        const std::vector<std::shared_ptr<Execution>> running = shared_->active;
        for (const std::shared_ptr<Execution>& execution : running) {
            if (execution->kind == CommandKind::Start) {
                send_ack(*shared_, *execution,
                         CommandAck{MAV_RESULT_CANCELLED, kProgressUnknown, {}});
            }
        }
    }
    return std::nullopt;
}

std::shared_ptr<MavlinkCommandReceiver::Execution> MavlinkCommandReceiver::start_execution(
    const GcsPeer& sender, std::uint16_t command, CommandKind kind) {
    auto execution = std::make_shared<Execution>(Execution{sender, command, kind, false, {}});
    shared_->active.push_back(execution);
    return execution;
}

CommandResponder MavlinkCommandReceiver::make_responder(
    const std::shared_ptr<Execution>& execution) const {
    const std::weak_ptr<Shared> weak = shared_;
    return CommandResponder([weak, execution](const CommandAck& ack) {
        if (const auto shared = weak.lock()) {
            send_ack(*shared, *execution, ack);
        }
    });
}

void MavlinkCommandReceiver::send_ack(Shared& shared, Execution& execution, const CommandAck& ack) {
    if (execution.completed) {
        return;  // CANCELLED 로 끝난 뒤 handler 가 보낸 결과
    }
    execution.last_ack = ack;
    execution.completed = ack.result != MAV_RESULT_IN_PROGRESS;
    transmit(shared.link, execution, ack);
    if (execution.completed) {
        auto& active = shared.active;
        active.erase(std::remove_if(active.begin(), active.end(),
                                    [&execution](const std::shared_ptr<Execution>& item) {
                                        return item.get() == &execution;
                                    }),
                     active.end());
    }
    update_activity(shared);
}

void MavlinkCommandReceiver::update_activity(Shared& shared) {
    shared.activity.set_command_in_progress(!shared.active.empty());
}

void MavlinkCommandReceiver::transmit(IGcsLink& link, const Execution& execution,
                                      const CommandAck& ack) {
    link.send_to(execution.sender,
                 detail::encode_command_ack(execution.command, ack.result, ack.progress,
                                            execution.sender.sysid, execution.sender.compid));
    if (!ack.reason.empty()) {
        const MAV_SEVERITY severity =
            ack.result == MAV_RESULT_FAILED ? MAV_SEVERITY_ERROR : MAV_SEVERITY_WARNING;
        link.send_to(execution.sender, detail::encode_statustext(severity, ack.reason));
    }
}

}  // namespace argus::mavlink_gateway
