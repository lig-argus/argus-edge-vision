// 명령을 지금 보낼 수 있는지 판단한다. 안 되면 운용자에게 보일 이유를 돌려준다 (버튼 활성화와 안내,
// 보내기 전 확인). mavlink_gateway 가 거부할 명령을 미리 막아 거부 응답 대신 이유를 보인다:
// 대상 링크와 응답 대기(icd 4.1), 제어권(5.1), 조종사 개입(5.4), FC 전달 조건(5.2), 추종과 스캔의
// 시작 조건(4.2)
#ifndef ARGUS_GCS_LINK_COMMAND_POLICY_HPP_
#define ARGUS_GCS_LINK_COMMAND_POLICY_HPP_

#include <QString>
#include <cstdint>
#include <optional>
#include <vector>

#include "argus/gcs/link/command.hpp"
#include "argus/gcs/link/flight_conditions.hpp"
#include "argus/gcs/link/vehicle_state.hpp"

namespace argus::gcs::link {

// 판단에 쓰는 통제기 쪽 상태
struct CommandContext {
    // 보내고 응답을 기다리는 명령 (CommandSender::awaitingCommand 가 참인 명령)
    std::vector<Command> awaiting;

    bool manualOn{false};  // 가상 조종(MANUAL_CONTROL) 송신 중
    ManualAxes manualAxes;
    bool paramWriteActive{false};  // 파라미터를 쓰는 중 (FC 재부팅 차단)
};

class CommandPolicy {
public:
    // Stabilized 에서 ARM 할 때 스로틀이 이보다 크면 막는다 (켜자마자 추력이 나지 않게)
    static constexpr std::int16_t kArmThrottleMax = 50;

    // Altitude, Position 에서는 스로틀이 가운데(50%)보다 높으면 PX4 가 ARM 을 거부한다. 통제기가
    // 먼저 막고 이유를 보인다
    static constexpr std::int16_t kArmThrottleCenter = 500;

    // SCAN_START 매개변수 범위 (icd 4.2)
    static constexpr float kScanRangeMaxRad = 6.2831855F;  // 2π
    static constexpr float kScanRateMaxRadS = 1.309F;      // 75 deg/s

    CommandPolicy(const VehicleState& state, const FlightConditions& conditions,
                  const CommandContext& context);

    std::optional<QString> refusal(Command command, const CommandArgs& args) const;

    // refusal 중 이 명령에만 해당하는 이유. 화면 전체에 이미 보이는 이유(대상과의 링크 끊김, 앞서
    // 보낸 명령의 응답 대기)로 막혔으면 nullopt
    std::optional<QString> specificRefusal(Command command, const CommandArgs& args) const;

    // PARAM_SET 을 보낼 수 없는 이유. 지상이고 시동이 꺼져 있고 추종, 스캔 중이 아닐 때만 (icd 5.2)
    std::optional<QString> paramWriteRefusal() const;

private:
    // 대상 링크가 끊겼거나 같은 대상, 같은 MAV_CMD 의 응답을 기다리는 중 (화면 전체에 보이는 이유)
    std::optional<QString> sharedRefusal(Command command) const;
    std::optional<QString> authorityRefusal(Command command) const;
    std::optional<QString> overrideRefusal(Command command) const;
    std::optional<QString> commandRefusal(Command command, const CommandArgs& args) const;
    std::optional<QString> setpointRefusal() const;
    std::optional<QString> groundRefusal() const;
    std::optional<QString> manualInputRefusal() const;
    std::optional<QString> armRefusal() const;
    std::optional<QString> takeoffRefusal(const CommandArgs& args) const;
    std::optional<QString> manualModeRefusal(Command command) const;
    std::optional<QString> startRefusal() const;
    std::optional<QString> trackStartRefusal(const CommandArgs& args) const;
    std::optional<QString> scanRefusal(const CommandArgs& args) const;
    std::optional<QString> endRcOverrideRefusal() const;

    const VehicleState& state_;
    const FlightConditions& conditions_;
    const CommandContext& context_;
};

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_COMMAND_POLICY_HPP_
