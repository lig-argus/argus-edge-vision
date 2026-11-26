// 명령을 COMMAND_LONG 의 값(받을 대상, MAV_CMD, param1~7)으로 만든다 (icd 4.1~4.4, 부록 A).
// 쓰지 않는 param 은 FC 로 보내는 표준 명령이면 0, mavlink_gateway 로 보내는 명령이면 NaN 이다
// (icd 4.1). 보내기와 재전송은 CommandSender 가 한다
#ifndef ARGUS_GCS_LINK_COMMAND_ENCODER_HPP_
#define ARGUS_GCS_LINK_COMMAND_ENCODER_HPP_

#include <QString>
#include <array>
#include <cstdint>
#include <vector>

#include "argus/gcs/link/command.hpp"
#include "argus/gcs/link/command_traits.hpp"

namespace argus::gcs::link {

// 명령을 만드는 데 필요한 통제기와 기체의 값
struct EncodeInputs {
    // mavlink_gateway 로 보내는 명령의 param7 (icd 4.1). CommandSender::takeNumber() 의 값
    std::uint32_t commandNumber{0};

    float altAmslM{kUnknown};  // 지금 해발 고도. TAKEOFF 의 p7
    bool horizontalPositionKnown{false};
    std::uint8_t gcsSysid{kGcsSysidDefault};  // 제어권 요청의 p4
};

struct EncodedCommand {
    CommandTarget target{CommandTarget::Fc};
    std::uint16_t mavCmd{0};
    std::array<float, 7> params{};
    std::vector<QString> notes;  // 보내기 전에 운용자에게 알릴 것 (기록 창)
};

// 명령의 MAV_CMD. 같은 대상의 같은 MAV_CMD 는 응답을 기다리는 동안 하나만 보낸다 (icd 4.1)
std::uint16_t mavCmdOf(Command command);

EncodedCommand encodeCommand(Command command, const CommandArgs& args, const EncodeInputs& in);

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_COMMAND_ENCODER_HPP_
