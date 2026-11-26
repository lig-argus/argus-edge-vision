// 명령마다 바뀌지 않는 속성을 한 표에 둔다: 화면 이름, 테스트 스크립트 이름, 받는 대상, 응답을
// 기다리지 않고 보내는지, 모드 변경, 스캔 표시의 후속 처리. 새 명령은 Command 에 하나, 이 표에 한
// 줄을 더하고, 명령마다 다른 판단(거부 command_policy, 값 만들기 command_encoder)만 그 자리에 쓴다
#ifndef ARGUS_GCS_LINK_COMMAND_TRAITS_HPP_
#define ARGUS_GCS_LINK_COMMAND_TRAITS_HPP_

#include <QString>
#include <array>
#include <cstdint>
#include <optional>

#include "argus/gcs/link/command.hpp"

namespace argus::gcs::link {

// 명령을 받는 구성 요소 (icd 4.1). FC 로 보내는 명령도 mavlink_gateway 가 검사한 뒤 전달한다
enum class CommandTarget : std::uint8_t {
    Fc,       // target_component 1 (icd 4.3)
    Gateway,  // target_component 195 (icd 4.2, 4.4)
};

struct CommandTraits {
    Command command;
    const char* name;  // 화면과 기록의 이름 (UTF-8). 비행 명령과 모드는 PX4, MAVLink 이름 그대로
    const char* scriptName;  // 테스트 스크립트에서 명령을 고르는 이름
    CommandTarget target;

    // 앞서 보낸 명령의 응답을 기다리지 않고 바로 보낸다. 착륙은 다른 명령의 처리를 기다리지 않고
    // 전달된다 (icd 5.2, 5.3)
    bool emergency;

    // 운용자의 모드 변경. 보낸 뒤 FC 가 OFFBOARD 를 벗어나도 운용자 명령 때문으로 보고 경고하지
    // 않는다
    bool changesMode;

    // 보내면 끝난 스캔 결과 표시를 바로 닫는다. 비행을 바꾸는 명령이라 지난 결과가 더 쓰이지 않는다
    bool dismissesScanResult;

    // 보내면 스캔 요청 표시를 지운다. 추종 중의 표적 찾기는 스캔이 아니므로 스캔으로 보이지 않게
    // 한다
    bool clearsScanRequest;
};

// Command 순서와 같은 표
const std::array<CommandTraits, kCommandCount>& allCommandTraits();
const CommandTraits& commandTraits(Command command);
std::optional<Command> commandFromScriptName(const QString& name);
QString commandName(Command command);

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_COMMAND_TRAITS_HPP_
