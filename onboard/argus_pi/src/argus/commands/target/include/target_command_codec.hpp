// GCS/운용 도구와 TargetSelector 사이의 command/request·result v1 계약이다.
#pragma once

#include <string>
#include <string_view>

#include "argus/control/target/include/target_selector.hpp"

namespace follow {

struct TargetCommandDecodeResult {
    bool valid{false};
    TargetCommand command;
    std::string error;
};

class TargetCommandCodec final {
public:
    static std::string encode_request(const TargetCommand& command);
    static TargetCommandDecodeResult decode_request(std::string_view packed);
    static std::string encode_result(const TargetCommandResult& result);
    static bool decode_result(std::string_view packed, TargetCommandResult& result,
                              std::string& error);
};

}  // namespace follow
