// system/status v1 MessagePack 계약의 엄격한 인코더/디코더다.
#pragma once

#include <cstddef>
#include <string>
#include <string_view>

#include "argus/diagnostics/status/include/system_status_v1.hpp"

namespace follow {

struct SystemStatusDecodeResult {
    bool valid{false};
    SystemStatusV1 message;
    std::string error;
};

class SystemStatusV1Codec final {
public:
    static std::string encode(const SystemStatusV1&);
    static SystemStatusDecodeResult decode(std::string_view);
    static constexpr std::size_t kMaxMessageBytes = 4096;
};

}  // namespace follow
