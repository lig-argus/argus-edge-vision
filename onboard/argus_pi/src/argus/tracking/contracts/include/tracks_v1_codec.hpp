// TracksV1 MessagePack 변환 경계를 선언한다.
#pragma once

#include <string>
#include <string_view>

#include "argus/tracking/contracts/include/tracks_v1.hpp"

namespace follow {

struct TracksDecodeResult {
    bool valid{false};
    TracksV1 message;
    std::string error;
};

class TracksV1Codec final {
public:
    static constexpr std::size_t kMaxMessageBytes = 1024 * 1024;
    static std::string encode(const TracksV1& message);
    static TracksDecodeResult decode(std::string_view packed);
};

}  // namespace follow
