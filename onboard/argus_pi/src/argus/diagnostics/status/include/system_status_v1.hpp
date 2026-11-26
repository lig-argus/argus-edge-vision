// 제어 임무·선택 ID·FC 상태와 실제 송신 여부를 관찰용 버스로 공개한다.
#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace follow {

inline constexpr char kSystemStatusV1Schema[] = "argus.system.status.v1";
inline constexpr char kSystemStatusV2Schema[] = "argus.system.status.v2";

struct SystemStatusV1 {
    std::string schema{kSystemStatusV1Schema};
    std::string source;
    std::int64_t monotonic_ns{0};
    std::string mission;
    std::string reason;
    std::uint64_t command_epoch{0};
    std::optional<std::uint64_t> target_generation;
    std::optional<std::int64_t> target_track_id;
    std::string target_state{"LOST"};
    double age_since_real_s{0};
    bool connected{false};
    bool armed{false};
    bool in_air{false};
    bool offboard{false};
    bool hold{false};
    bool command_sent{false};
    bool offboard_start_requested{false};
    bool offboard_stop_requested{false};
    std::string fc_uid;
    bool identity_verified{false};
};

}  // namespace follow
