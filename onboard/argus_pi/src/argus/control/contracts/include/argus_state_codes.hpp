// PR27 숫자 코드와 내부 상태 이름의 명시적 변환. 내부 phase enum을 캐스팅하지 않는다.
#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string_view>
namespace follow {
inline std::optional<std::uint8_t> mission_state_code(std::string_view name) {
    constexpr std::array<std::string_view, 8> names{
        "IDLE",      "PREPARING",   "FOLLOW_ACTIVE", "HOLD_TRANSITION",
        "HOLD_WAIT", "SCAN_ACTIVE", "MANUAL",        "FAULT"};
    for (std::uint8_t i = 0; i < names.size(); ++i) {
        if (name == names[i]) {
            return i;
        }
    }
    return std::nullopt;
}
inline std::optional<std::uint8_t> target_state_code(std::string_view name) {
    constexpr std::array<std::string_view, 4> names{"NONE", "CONFIRMED", "PREDICTED", "LOST"};
    for (std::uint8_t i = 0; i < names.size(); ++i) {
        if (name == names[i]) {
            return i;
        }
    }
    return std::nullopt;
}
}  // namespace follow
