// 추종과 스캔은 Offboard 모드로 동작한다. 운용자 명령 없이 Offboard 를 벗어나거나(조종사 개입,
// failsafe, mavlink_gateway 의 추종 종료) DISARM 된 채 Offboard 로 남으면 경고한다.
// 모드 표시는 화면에서 작게 보이므로 운용자가 놓치지 않게 기록으로 알린다.
#ifndef ARGUS_GCS_LINK_MODE_CHANGE_WATCH_HPP_
#define ARGUS_GCS_LINK_MODE_CHANGE_WATCH_HPP_

#include <cstdint>
#include <functional>
#include <optional>

#include "argus/gcs/link/log_entry.hpp"
#include "argus/gcs/link/vehicle_state.hpp"

namespace argus::gcs::link {

class ModeChangeWatch {
public:
    using LogHandler = std::function<void(const LogEntry&)>;

    // 운용자가 모드를 바꾸는 명령을 보낸 뒤 이 시간 안의 Offboard 이탈은 그 명령 때문으로 본다
    static constexpr std::int64_t kOperatorWindowNs = 5 * kNsPerSecond;

    explicit ModeChangeWatch(LogHandler log) : log_(std::move(log)) {}

    void noteOperatorModeCommand(std::int64_t nowNs) { lastOperatorCommandNs_ = nowNs; }

    // FC HEARTBEAT 를 해석하기 전과 뒤의 상태로 판단한다
    void check(const FcState& before, const FcState& after, std::int64_t nowNs) const;

private:
    LogHandler log_;
    std::optional<std::int64_t> lastOperatorCommandNs_;
};

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_MODE_CHANGE_WATCH_HPP_
