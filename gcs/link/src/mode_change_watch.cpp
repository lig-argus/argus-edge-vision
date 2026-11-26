#include "argus/gcs/link/mode_change_watch.hpp"

#include "argus/gcs/link/px4_mode.hpp"

namespace argus::gcs::link {

void ModeChangeWatch::check(const FcState& before, const FcState& after, std::int64_t nowNs) const {
    const bool wasOffboard = before.mode == FlightMode::Offboard;
    const bool isOffboard = after.mode == FlightMode::Offboard;
    const bool operatorCommand =
        lastOperatorCommandNs_ && nowNs - *lastOperatorCommandNs_ <= kOperatorWindowNs;
    if (before.armed && wasOffboard && after.armed && !isOffboard && !operatorCommand) {
        log_(gcsLog(LogKind::Fc, LogLevel::Serious,
                    QStringLiteral("운용자 명령 없이 FC 모드 변경됨: Offboard → %1 (조종사 개입, "
                                   "failsafe, 임무 컴퓨터의 추종 종료 중 하나)")
                        .arg(flightModeName(after.mode))));
    }
    const bool wasDisarmedInOffboard = !before.armed && wasOffboard;
    if (!after.armed && isOffboard && !wasDisarmedInOffboard) {
        log_(gcsLog(
            LogKind::Fc, LogLevel::Serious,
            QStringLiteral("추종 비정상 종료: FC 가 Offboard 모드로 남음. ARM 전 [HOLD] 필요")));
    }
}

}  // namespace argus::gcs::link
