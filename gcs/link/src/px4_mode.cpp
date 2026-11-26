#include "argus/gcs/link/px4_mode.hpp"

namespace argus::gcs::link {

namespace {

// main mode (icd 부록 A)
constexpr std::uint32_t kMainManual = 1;
constexpr std::uint32_t kMainAltitude = 2;
constexpr std::uint32_t kMainPosition = 3;
constexpr std::uint32_t kMainAuto = 4;
constexpr std::uint32_t kMainAcro = 5;
constexpr std::uint32_t kMainOffboard = 6;
constexpr std::uint32_t kMainStabilized = 7;

// AUTO 의 sub mode (icd 부록 A)
constexpr std::uint32_t kAutoReady = 1;
constexpr std::uint32_t kAutoTakeoff = 2;
constexpr std::uint32_t kAutoLoiter = 3;
constexpr std::uint32_t kAutoMission = 4;
constexpr std::uint32_t kAutoRtl = 5;
constexpr std::uint32_t kAutoLand = 6;
constexpr std::uint32_t kAutoFollowTarget = 8;
constexpr std::uint32_t kAutoPrecisionLand = 9;

constexpr std::uint32_t kModeMask = 0xFFU;
constexpr std::uint32_t kMainShift = 16U;
constexpr std::uint32_t kSubShift = 24U;

FlightMode autoMode(std::uint32_t sub) {
    switch (sub) {
        case kAutoReady:
            return FlightMode::Ready;
        case kAutoTakeoff:
            return FlightMode::Takeoff;
        case kAutoLoiter:
            return FlightMode::Hold;
        case kAutoMission:
            return FlightMode::Mission;
        case kAutoRtl:
            return FlightMode::Return;
        case kAutoLand:
            return FlightMode::Land;
        case kAutoFollowTarget:
            return FlightMode::FollowTarget;
        case kAutoPrecisionLand:
            return FlightMode::PrecisionLand;
        default:
            return FlightMode::Unknown;
    }
}

}  // namespace

FlightMode px4FlightMode(std::uint32_t customMode) {
    switch ((customMode >> kMainShift) & kModeMask) {
        case kMainManual:
            return FlightMode::Manual;
        case kMainAltitude:
            return FlightMode::Altitude;
        case kMainPosition:
            return FlightMode::Position;
        case kMainAuto:
            return autoMode((customMode >> kSubShift) & kModeMask);
        case kMainAcro:
            return FlightMode::Acro;
        case kMainOffboard:
            return FlightMode::Offboard;
        case kMainStabilized:
            return FlightMode::Stabilized;
        default:
            return FlightMode::Unknown;
    }
}

QString flightModeName(FlightMode mode) {
    switch (mode) {
        case FlightMode::Unknown:
            return QStringLiteral("Unknown");
        case FlightMode::Manual:
            return QStringLiteral("Manual");
        case FlightMode::Altitude:
            return QStringLiteral("Altitude");
        case FlightMode::Position:
            return QStringLiteral("Position");
        case FlightMode::Acro:
            return QStringLiteral("Acro");
        case FlightMode::Offboard:
            return QStringLiteral("Offboard");
        case FlightMode::Stabilized:
            return QStringLiteral("Stabilized");
        case FlightMode::Ready:
            return QStringLiteral("Ready");
        case FlightMode::Takeoff:
            return QStringLiteral("Takeoff");
        case FlightMode::Hold:
            return QStringLiteral("Hold");
        case FlightMode::Mission:
            return QStringLiteral("Mission");
        case FlightMode::Return:
            return QStringLiteral("Return");
        case FlightMode::Land:
            return QStringLiteral("Land");
        case FlightMode::FollowTarget:
            return QStringLiteral("Follow Target");
        case FlightMode::PrecisionLand:
            return QStringLiteral("Precision Land");
    }
    return QStringLiteral("Unknown");
}

}  // namespace argus::gcs::link
