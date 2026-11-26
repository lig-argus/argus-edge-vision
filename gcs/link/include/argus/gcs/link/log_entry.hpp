// 운용 기록 한 줄. 화면의 운용 로그, 알림, 녹화의 사건 기록이 쓴다
#ifndef ARGUS_GCS_LINK_LOG_ENTRY_HPP_
#define ARGUS_GCS_LINK_LOG_ENTRY_HPP_

#include <QString>
#include <cstdint>

namespace argus::gcs::link {

// 무엇에 대한 기록인가
enum class LogKind : std::uint8_t {
    Command,  // 통제기가 보낸 명령 (거부되어 보내지 않은 명령 포함)
    Reply,    // 명령 응답 (수락, 진행 중, 거부, 응답 없음)
    Fc,       // FC 문구, 모드 변경, FC 링크
    Gateway,  // mavlink_gateway 문구, 링크, 영상 접속 정보
    Gcs,      // 통제기 자체: 가상 조종, 녹화, 설정
};

// 얼마나 심각한가 (낮은 것부터)
enum class LogLevel : std::uint8_t { Off, Standby, Normal, Caution, Serious, Critical };

// 문구를 누가 썼는가. 기체가 보낸 문구(STATUSTEXT)는 심각도가 높아도 알림으로 띄우지 않고 로그에만
// 남긴다. 알림은 통제기가 판단한 심각 이상의 기록만 띄운다
enum class LogOrigin : std::uint8_t { Gcs, Fc, Gateway };

struct LogEntry {
    LogKind kind{LogKind::Gcs};
    LogLevel level{LogLevel::Off};
    LogOrigin origin{LogOrigin::Gcs};
    QString text;
};

// 종류에 따른 기본 수준: 명령은 대기, 응답은 정상, 그 밖은 꺼짐
constexpr LogLevel defaultLevel(LogKind kind) {
    switch (kind) {
        case LogKind::Command:
            return LogLevel::Standby;
        case LogKind::Reply:
            return LogLevel::Normal;
        case LogKind::Fc:
        case LogKind::Gateway:
        case LogKind::Gcs:
            return LogLevel::Off;
    }
    return LogLevel::Off;
}

// 통제기가 판단한 기록 (기본 수준)
inline LogEntry gcsLog(LogKind kind, const QString& text) {
    return LogEntry{kind, defaultLevel(kind), LogOrigin::Gcs, text};
}

// 통제기가 판단한 기록 (수준 지정)
inline LogEntry gcsLog(LogKind kind, LogLevel level, const QString& text) {
    return LogEntry{kind, level, LogOrigin::Gcs, text};
}

// 알림으로도 띄울 기록인가
constexpr bool needsNotification(const LogEntry& entry) {
    return entry.origin == LogOrigin::Gcs &&
           (entry.level == LogLevel::Serious || entry.level == LogLevel::Critical);
}

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_LOG_ENTRY_HPP_
