// STATUSTEXT 조각 모으기 (icd 2장). text 는 50 바이트이고 끝에 NUL 이 없을 수 있다. 50 자를 넘는
// 문구는 같은 id 에 chunk_seq 0, 1, ... 로 나뉘어 오고, NUL 이 있는(50 자 미만) 조각이 마지막이다.
// 마지막 조각이 오거나 kTimeoutNs 동안 다음 조각이 없으면 한 줄로 낸다
#ifndef ARGUS_GCS_LINK_STATUS_TEXT_HPP_
#define ARGUS_GCS_LINK_STATUS_TEXT_HPP_

#include <QByteArray>
#include <QString>
#include <cstdint>
#include <optional>
#include <vector>

#include "argus/gcs/link/link_protocol.hpp"
#include "argus/gcs/link/log_entry.hpp"
#include "argus/mavlink.h"

namespace argus::gcs::link {

// 다 모은 문구 한 줄과 그 심각도의 기록 수준
struct StatusTextLine {
    QString text;
    LogLevel level{LogLevel::Off};
};

// STATUSTEXT 심각도(MAV_SEVERITY 0~7) → 기록 수준. EMERGENCY, ALERT, CRITICAL 은 Critical,
// ERROR 는 Serious, WARNING 은 Caution, NOTICE, INFO, DEBUG 는 Off
LogLevel statusTextLevel(std::uint8_t severity);

// 보낸 곳(FC, mavlink_gateway)마다 하나씩 둔다. 조각 순서는 보낸 곳 안에서만 의미가 있다
class StatusTextAssembler {
public:
    static constexpr std::int64_t kTimeoutNs = kNsPerSecond;

    // 조각 하나를 받는다. 다 모은 줄을 낸 순서대로 돌려준다 (앞서 모으던 줄이 끊긴 것 포함).
    // nowNs 는 monotonic ns
    std::vector<StatusTextLine> add(const mavlink_statustext_t& text, std::int64_t nowNs);

    // 모으던 줄의 다음 조각이 kTimeoutNs 넘게 없으면 그 줄을 낸다. 마지막 조각이 꼭 50 자였거나
    // 조각을 잃은 경우다
    std::optional<StatusTextLine> expire(std::int64_t nowNs);

private:
    struct Pending {
        std::uint16_t id{0};
        std::uint8_t nextSeq{0};
        QByteArray text;
        std::int64_t lastNs{0};
        std::uint8_t severity{0};  // MAV_SEVERITY (첫 조각의 값)
    };

    std::optional<StatusTextLine> flush();

    std::optional<Pending> pending_;
};

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_STATUS_TEXT_HPP_
