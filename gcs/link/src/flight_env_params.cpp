#include "argus/gcs/link/flight_env_params.hpp"

#include <QByteArray>
#include <QStringList>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>
#include <utility>

namespace argus::gcs::link {

namespace {

// PARAM_REQUEST_READ 에서 이름으로 묻는다는 뜻의 param_index
constexpr std::int16_t kParamIndexByName = -1;

// PX4 는 정수 파라미터를 float 칸에 비트 그대로 싣는다
// (MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE)
float encodeParam(const ParamValue& value) {
    if (value.type != MAV_PARAM_TYPE_INT32) {
        return static_cast<float>(value.value);
    }
    const auto raw = static_cast<std::int32_t>(std::llround(value.value));
    float encoded = 0.0F;
    std::memcpy(&encoded, &raw, sizeof(encoded));
    return encoded;
}

double decodeParam(float encoded, std::uint8_t type) {
    if (type == MAV_PARAM_TYPE_REAL32) {
        return encoded;
    }
    std::int32_t raw = 0;
    std::memcpy(&raw, &encoded, sizeof(raw));
    return raw;
}

bool linkUp(LinkState link) { return link == LinkState::Ok || link == LinkState::Delayed; }

// mavlink_gateway 가 추종 또는 스캔을 하고 있다 (icd 부록 B). 이 동안의 PARAM_SET 은 전달되지
// 않는다
bool followingOrScanning(const GatewayStatus& gateway) {
    if (!gateway.received) {
        return false;
    }
    switch (gateway.missionState) {
        case MissionState::Preparing:
        case MissionState::FollowActive:
        case MissionState::HoldTransition:
        case MissionState::ScanActive:
            return true;
        case MissionState::Idle:
        case MissionState::HoldWait:
        case MissionState::Manual:
        case MissionState::Fault:
            return false;
    }
    return false;
}

// 생성된 구조체의 param_id. 16 자이면 끝에 NUL 이 없다
QString paramName(const mavlink_param_value_t& param) {
    const auto* end = std::find(std::begin(param.param_id), std::end(param.param_id), '\0');
    return QString::fromLatin1(param.param_id, std::distance(std::begin(param.param_id), end));
}

}  // namespace

FlightEnvParams::FlightEnvParams(Send send, Log log)
    : send_(std::move(send)), log_(std::move(log)) {}

void FlightEnvParams::setProfiles(std::vector<ParamProfile> profiles) {
    profiles_ = std::move(profiles);
}

bool FlightEnvParams::hasProfile(FlightEnv env) const {
    return std::any_of(profiles_.begin(), profiles_.end(),
                       [env](const ParamProfile& p) { return p.env == env && !p.values.empty(); });
}

std::vector<QString> FlightEnvParams::names() const {
    std::vector<QString> names;
    for (const ParamProfile& profile : profiles_) {
        for (const ParamValue& value : profile.values) {
            if (std::find(names.begin(), names.end(), value.name) == names.end()) {
                names.push_back(value.name);
            }
        }
    }
    return names;
}

std::vector<QString> FlightEnvParams::missingNames() const {
    std::vector<QString> missing;
    for (const QString& name : names()) {
        if (fcValues_.find(name) == fcValues_.end()) {
            missing.push_back(name);
        }
    }
    return missing;
}

void FlightEnvParams::queueReads(std::int64_t nowNs) {
    const std::vector<QString> all = names();
    reads_.assign(all.begin(), all.end());
    readRounds_ = reads_.empty() ? 0 : 1;
    lastReadNs_ = nowNs;
}

void FlightEnvParams::check(std::int64_t nowNs, LinkState fcLink) {
    if (!linkUp(fcLink)) {
        reads_.clear();
    } else {
        // 읽기는 check 마다 하나씩 보낸다 (한꺼번에 보내면 FC 의 응답이 몰린다)
        sendNextRead();
        requeueMissing(nowNs);
    }
    checkWrite(nowNs);
}

void FlightEnvParams::sendNextRead() {
    if (reads_.empty()) {
        return;
    }
    const QByteArray id = reads_.front().toLatin1();
    reads_.pop_front();
    mavlink_message_t message{};
    mavlink_msg_param_request_read_pack(sysid_, kGcsCompid, &message, kVehicleSysid, kFcCompid,
                                        id.constData(), kParamIndexByName);
    send_(message);
}

// 응답이 빠진 이름은 다시 묻는다. 횟수를 넘으면 더 묻지 않고 화면에 "미확인" 을 보인다
void FlightEnvParams::requeueMissing(std::int64_t nowNs) {
    if (!reads_.empty() || readRounds_ <= 0 || readRounds_ >= kMaxSends ||
        nowNs - lastReadNs_ < kReadRetryNs) {
        return;
    }
    const std::vector<QString> missing = missingNames();
    reads_.assign(missing.begin(), missing.end());
    ++readRounds_;
    lastReadNs_ = nowNs;
}

void FlightEnvParams::checkWrite(std::int64_t nowNs) {
    if (!write_ || nowNs - write_->sentNs <= kWriteRetryNs) {
        return;
    }
    const Write& write = *write_;
    const ParamValue& value = write.values.at(write.index);
    if (write.sends < kMaxSends) {
        sendSet(value, nowNs);
        return;
    }
    const auto got = fcValues_.find(value.name);
    const QString seen = got == fcValues_.end() ? QStringLiteral("응답 없음")
                                                : QStringLiteral("FC 값 %1").arg(got->second.value);
    log_(gcsLog(LogKind::Reply, LogLevel::Serious,
                QStringLiteral("%1 모드 적용 실패: %2 = %3 미확인 (%4, %5회 전송). 앞의 %6개만 "
                               "변경됨: [%1] 재시도 필요")
                    .arg(flightEnvName(write.env), value.name)
                    .arg(value.value)
                    .arg(seen)
                    .arg(kMaxSends)
                    .arg(write.index)));
    write_.reset();
}

void FlightEnvParams::sendSet(const ParamValue& value, std::int64_t nowNs) {
    const QByteArray id = value.name.toLatin1();
    mavlink_message_t message{};
    mavlink_msg_param_set_pack(sysid_, kGcsCompid, &message, kVehicleSysid, kFcCompid,
                               id.constData(), encodeParam(value), value.type);
    send_(message);
    if (write_) {
        write_->sentNs = nowNs;
        ++write_->sends;
    }
}

void FlightEnvParams::onMessage(const mavlink_message_t& message, std::int64_t nowNs) {
    if (message.msgid != MAVLINK_MSG_ID_PARAM_VALUE || message.sysid != kVehicleSysid ||
        message.compid != kFcCompid) {
        return;
    }
    mavlink_param_value_t param{};
    mavlink_msg_param_value_decode(&message, &param);
    const QString name = paramName(param);
    const ParamValue value{name, param.param_type,
                           decodeParam(param.param_value, param.param_type)};
    fcValues_[name] = value;
    confirmWrite(value, nowNs);
}

void FlightEnvParams::confirmWrite(const ParamValue& received, std::int64_t nowNs) {
    if (!write_) {
        return;
    }
    Write& write = *write_;
    const ParamValue& wanted = write.values.at(write.index);
    // 이름이 같아도 값이 다르면(앞서 읽은 값의 늦은 응답 등) 확인으로 치지 않고 재전송을 기다린다
    if (received.name != wanted.name ||
        !sameParamValue(wanted.type, received.value, wanted.value)) {
        return;
    }
    ++write.index;
    write.sends = 0;
    if (write.index < write.values.size()) {
        sendSet(write.values.at(write.index), nowNs);
        return;
    }
    rebootNeeded_ = true;
    log_(gcsLog(LogKind::Reply, QStringLiteral("%1 모드 파라미터 %2개 적용. [FC 재부팅]해야 반영됨")
                                    .arg(flightEnvName(write.env))
                                    .arg(write.values.size())));
    write_.reset();
}

std::optional<QString> FlightEnvParams::writeRefusal(FlightEnv env,
                                                     const VehicleState& state) const {
    if (!linkUp(state.fcLink.state)) {
        return QStringLiteral("FC 미연결");
    }
    if (!hasProfile(env)) {
        return QStringLiteral("%1 파라미터 묶음 없음").arg(flightEnvName(env));
    }
    if (state.fc.armed) {
        return QStringLiteral("ARM 상태에서 변경 불가: 착륙 후 DISARM 필요");
    }
    if (state.fc.landedState == MAV_LANDED_STATE_UNDEFINED) {
        return QStringLiteral("착륙 상태를 알 수 없어 변경 불가");
    }
    if (state.fc.landedState != MAV_LANDED_STATE_ON_GROUND) {
        return QStringLiteral("비행 중 변경 불가: 착륙 후 가능");
    }
    if (followingOrScanning(state.gateway)) {
        return QStringLiteral("추종, 스캔 중 변경 불가: [중단] 후 가능");
    }
    if (const std::optional<FlightEnv> active = writing()) {
        return QStringLiteral("%1 모드 파라미터 기록 중").arg(flightEnvName(*active));
    }
    return std::nullopt;
}

std::optional<QString> FlightEnvParams::apply(FlightEnv env, const VehicleState& state,
                                              std::int64_t nowNs) {
    if (std::optional<QString> why = writeRefusal(env, state)) {
        log_(gcsLog(LogKind::Command, LogLevel::Serious,
                    QStringLiteral("%1 모드 전환 안 함. %2").arg(flightEnvName(env), *why)));
        return why;
    }
    const auto profile =
        std::find_if(profiles_.begin(), profiles_.end(),
                     [env](const ParamProfile& p) { return p.env == env && !p.values.empty(); });
    write_ = Write{env, profile->values, 0, 0, 0};
    log_(gcsLog(LogKind::Command, QStringLiteral("%1 모드 파라미터 %2개 쓰기")
                                      .arg(flightEnvName(env))
                                      .arg(profile->values.size())));
    sendSet(profile->values.front(), nowNs);
    return std::nullopt;
}

void FlightEnvParams::onRebooting(std::int64_t nowNs) {
    rebootNeeded_ = false;
    fcValues_.clear();
    // 재부팅이 짧아 링크가 두절로 판정되지 않아도 다시 묻기로 모두 읽는다
    reads_.clear();
    readRounds_ = 1;
    lastReadNs_ = nowNs;
}

bool FlightEnvParams::matches(const ParamProfile& profile) const {
    return !profile.values.empty() &&
           std::all_of(profile.values.begin(), profile.values.end(), [this](const ParamValue& v) {
               const auto got = fcValues_.find(v.name);
               return got != fcValues_.end() && sameParamValue(v.type, got->second.value, v.value);
           });
}

std::optional<FlightEnv> FlightEnvParams::current() const {
    for (const ParamProfile& profile : profiles_) {
        if (matches(profile)) {
            return profile.env;
        }
    }
    return std::nullopt;
}

QString FlightEnvParams::text(LinkState fcLink, std::int64_t nowNs) const {
    if (profiles_.empty()) {
        return QStringLiteral("미확인: 파라미터 묶음 없음");
    }
    if (const std::optional<FlightEnv> env = current()) {
        return flightEnvName(*env);
    }
    if (!linkUp(fcLink)) {
        return QStringLiteral("-");  // FC 연결 전, 두절: 통제기 공통의 빈 값
    }
    const std::vector<QString> missing = missingNames();
    if (missing.empty()) {
        return mixedText();
    }
    const bool reading =
        !reads_.empty() || readRounds_ < kMaxSends || nowNs - lastReadNs_ < kReadRetryNs;
    if (reading) {
        return QStringLiteral("읽기 중");
    }
    QStringList list;
    for (const QString& name : missing) {
        list << name;
    }
    return QStringLiteral("미확인: 응답 없음 %1").arg(list.join(QStringLiteral(", ")));
}

// 다 읽었는데 어느 묶음과도 같지 않다: 실내 묶음과 다른 이름을 보인다 (직접 바꾼 값을 찾게)
QString FlightEnvParams::mixedText() const {
    QStringList differ;
    for (const ParamProfile& profile : profiles_) {
        if (profile.env != FlightEnv::Indoor) {
            continue;
        }
        for (const ParamValue& v : profile.values) {
            const ParamValue& got = fcValues_.at(v.name);
            if (!sameParamValue(v.type, got.value, v.value)) {
                differ << QStringLiteral("%1=%2").arg(v.name).arg(got.value);
            }
        }
    }
    return QStringLiteral("섞임: 실내와 다른 값 %1").arg(differ.join(QStringLiteral(", ")));
}

std::optional<FlightEnv> FlightEnvParams::writing() const {
    return write_ ? std::optional<FlightEnv>(write_->env) : std::nullopt;
}

}  // namespace argus::gcs::link
