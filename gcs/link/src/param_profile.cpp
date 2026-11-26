#include "argus/gcs/link/param_profile.hpp"

#include <QStringList>
#include <algorithm>
#include <cmath>

#include "argus/mavlink.h"

namespace argus::gcs::link {

namespace {

// 한 줄의 칸 수: 기체ID, 컴포넌트ID, 이름, 값, 형식
constexpr qsizetype kFieldCount = 5;
constexpr qsizetype kNameField = 2;
constexpr qsizetype kValueField = 3;
constexpr qsizetype kTypeField = 4;

// 주석과 빈 줄이 아닌 한 줄을 값으로 바꾼다. 형식이 맞지 않으면 nullopt
std::optional<ParamValue> parseLine(const QString& line) {
    const QStringList fields = line.split(QLatin1Char('\t'));
    if (fields.size() != kFieldCount) {
        return std::nullopt;
    }
    bool valueOk = false;
    bool typeOk = false;
    const double value = fields.at(kValueField).toDouble(&valueOk);
    const std::int32_t type = fields.at(kTypeField).toInt(&typeOk);
    const QString& name = fields.at(kNameField);
    const bool knownType = type == MAV_PARAM_TYPE_INT32 || type == MAV_PARAM_TYPE_REAL32;
    if (!valueOk || !typeOk || !knownType || name.isEmpty() || name.size() > kParamIdMaxLength) {
        return std::nullopt;
    }
    return ParamValue{name, static_cast<std::uint8_t>(type), value};
}

}  // namespace

bool ParamValue::operator==(const ParamValue& other) const {
    return name == other.name && type == other.type && sameParamValue(type, value, other.value);
}

bool sameParamValue(std::uint8_t type, double a, double b) {
    if (type == MAV_PARAM_TYPE_INT32) {
        return std::llround(a) == std::llround(b);
    }
    // REAL32: 파일의 0.075 와 FC 가 돌려준 float 0.075000003 을 같게 본다
    return static_cast<float>(a) == static_cast<float>(b);
}

std::optional<ParamProfile> parseParamProfile(FlightEnv env, const QString& text, QString* error) {
    ParamProfile profile{env, {}};
    const QStringList lines = text.split(QLatin1Char('\n'));
    for (qsizetype i = 0; i < lines.size(); ++i) {
        const QString line = lines.at(i).trimmed();
        if (line.isEmpty() || line.startsWith(QLatin1Char('#'))) {
            continue;
        }
        const std::optional<ParamValue> entry = parseLine(line);
        if (!entry) {
            if (error != nullptr) {
                *error = QStringLiteral("%1번째 줄 형식 오류: %2").arg(i + 1).arg(line);
            }
            return std::nullopt;
        }
        std::vector<ParamValue>& values = profile.values;
        const auto same = std::find_if(values.begin(), values.end(), [&entry](const ParamValue& v) {
            return v.name == entry->name;
        });
        if (same != values.end()) {
            *same = *entry;  // 같은 이름이 다시 나오면 뒤의 값을 쓴다 (QGC 의 불러오기와 같다)
        } else {
            values.push_back(*entry);
        }
    }
    return profile;
}

std::optional<ParamProfile> parseParamProfile(FlightEnv env, const QByteArray& utf8,
                                              QString* error) {
    return parseParamProfile(env, QString::fromUtf8(utf8), error);
}

QString flightEnvName(FlightEnv env) {
    return env == FlightEnv::Indoor ? QStringLiteral("실내") : QStringLiteral("실외");
}

}  // namespace argus::gcs::link
