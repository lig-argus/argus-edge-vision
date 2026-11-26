// 비행 환경(실내, 실외) 파라미터 묶음. PX4 파라미터 파일(QGC 형식)의 글을 받아 값 목록으로 바꾼다.
// 파일을 읽는 일은 부르는 쪽이 한다 (이 모듈은 글만 다룬다)
#ifndef ARGUS_GCS_LINK_PARAM_PROFILE_HPP_
#define ARGUS_GCS_LINK_PARAM_PROFILE_HPP_

#include <QByteArray>
#include <QString>
#include <cstdint>
#include <optional>
#include <vector>

namespace argus::gcs::link {

// MAVLink param_id 의 최대 길이 (PARAM_SET, PARAM_VALUE, PARAM_REQUEST_READ)
inline constexpr qsizetype kParamIdMaxLength = 16;

// FC 파라미터 하나. PX4 는 INT32 와 REAL32 만 쓴다
struct ParamValue {
    QString name;
    std::uint8_t type{0};  // MAV_PARAM_TYPE_INT32(6) 또는 MAV_PARAM_TYPE_REAL32(9)
    double value{0.0};     // INT32 도 이 칸에 둔다 (2^31 안의 정수는 double 에서 정확하다)

    bool operator==(const ParamValue& other) const;
};

// 비행 환경. 실내는 GPS 없이 optical flow 와 거리 센서로, 실외는 GPS 로 수평 위치를 추정한다
enum class FlightEnv : std::uint8_t { Indoor, Outdoor };

struct ParamProfile {
    FlightEnv env{FlightEnv::Indoor};
    std::vector<ParamValue> values;  // 글의 순서. 같은 이름은 한 번만 (뒤에 나온 값)
};

// QGC 파라미터 파일의 글을 읽는다. 줄 형식은 "기체ID 컴포넌트ID 이름 값 형식"(탭 구분)이고 '#' 로
// 시작하는 줄과 빈 줄은 건너뛴다. 형식이 INT32, REAL32 가 아니거나 칸이 맞지 않는 줄이 있으면 그
// 줄 번호와 이유를 error 에 넣고 nullopt 를 돌려준다 (일부만 쓰면 두 환경이 섞인 값이 된다)
std::optional<ParamProfile> parseParamProfile(FlightEnv env, const QString& text, QString* error);

// UTF-8 글
std::optional<ParamProfile> parseParamProfile(FlightEnv env, const QByteArray& utf8,
                                              QString* error);

// 같은 값인가. INT32 는 정확히, REAL32 는 float 로 보낸 값이므로 float 정밀도 안에서 비교한다
bool sameParamValue(std::uint8_t type, double a, double b);

// 화면과 기록에 쓰는 이름 ("실내", "실외")
QString flightEnvName(FlightEnv env);

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_PARAM_PROFILE_HPP_
