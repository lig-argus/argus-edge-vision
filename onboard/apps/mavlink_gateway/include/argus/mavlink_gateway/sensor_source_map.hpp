// 카메라 식별 문자열(source)과 ARGUS_SENSOR 의 대응, 센서별 표적 지정(FOLLOWABLE) 허용
// 규약: docs/spec/process_interfaces.md 3장 (source 와 ARGUS_SENSOR 의 대응은 mavlink_gateway
// 설정),
//      common/mavlink/message_definitions/argus.xml (ARGUS_SENSOR_FLAGS_FOLLOWABLE)
// 사용: ArgusSensorStatusReporter
#ifndef ARGUS_ONBOARD_SENSOR_SOURCE_MAP_HPP_
#define ARGUS_ONBOARD_SENSOR_SOURCE_MAP_HPP_

#include <string>
#include <utility>
#include <vector>

#include "argus/mavlink.h"

namespace argus::mavlink_gateway {

struct SensorSource {
    std::string source;  // RPi 내부 메시지의 source (예: camera-ir)
    ARGUS_SENSOR sensor = ARGUS_SENSOR_UNKNOWN;
    bool followable = false;  // 이 센서의 객체를 추종 표적으로 지정할 수 있는지
};

// 설정 파일을 읽기 전까지는 default_map() 을 사용한다 (다른 설정과 같이 코드 기본값)
class SensorSourceMap {
public:
    explicit SensorSourceMap(std::vector<SensorSource> entries) : entries_(std::move(entries)) {}

    // camera-ir → IR, camera-eo → EO, 둘 다 표적 지정 허용 (docs/spec/video.md 의 source 예와 같음)
    static SensorSourceMap default_map() {
        return SensorSourceMap(
            {{"camera-ir", ARGUS_SENSOR_IR, true}, {"camera-eo", ARGUS_SENSOR_EO, true}});
    }

    const std::vector<SensorSource>& entries() const { return entries_; }

private:
    std::vector<SensorSource> entries_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_SENSOR_SOURCE_MAP_HPP_
