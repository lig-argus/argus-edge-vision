// vehicle_api_bus 의 구현 파일들이 함께 쓰는 로그 함수와 발행 결과 기록. 라이브러리 밖에서는
// include 하지 않는다
#ifndef ARGUS_VEHICLE_API_BUS_SRC_DETAIL_HPP_
#define ARGUS_VEHICLE_API_BUS_SRC_DETAIL_HPP_

#include <iostream>
#include <string>
#include <string_view>

namespace argus::vehicle_api_bus::detail {

inline void log_error(const std::string& text) {
    std::cerr << "[vehicle_api_bus] " << text << '\n';
}

// 토픽의 발행 결과를 받아, 실패가 시작될 때와 복구될 때 한 번씩 로그에 남긴다. 주기적으로 발행하는
// 토픽이 실패할 때마다 로그를 남기지 않기 위한 것이다. failing 은 그 토픽의 직전 결과(호출자가
// 보관)
inline void record_publish_result(std::string_view topic, bool published, bool& failing) {
    if (!published && !failing) {
        log_error(std::string(topic) + " 발행 실패 시작");
    } else if (published && failing) {
        log_error(std::string(topic) + " 발행 복구");
    }
    failing = !published;
}

}  // namespace argus::vehicle_api_bus::detail

#endif  // ARGUS_VEHICLE_API_BUS_SRC_DETAIL_HPP_
