// bus_zmq 의 구현 파일들이 함께 쓰는 상수와 로그 함수. 라이브러리 밖에서는 include 하지 않는다
#ifndef ARGUS_BUS_ZMQ_SRC_DETAIL_HPP_
#define ARGUS_BUS_ZMQ_SRC_DETAIL_HPP_

#include <cstdint>
#include <iostream>
#include <string>

namespace argus::bus_zmq::detail {

inline constexpr std::int64_t kNsPerMs = 1'000'000;

inline void log_error(const std::string& text) { std::cerr << "[bus_zmq] " << text << '\n'; }

}  // namespace argus::bus_zmq::detail

#endif  // ARGUS_BUS_ZMQ_SRC_DETAIL_HPP_
