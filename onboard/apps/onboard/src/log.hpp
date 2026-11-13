// onboard 의 구현 파일들이 함께 사용하는 로그 함수. 모듈 밖에서는 include 하지 않는다
#ifndef ARGUS_ONBOARD_SRC_LOG_HPP_
#define ARGUS_ONBOARD_SRC_LOG_HPP_

#include <iostream>
#include <string>

namespace argus::onboard::detail {

inline void log_error(const std::string& text) { std::cerr << "[onboard] " << text << '\n'; }

}  // namespace argus::onboard::detail

#endif  // ARGUS_ONBOARD_SRC_LOG_HPP_
