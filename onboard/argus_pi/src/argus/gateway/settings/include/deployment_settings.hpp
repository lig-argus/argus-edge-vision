// 배포 설정 읽기 (Pi deployment_config.py의 load·materialize와 같은 규칙).
// 장치·소켓·비행 동작은 하지 않는다. materialize만 run/config/<digest>/ 아래 파일을 쓴다.
#pragma once

#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

namespace argus::gateway {

using OrderedJson = nlohmann::ordered_json;
using EnvironmentMap = std::map<std::string, std::string>;

// Python 원본이 ValueError로 거부하는 설정 오류. 메시지는 원본과 같다.
class SettingsError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// 현재 프로세스의 환경 변수.
EnvironmentMap current_environment();

// deployment_config.load(root, environ). 반환 객체의 키 순서도 원본과 같다.
OrderedJson load_deployment_settings(const std::filesystem::path& root,
                                     const EnvironmentMap& environment);

// deployment_config.materialize(data). 연결·신원 값만 바꾼 moving/yaw 설정과 router 설정을 쓴다.
OrderedJson materialize_deployment_settings(OrderedJson data);

}  // namespace argus::gateway
