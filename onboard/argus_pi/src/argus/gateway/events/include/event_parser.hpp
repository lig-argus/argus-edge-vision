// PX4 이벤트 정의 해석 (Pi tools/real_fc/vendor/event_parser.py와 같은 규칙).
// 메시지 태그(<profile>, <a>, <param>)와 {N[:.D][단위]} 인수 치환을 원본과 같은 순서로 처리한다.
#pragma once

#include <cstdint>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "argus/gateway/common/include/python_compat.hpp"

namespace argus::gateway {

// 원본 EventLog._decode가 잡는 예외(KeyError·ValueError·struct.error·ParserException 등).
// what()은 Python str(exc)와 같게 만든다.
class EventDecodeError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct ParsedEventText {
    std::string message;
    std::string group;
    std::string description;
};

class EventParser {
public:
    EventParser() = default;
    EventParser(const EventParser&) = delete;
    EventParser& operator=(const EventParser&) = delete;
    EventParser(EventParser&&) = default;
    EventParser& operator=(EventParser&&) = default;
    ~EventParser() = default;

    // 정의 JSON(키 순서 유지)을 읽는다. version < 1이거나 형식이 틀리면 EventDecodeError.
    void load_definitions(nlohmann::ordered_json definitions);
    // 정의에 없는 ID면 nullopt. 인수 해석·메시지 처리 실패는 EventDecodeError.
    std::optional<ParsedEventText> parse(std::uint32_t event_id, const Bytes& arguments) const;
    // 2026-11-18: 정의의 이벤트 이름 (예: check_estimator_mag_interference). 없으면 nullopt.
    std::optional<std::string> name(std::uint32_t event_id) const;
    void set_profile(std::string profile) { profile_ = std::move(profile); }

private:
    struct Group {
        std::string name;
        std::map<long long, const nlohmann::ordered_json*> events;
    };
    struct Component {
        std::string name_space;
        std::vector<Group> groups;
    };

    nlohmann::ordered_json definitions_;
    std::map<long long, Component> components_;
    std::map<std::string, const nlohmann::ordered_json*> enums_;
    std::string profile_{"dev"};
};

}  // namespace argus::gateway
