// Python 게이트웨이와 같은 문자열을 만들기 위한 도구.
// 원본이 Python repr/json.dumps/str.strip 결과를 그대로 기록·비교하므로 C++에서도 같은 규칙을 쓴다.
#pragma once

#include <cstddef>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace argus::gateway {

using Bytes = std::vector<std::uint8_t>;

// Python 3 repr(float): 가장 짧은 왕복 자릿수, 소수점 위치가 -4 이하이거나 16 초과면 지수 표기.
std::string python_float_repr(double value);

// Python json.dumps(value, sort_keys=True) 기본값(ensure_ascii, ", " / ": ")과 같은 문자열.
std::string python_json_dumps_sorted(const nlohmann::json& value);

// Python 진릿값 (None·False·0·빈 문자열·빈 목록·빈 객체는 거짓).
bool python_truthy(const nlohmann::json& value);

std::u32string utf8_to_u32(const std::string& text);
std::string u32_to_utf8(const std::u32string& text);

// Python str.isspace()와 같은 공백 판정, str.strip()과 같은 양끝 제거.
bool python_is_space(char32_t code_point);
std::u32string python_strip(const std::u32string& text);

// Python text[:count]처럼 코드 포인트 기준으로 앞부분을 자른다.
std::string utf8_prefix(const std::string& text, std::size_t code_points);

// Python bytes.hex() (소문자).
std::string hex_lower(const Bytes& bytes);

// Python int(text) (앞뒤 공백, 부호, 숫자 사이 밑줄 허용). 실패하면 false.
bool python_parse_int(const std::string& text, long long& value);

// Python "{:.Nf}".format(value): 정확한 반올림, NaN은 부호 없이 nan, 무한대는 inf/-inf.
// 표기 길이가 넘치면 std::overflow_error.
std::string python_format_fixed(double value, int decimals);

// 원본 vehicle.number(value, scale): float(value) * scale이 유한하면 값, 아니면 없음.
// float()처럼 bool, 정수, 실수, 숫자 문자열(공백·밑줄 허용)을 받는다.
std::optional<double> python_number(const nlohmann::json& value, double scale = 1.0);

// Python len(str) (코드 포인트 수)
std::size_t utf8_length(const std::string& text);

// Python ValueError에 해당하는 거부 (메시지는 원본과 같다)
class ValueError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Object insertion and replacement mirror JSON dictionary assignment.
inline void json_set(nlohmann::json& object, std::string key, nlohmann::json value) {
    if (object.is_null()) {
        object = nlohmann::json::object();
    }
    object.get_ref<nlohmann::json::object_t&>().insert_or_assign(std::move(key), std::move(value));
}

}  // namespace argus::gateway
