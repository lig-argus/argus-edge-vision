// msgpack 코덱 (Python msgpack 1.x packb(use_bin_type=True) / unpackb(raw=False)과 같은 규칙).
// 추종 제어기와 주고받는 ZMQ 메시지에 쓴다. 키 순서를 지키는 ordered_json을 쓰며 bytes는 binary다.
#pragma once

#include <cstdint>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "argus/gateway/common/include/python_compat.hpp"

namespace argus::gateway {

using Wire = nlohmann::ordered_json;

// 정수는 가장 작은 형식(양수는 uint 계열), 실수는 항상 float64, 문자열은 str8/16/32, bytes는 bin.
Bytes msgpack_pack(const Wire& value);

// raw=False: str은 UTF-8로 풀고(잘못되면 ValueError), bin은 binary, float32는 double.
// 맵 키는 str만 받는다 (원본 strict_map_key). reject_duplicate_keys면 같은 키를 ValueError로
// 거부한다. 남는 바이트가 있으면 ValueError (원본 ExtraData).
Wire msgpack_unpack(const Bytes& data, bool reject_duplicate_keys = false);

// Python == (True == 1, 1 == 1.0, 목록·사전은 원소별, 사전 키 순서 무관)
bool python_equal(const Wire& left, const Wire& right);

// Python type(value) is int (bool 제외)
inline bool python_is_int(const Wire& value) { return value.is_number_integer(); }

// Python type(value) in (int, float) (bool 제외)
inline bool python_is_number(const Wire& value) { return value.is_number(); }

// Python str(value): 문자열은 그대로, 나머지는 repr
std::string python_str_value(const Wire& value);

// Python repr(value) (목록·사전·bytes 포함)
std::string python_repr_value(const Wire& value);

// Python json.dumps(value, ensure_ascii=..., allow_nan=False, separators=...) (키 순서 유지).
// compact면 (',', ':'), 아니면 기본 (', ', ': '). NaN·무한대는 원본처럼 ValueError.
std::string python_json_dumps(const Wire& value, bool ensure_ascii, bool compact);

// Python json.dumps(value, ensure_ascii=..., indent=n) (구분자 (',', ': '), 빈 목록·사전은 [] {})
std::string python_json_dumps_indent(const Wire& value, int indent, bool ensure_ascii);

// Python 진릿값 (None·False·0·빈 문자열·bytes·목록·사전은 거짓)
bool wire_truthy(const Wire& value);

// Python dict[key] = value (있는 키는 자리 유지, 없으면 끝에 추가)
inline void wire_set(Wire& object, const std::string& key, Wire value) {
    if (object.is_null()) {
        object = Wire::object();
    }
    if (object.contains(key)) {
        object.at(key) = std::move(value);
    } else {
        object.emplace(key, std::move(value));
    }
}

// Python dict.get(key) (없거나 사전이 아니면 None)
Wire wire_get(const Wire& object, const std::string& key);

}  // namespace argus::gateway
