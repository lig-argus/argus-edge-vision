// 게이트웨이 1단계 단위 시험: Python 표기 호환, SHA-256, 이벤트 해석·순번, 송신 잠금.
// Python 원본과의 대량 비교는 tests/gateway/differential/stage1_diff.py가 맡는다.
#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "argus/gateway/common/include/python_compat.hpp"
#include "argus/gateway/common/include/sha256.hpp"
#include "argus/gateway/events/include/event_log.hpp"
#include "argus/gateway/events/include/event_parser.hpp"
#include "argus/gateway/ownership/include/command_owner.hpp"
#include "argus/gateway/settings/include/flight_limits.hpp"

namespace gw = argus::gateway;
using nlohmann::json;

namespace {

int& failures() {
    static int count = 0;
    return count;
}

void check(bool condition, const std::string& what) {
    if (!condition) {
        ++failures();
        std::cerr << "실패: " << what << '\n';
    }
}

void check_equal(const std::string& actual, const std::string& expected, const std::string& what) {
    check(actual == expected, what + " → [" + actual + "] 기대 [" + expected + "]");
}

void python_formatting() {
    check_equal(gw::python_float_repr(0.1), "0.1", "repr 0.1");
    check_equal(gw::python_float_repr(1.0), "1.0", "repr 1.0");
    check_equal(gw::python_float_repr(-2.5), "-2.5", "repr -2.5");
    check_equal(gw::python_float_repr(1e16), "1e+16", "repr 1e16");
    check_equal(gw::python_float_repr(1e15), "1000000000000000.0", "repr 1e15");
    check_equal(gw::python_float_repr(1e-5), "1e-05", "repr 1e-5");
    check_equal(gw::python_float_repr(0.0001), "0.0001", "repr 1e-4");
    check_equal(gw::python_float_repr(static_cast<double>(0.1F)), "0.10000000149011612",
                "repr float32 0.1");
    check_equal(gw::python_float_repr(1.5e300), "1.5e+300", "repr 1.5e300");
    check_equal(gw::python_float_repr(-0.0), "-0.0", "repr -0.0");
    const json value = {{"b", 1}, {"a", {1.5, "한"}}, {"c", nullptr}, {"d", "\n\"\\"}};
    check_equal(gw::python_json_dumps_sorted(value),
                R"({"a": [1.5, ")" + std::string(1, '\\') + "ud55c" +
                    R"("], "b": 1, "c": null, "d": "\n\"\\"})",
                "json.dumps sort_keys");
    check_equal(gw::utf8_prefix("가나다라", 2), "가나", "코드 포인트 자르기");
    long long parsed = 0;
    check(gw::python_parse_int(" -1_0 ", parsed) && parsed == -10, "int(' -1_0 ')");
    check(!gw::python_parse_int("1__0", parsed), "int('1__0') 거부");
    check(!gw::python_parse_int("x", parsed), "int('x') 거부");
}

void sha256_vectors() {
    check_equal(gw::sha256_hex(""),
                "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855", "sha256('')");
    check_equal(gw::sha256_hex("abc"),
                "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
                "sha256('abc')");
    check_equal(gw::sha256_hex(std::string(1000, 'a')),
                "41edece42d63e8d9bf515a9ba6932e1c20cbc9f5a5d134645adb5db1b9737ea3",
                "sha256(a*1000)");
}

void parser_messages() {
    gw::EventParser parser;
    parser.load_definitions(nlohmann::ordered_json::parse(R"({
      "version": 2,
      "components": {"1": {"namespace": "px4",
        "enums": {"mode": {"type": "uint8_t", "entries": {"1": {"description": "Hold"}, "2": {"description": "Land"}}},
                  "flags": {"type": "uint8_t", "is_bitfield": true, "separator": ", ",
                            "entries": {"1": {"description": "A"}, "4": {"description": "C"}}}},
        "event_groups": {"default": {"events": {
          "7": {"name": "e7", "message": " Alt {1:.2m_v}, mode {2}, flags {3} \\{x} <param>COM_X</param> ",
                "description": "<profile name=\"dev\">dev only</profile><profile name=\"!dev\">user</profile> {4}",
                "arguments": [{"type": "float"}, {"type": "mode"}, {"type": "flags"}, {"type": "int16_t"}]}}}}}}})"));
    const gw::Bytes args{0x00, 0x00, 0xc0, 0x3f, 0x02, 0x07, 0xfe, 0xff};
    const auto parsed = parser.parse((1U << 24) | 7U, args);
    check(parsed.has_value(), "이벤트 7 해석");
    if (parsed) {
        check_equal(parsed->message, "Alt 1.50 m, mode Land, flags A, (unknown: 7), C {x} COM_X",
                    "메시지 치환");
        check_equal(parsed->description, "dev only -2", "프로필·음수 인수");
        check_equal(parsed->group, "default", "그룹 이름");
    }
    check(!parser.parse((2U << 24) | 7U, args).has_value(), "없는 구성요소");
    bool threw = false;
    try {
        (void)parser.parse((1U << 24) | 7U, gw::Bytes{0x00, 0x00});
    } catch (const gw::EventDecodeError& error) {
        threw = std::string(error.what()) == "unpack requires a buffer of 4 bytes";
    }
    check(threw, "인수 부족은 struct.error 문구");
}

void event_sequencing() {
    double now = 10.0;
    gw::EventLog log([&now] { return now; }, "/nonexistent/fc-events.json");
    check(log.status().rfind("메타데이터 사용 불가: ", 0) == 0, "메타데이터 없음 표시");
    log.ingest("CURRENT_EVENT_SEQUENCE", {{"sequence", 100}, {"flags", 0}});
    const auto first = log.due();
    check(first && first->first == 69 && first->second == 100, "처음 32개 과거 요청 69..100");
    check(!log.due().has_value(), "1초 안에는 다시 요청하지 않음");
    log.ingest("EVENT", {{"sequence", 101},
                         {"id", 5},
                         {"event_time_boot_ms", 1000},
                         {"log_levels", 6},
                         {"arguments", json::array({1, 2})},
                         {"destination_system", 0},
                         {"destination_component", 0}});
    const json snapshot = log.snapshot();
    check(snapshot.at("records").size() == 1, "EVENT 기록");
    check(snapshot.at("records").at(0).at("severity") == "INFO", "수준 INFO");
    check(!log.observe_boot(5000) && log.observe_boot(2000), "SYSTEM_TIME 역행은 재부팅");
    check(log.epoch() == 1, "재부팅 뒤 epoch 1");
    check(!log.observe_boot(0xfffffff0LL), "시각 증가는 재부팅 아님");
    check(!log.observe_boot(10), "32bit wrap은 재부팅 아님");
}

void command_owner_lock() {
    const std::string path = "/tmp/argus-gateway-owner-test-" + std::to_string(::getpid());
    {
        gw::CommandOwner first(path);
        gw::CommandOwner second(path);
        check(first.acquire(), "첫 잠금");
        check(!second.acquire(), "두 번째 잠금 거부");
        const json status = second.status();
        check(status.at("role") == "fc" && status.at("available") == false,
              "다른 fd에서 role fc 확인");
        first.release();
        check(second.status().at("available") == true, "해제 뒤 사용 가능");
        check(second.acquire(), "해제 뒤 두 번째 잠금");
    }
    std::filesystem::remove(path);
    gw::CommandOwner disabled("");
    check(disabled.acquire() && disabled.status().at("enabled") == false, "경로 없으면 비활성");
}

void limits_and_profiles() {
    const gw::FlightLimits limits;
    check(limits.brake_height() > 4.89 && limits.brake_height() < 4.91, "제동 고도 4.9");
    check(
        gw::outdoor_profile().size() == 9 && gw::outdoor_profile().back().first == "EKF2_BARO_CTRL",
        "실외 파라미터 순서");
}

}  // namespace

int main() {
    try {
        python_formatting();
        sha256_vectors();
        parser_messages();
        event_sequencing();
        command_owner_lock();
        limits_and_profiles();
    } catch (const std::exception& error) {
        std::cerr << "예외: " << error.what() << '\n';
        return 1;
    }
    if (failures() != 0) {
        std::cerr << failures() << "개 실패\n";
        return 1;
    }
    std::cout << "GatewayStage1Test 통과\n";
    return 0;
}
