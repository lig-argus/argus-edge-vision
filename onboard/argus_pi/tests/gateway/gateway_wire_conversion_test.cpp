// 2026-11-16 to_wire 최적화 동일성 시험: 이전 구현(wire_set으로 키마다 검색)과 새 구현(바로
// 덧붙임)이 같은 Wire와 같은 msgpack 바이트를 내는지, 무작위 중첩 JSON과 실제 자세 기록 모양으로
// 확인한다.
#include <chrono>
#include <cstdint>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

#include "argus/gateway/common/include/msgpack_codec.hpp"
#include "argus/gateway/follow/include/follow_owner.hpp"

namespace gw = argus::gateway;

namespace {

// 이전 구현 그대로 (기준)
gw::Wire reference_to_wire(const gw::Json& value) {  // NOLINT(misc-no-recursion)
    switch (value.type()) {
        case gw::Json::value_t::object: {
            gw::Wire out = gw::Wire::object();
            for (const auto& item : value.items()) {
                gw::wire_set(out, item.key(), reference_to_wire(item.value()));
            }
            return out;
        }
        case gw::Json::value_t::array: {
            gw::Wire out = gw::Wire::array();
            for (const auto& item : value) {
                out.push_back(reference_to_wire(item));
            }
            return out;
        }
        case gw::Json::value_t::number_float:
            return value.get<double>();
        case gw::Json::value_t::number_unsigned:
            return value.get<std::uint64_t>();
        case gw::Json::value_t::number_integer:
            return value.get<std::int64_t>();
        case gw::Json::value_t::boolean:
            return value.get<bool>();
        case gw::Json::value_t::string:
            return value.get<std::string>();
        default:
            return nullptr;
    }
}

gw::Json random_json(std::mt19937& rng, int depth) {  // NOLINT(misc-no-recursion)
    std::uniform_int_distribution<int> kind(0, depth > 0 ? 7 : 5);
    switch (kind(rng)) {
        case 0:
            return nullptr;
        case 1:
            return std::uniform_int_distribution<int>(0, 1)(rng) == 1;
        case 2:
            return std::uniform_int_distribution<std::int64_t>(-1000000, 1000000)(rng);
        case 3:
            return std::uniform_int_distribution<std::uint64_t>(0, UINT64_MAX)(rng);
        case 4:
            return std::uniform_real_distribution<double>(-1e6, 1e6)(rng);
        case 5:
            return "s" + std::to_string(rng() % 1000);
        case 6: {
            gw::Json out = gw::Json::array();
            const int n = static_cast<int>(rng() % 6);
            for (int i = 0; i < n; ++i) {
                out.push_back(random_json(rng, depth - 1));
            }
            return out;
        }
        default: {
            gw::Json out = gw::Json::object();
            const int n = static_cast<int>(rng() % 12);
            for (int i = 0; i < n; ++i) {
                out["k" + std::to_string(rng() % 20)] = random_json(rng, depth - 1);
            }
            return out;
        }
    }
}

void require_same(const gw::Json& input, const char* what) {
    const gw::Wire expected = reference_to_wire(input);
    const gw::Wire actual = gw::to_wire(input);
    if (expected != actual || expected.dump() != actual.dump()) {
        throw std::runtime_error(std::string("to_wire mismatch: ") + what);
    }
    if (gw::msgpack_pack(expected) != gw::msgpack_pack(actual)) {
        throw std::runtime_error(std::string("msgpack mismatch: ") + what);
    }
}

}  // namespace

int main() {
    try {
        std::mt19937 rng(20261116);
        for (int i = 0; i < 3000; ++i) {
            require_same(random_json(rng, 4), "random");
        }
        // 실제 cmc_pose_history 행 모양 (16행 × 3종류)
        gw::Json history = gw::Json::object();
        for (const char* kind : {"ATTITUDE", "LOCAL_POSITION_NED", "DISTANCE_SENSOR"}) {
            gw::Json rows = gw::Json::array();
            for (int i = 0; i < 16; ++i) {
                rows.push_back({{"received_monotonic_ns", 1000000000LL + i},
                                {"data",
                                 {{"time_boot_ms", 12345 + i},
                                  {"roll", 0.01 * i},
                                  {"pitch", -0.02},
                                  {"yaw", 1.5},
                                  {"rollspeed", 0.1},
                                  {"pitchspeed", 0.2},
                                  {"yawspeed", 0.3},
                                  {"x", 1.0},
                                  {"y", 2.0},
                                  {"z", -1.5}}}});
            }
            history[kind] = rows;
        }
        require_same(history, "pose history");
        require_same(gw::Json::object(), "empty object");
        require_same(gw::Json::array(), "empty array");

        // 참고용 속도 비교 (판정에는 쓰지 않음)
        const auto time_it = [&](auto&& f) {
            const auto start = std::chrono::steady_clock::now();
            for (int i = 0; i < 300; ++i) {
                (void)f(history);
            }
            return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() -
                                                             start)
                       .count() /
                   300;
        };
        const double before = time_it(reference_to_wire);
        const double after = time_it([](const gw::Json& v) { return gw::to_wire(v); });
        std::cout << "pose history to_wire: before " << before << " us, after " << after << " us\n";
        std::cout << "GatewayWireConversionTest passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
