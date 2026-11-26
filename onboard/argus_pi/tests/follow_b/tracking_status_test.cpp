#include <cassert>
#include <cmath>

#include "argus/tracking/contracts/include/tracking_status_snapshot.hpp"

int main() {
    using namespace follow;
    constexpr std::int64_t begin = 100000000000;
    constexpr std::uint64_t generation = 9007199254740993ULL;
    TrackingStatusSnapshot status{begin};
    assert(status.due(begin));
    auto message = status.snapshot(begin, generation, false);
    assert(message.at("schema") == "argus.tracking.status.v1" && message.at("seq") == 1);
    assert(!message.at("fc_fresh").get<bool>() && !message.at("cmc_valid").get<bool>());
    assert(message.at("fps") == 0 && message.at("generation").get<std::uint64_t>() == generation);
    assert(!status.due(begin + 999999999) && status.due(begin + 1000000000));
    status.record_success(begin + 900000000);
    status.record_success(begin + 950000000);
    message = status.snapshot(begin + 1000000000, generation, true);
    assert(message.at("seq") == 2 && message.at("cmc_valid").get<bool>());
    assert(std::abs(message.at("fps").get<double>() - 2) < 1e-12);
    message = status.snapshot(begin + 1200000000, generation, true);
    assert(!message.at("cmc_valid").get<bool>());  // 250ms 경계는 유효하지 않다.
    status.record_success(begin + 1250000000);
    status.record_failure();
    message = status.snapshot(begin + 1260000000, generation, true);
    assert(!message.at("cmc_valid").get<bool>());
    status.record_success(begin + 1300000000);
    message = status.snapshot(begin + 1310000000, generation, false);
    assert(!message.at("cmc_valid").get<bool>());
    status.record_success(begin + 1320000000);
    message = status.snapshot(begin + 1330000000, generation, true, false);
    assert(message.at("fc_fresh").get<bool>() && !message.at("cmc_valid").get<bool>());
    status.record_success(begin + 1500000000);
    message = status.snapshot(begin + 1400000000, generation, true);
    assert(!message.at("cmc_valid").get<bool>());  // 미래 표본을 정상으로 표시하지 않는다.
    const auto packed = NativeJson::to_msgpack(message);
    const auto decoded = NativeJson::from_msgpack(packed);
    assert(decoded.at("generation").get<std::uint64_t>() == generation);
}
