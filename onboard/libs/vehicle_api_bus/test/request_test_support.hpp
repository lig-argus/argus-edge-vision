// 요청-응답 어댑터 테스트가 함께 쓰는 것
#ifndef ARGUS_VEHICLE_API_BUS_TEST_REQUEST_TEST_SUPPORT_HPP_
#define ARGUS_VEHICLE_API_BUS_TEST_REQUEST_TEST_SUPPORT_HPP_

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>

#include "argus/bus_testing/fake_bus.hpp"
#include "argus/core_testing/fake_clock.hpp"
#include "argus/core_testing/fake_poller.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api_bus {

inline constexpr std::chrono::milliseconds kRequestTimeout{100};

inline vehicle_api::RequestHeader sample_header() {
    vehicle_api::RequestHeader header;
    header.id.command_id.value = 42;
    header.id.session_id = "session-a";
    header.id.seq = 7;
    header.id.fc_epoch = 3;
    header.valid_until_monotonic_ns = 123'456'789'000;
    return header;
}

// 필드 번호 1, wire type length-delimited(2), 길이 5 인데 내용이 없는 본문 (파싱 실패)
inline bus::Bytes broken_body() { return {0x0A, 0x05}; }

class RequestTest : public ::testing::Test {
protected:
    // 지금 처리할 수 있는 이벤트를 처리한다 (요청 전달, 응답 전달)
    void process() {
        poller_.process_events(std::chrono::milliseconds{0}, kBudget);
        poller_.process_events(std::chrono::milliseconds{0}, kBudget);
    }

    // timeout 이 지나도록 기다리며 이벤트를 처리한다
    void wait_past_timeout() {
        poller_.process_events(kRequestTimeout + std::chrono::milliseconds{1}, kBudget);
    }

    static constexpr std::chrono::milliseconds kBudget{5};

    core_testing::FakeClock clock_;
    core_testing::FakePoller poller_{clock_};
    bus_testing::FakeBus bus_{poller_, clock_};
};

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_TEST_REQUEST_TEST_SUPPORT_HPP_
