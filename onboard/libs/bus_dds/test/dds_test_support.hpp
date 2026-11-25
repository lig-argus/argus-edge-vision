// bus_dds 테스트가 공유하는 값 타입, 타입 표, 이벤트 처리 반복
#ifndef ARGUS_BUS_DDS_TEST_DDS_TEST_SUPPORT_HPP_
#define ARGUS_BUS_DDS_TEST_DDS_TEST_SUPPORT_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <string_view>

#include "argus/bus/message_type.hpp"
#include "argus/bus_dds/dds_type_table.hpp"
#include "argus/core/poll_poller.hpp"
#include "test_textPubSubTypes.hpp"

namespace argus::bus_dds {

// 테스트용 C++ 값 타입. IDL 타입 argus::bus_dds_test::TestText 에 대응한다
struct Text {
    std::string text;
    std::int32_t number = 0;
};

// 표에 넣지 않는 타입 (표에 없는 타입의 생성 실패 테스트용)
struct Unregistered {
    std::int32_t value = 0;
};

using std::string_view_literals::operator""sv;

inline const bus::MessageType kTextType = bus::message_type<Text>("argus.test.Text"sv);
inline const bus::MessageType kUnregisteredType =
    bus::message_type<Unregistered>("argus.test.Unregistered"sv);

// 같은 컴퓨터의 다른 프로그램과 섞이지 않도록 기본값(0)이 아닌 domain 을 사용한다
inline constexpr std::uint32_t kTestDomain = 77;

inline DdsTypeTable test_types() {
    DdsTypeTable table;
    add_dds_type<Text, bus_dds_test::TestTextPubSubType>(
        table,
        [](const Text& value, bus_dds_test::TestText& sample) {
            sample.text(value.text);
            sample.number(value.number);
            return true;
        },
        [](const bus_dds_test::TestText& sample, Text& value) {
            value.text = sample.text();
            value.number = sample.number();
            return true;
        });
    return table;
}

// condition 이 참이 될 때까지 step 과 이벤트 처리를 반복한다 (최대 약 4 초). DDS 의
// 발견(discovery)이 끝나기 전에 보낸 표본은 전달되지 않으므로, 보내는 동작을 step 에 넣어 받을
// 때까지 다시 보낸다
inline bool pump_until(core::PollPoller& poller, const std::function<void()>& step,
                       const std::function<bool()>& condition) {
    constexpr std::int32_t kMaxRounds = 400;
    for (std::int32_t round = 0; round < kMaxRounds && !condition(); ++round) {
        if (step) {
            step();
        }
        poller.process_events(std::chrono::milliseconds{10}, std::chrono::milliseconds{50});
    }
    return condition();
}

// duration 동안 이벤트를 처리한다. 더 도착하는 것이 없는지 확인할 때 사용한다
inline void process_for(core::PollPoller& poller, std::chrono::milliseconds duration) {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
        poller.process_events(std::chrono::milliseconds{10}, std::chrono::milliseconds{50});
    }
}

}  // namespace argus::bus_dds

#endif  // ARGUS_BUS_DDS_TEST_DDS_TEST_SUPPORT_HPP_
