// bus 가 나르는 메시지의 타입을 식별하는 값과, 메시지와 함께 전달되는 정보.
// 타입을 전송 형식으로 바꾸는 방법은 여기에 없다. 통신 구현이 자기 변환 표에서 type 으로 찾는다
// (ZmqBus 는 직렬화 함수의 표, DDS 구현은 IDL 타입의 표).
// 사용: IBus 의 create_*() 에 넘긴다. 호출하는 코드는 argus/bus/typed.hpp 의 래퍼를 쓴다
#ifndef ARGUS_BUS_MESSAGE_TYPE_HPP_
#define ARGUS_BUS_MESSAGE_TYPE_HPP_

#include <cstdint>
#include <memory>
#include <string_view>
#include <type_traits>
#include <typeindex>

namespace argus::bus {

struct MessageType {
    std::string_view name;  // 로그와 타입 구분에 쓰는 이름. 프로그램이 끝날 때까지 유효한 문자열
    std::type_index type;   // C++ 값 타입

    // 기본값으로 만든 값. 구현이 받은 메시지를 채울 대상으로 쓴다
    std::shared_ptr<void> (*create)();

    // value 의 복사본. 구현이 값을 큐에 보관할 때 쓴다. value 는 type 의 값을 가리켜야 한다
    std::shared_ptr<void> (*clone)(const void* value);

    friend bool operator==(const MessageType& lhs, const MessageType& rhs) {
        return lhs.type == rhs.type;
    }
    friend bool operator!=(const MessageType& lhs, const MessageType& rhs) { return !(lhs == rhs); }
};

// T 는 기본 생성과 복사가 되는 값 타입이어야 한다. name 은 타입마다 하나로 정한다.
// 예외를 던지지 않으므로 반환값을 namespace 범위의 상수로 둘 수 있다
template <typename T>
MessageType message_type(std::string_view name) noexcept {
    static_assert(std::is_default_constructible_v<T>, "T 는 기본 생성이 되어야 한다");
    static_assert(std::is_copy_constructible_v<T>, "T 는 복사가 되어야 한다");
    return MessageType{
        name,
        std::type_index(typeid(T)),
        []() -> std::shared_ptr<void> { return std::make_shared<T>(); },
        [](const void* value) -> std::shared_ptr<void> {
            return std::make_shared<T>(*static_cast<const T*>(value));
        },
    };
}

// 메시지 하나와 함께 handler 에 전달되는 정보
struct MessageInfo {
    // 발행자가 발행마다 1 씩 올려 붙인 번호 (발행자마다 1 부터). 받는 쪽이 유실을 센다
    std::uint64_t sequence = 0;
};

}  // namespace argus::bus

#endif  // ARGUS_BUS_MESSAGE_TYPE_HPP_
