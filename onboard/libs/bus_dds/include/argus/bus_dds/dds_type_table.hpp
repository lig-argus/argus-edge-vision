// DdsBus 가 C++ 값 타입을 IDL 타입의 표본(sample)으로 변환하는 데 사용하는 표.
// DdsBus 는 메시지 타입을 알지 못한다. 실행 파일이 사용하는 타입마다 IDL 타입(fastddsgen 이 생성한
// PubSubType)과 변환 함수를 이 표에 넣어 DdsBus 에 넘긴다 (ZmqBus 의 ZmqCodecTable 에 대응).
// 표에 없는 타입으로는 발행자, 구독자, 요청하는 쪽, 응답하는 쪽을 만들 수 없다.
// 규약: docs/spec/process_interfaces.md 3장
#ifndef ARGUS_BUS_DDS_DDS_TYPE_TABLE_HPP_
#define ARGUS_BUS_DDS_DDS_TYPE_TABLE_HPP_

#include <fastdds/dds/topic/TypeSupport.hpp>
#include <functional>
#include <map>
#include <typeindex>
#include <utility>

namespace argus::bus_dds {

// C++ 값 타입 하나의 IDL 타입과 변환 함수
struct DdsType {
    // IDL 타입의 TypeSupport 를 만든다. DdsBus 는 이것으로 타입을 등록하고 표본을 만든다
    std::function<eprosima::fastdds::dds::TypeSupport()> make_type_support;

    // value(C++ 값)를 sample(IDL 타입의 표본)에 채운다. 바꿀 수 없으면 false
    std::function<bool(const void* value, void* sample)> to_idl;

    // sample 을 value 에 채운다. value 는 기본값으로 만든 값을 가리킨다. 형식이 맞지 않으면 false
    // 이고, DdsBus 는 그 표본을 폐기한다
    std::function<bool(const void* sample, void* value)> from_idl;
};

// C++ 값 타입 → IDL 타입과 변환 함수
using DdsTypeTable = std::map<std::type_index, DdsType>;

// C++ 값 타입 T 를 IDL 타입(PubSubType::type)에 대응시킨다. 같은 타입이 이미 있으면 바꾼다.
// to_idl 은 bool(const T& value, Idl& sample), from_idl 은 bool(const Idl& sample, T& value) 로
// 부를 수 있어야 한다 (Idl 은 PubSubType::type). void* 를 타입으로 바꾸는 코드를 이 함수 한 곳에만
// 둔다
template <typename T, typename PubSubType, typename ToIdl, typename FromIdl>
void add_dds_type(DdsTypeTable& table, ToIdl to_idl, FromIdl from_idl) {
    using Idl = typename PubSubType::type;
    DdsType type;
    type.make_type_support = [] { return eprosima::fastdds::dds::TypeSupport(new PubSubType()); };
    type.to_idl = [to_idl = std::move(to_idl)](const void* value, void* sample) -> bool {
        return to_idl(*static_cast<const T*>(value), *static_cast<Idl*>(sample));
    };
    type.from_idl = [from_idl = std::move(from_idl)](const void* sample, void* value) -> bool {
        return from_idl(*static_cast<const Idl*>(sample), *static_cast<T*>(value));
    };
    table.insert_or_assign(std::type_index(typeid(T)), std::move(type));
}

}  // namespace argus::bus_dds

#endif  // ARGUS_BUS_DDS_DDS_TYPE_TABLE_HPP_
