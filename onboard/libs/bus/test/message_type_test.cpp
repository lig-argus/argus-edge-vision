#include "argus/bus/message_type.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <typeindex>

namespace argus::bus {
namespace {

struct Sample {
    int value = 7;
    std::string text = "default";
};

struct Other {
    int value = 0;
};

TEST(MessageTypeTest, HoldsNameAndCppType) {
    const MessageType type = message_type<Sample>("test.Sample");

    EXPECT_EQ(type.name, "test.Sample");
    EXPECT_EQ(type.type, std::type_index(typeid(Sample)));
}

TEST(MessageTypeTest, CreateMakesDefaultValue) {
    const MessageType type = message_type<Sample>("test.Sample");

    const std::shared_ptr<void> created = type.create();

    ASSERT_NE(created, nullptr);
    const auto* sample = static_cast<const Sample*>(created.get());
    EXPECT_EQ(sample->value, 7);
    EXPECT_EQ(sample->text, "default");
}

TEST(MessageTypeTest, CloneCopiesTheValue) {
    const MessageType type = message_type<Sample>("test.Sample");
    Sample original;
    original.value = 42;
    original.text = "original";

    const std::shared_ptr<void> cloned = type.clone(&original);
    original.value = 0;  // 복사본은 원본이 바뀌어도 그대로다
    original.text.clear();

    ASSERT_NE(cloned, nullptr);
    const auto* sample = static_cast<const Sample*>(cloned.get());
    EXPECT_NE(sample, &original);
    EXPECT_EQ(sample->value, 42);
    EXPECT_EQ(sample->text, "original");
}

TEST(MessageTypeTest, ComparesByCppTypeOnly) {
    const MessageType sample = message_type<Sample>("test.Sample");
    const MessageType renamed = message_type<Sample>("test.Renamed");
    const MessageType other = message_type<Other>("test.Sample");

    EXPECT_TRUE(sample == renamed);  // 이름이 달라도 C++ 타입이 같으면 같다
    EXPECT_FALSE(sample != renamed);
    EXPECT_TRUE(sample != other);  // 이름이 같아도 C++ 타입이 다르면 다르다
    EXPECT_FALSE(sample == other);
}

TEST(MessageInfoTest, SequenceIsZeroByDefault) { EXPECT_EQ(MessageInfo{}.sequence, 0U); }

}  // namespace
}  // namespace argus::bus
