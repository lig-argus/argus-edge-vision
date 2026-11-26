#include "argus/mavlink_gateway/sysfs_cpu_temperature.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <optional>
#include <string>

namespace argus::mavlink_gateway {
namespace {

std::string write_temp_file(const std::string& name, const std::string& content) {
    const std::string path = ::testing::TempDir() + name;
    std::ofstream(path) << content;
    return path;
}

TEST(SysfsCpuTemperatureTest, ReadsMillidegrees) {
    const std::string path = write_temp_file("argus_cpu_temp_valid", "52375\n");

    EXPECT_EQ(SysfsCpuTemperature(path).celsius(), std::optional<double>{52.375});
    std::remove(path.c_str());
}

TEST(SysfsCpuTemperatureTest, MissingOrInvalidFileGivesNoValue) {
    const std::string path = write_temp_file("argus_cpu_temp_invalid", "hot\n");

    EXPECT_EQ(SysfsCpuTemperature(path).celsius(), std::nullopt);
    EXPECT_EQ(SysfsCpuTemperature(::testing::TempDir() + "argus_no_such_file").celsius(),
              std::nullopt);
    std::remove(path.c_str());
}

}  // namespace
}  // namespace argus::mavlink_gateway
