#include "fastdds_specific.hpp"

#include <chrono>
#include <cstdint>
#include <fastdds/dds/core/ReturnCode.hpp>
#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <fastdds/rtps/common/WriteParams.hpp>

namespace argus::bus_dds::detail {
namespace {

namespace dds = eprosima::fastdds::dds;

template <typename Qos>
void set_common(Qos& qos, std::int32_t depth) {
    qos.reliability().kind = dds::RELIABLE_RELIABILITY_QOS;
    qos.durability().kind = dds::VOLATILE_DURABILITY_QOS;
    qos.history().kind = dds::KEEP_LAST_HISTORY_QOS;
    qos.history().depth = depth;
    qos.data_sharing().off();
}

dds::Duration_t duration_of(std::chrono::nanoseconds value) {
    constexpr std::int64_t kNsPerSecond = 1'000'000'000;
    const std::int64_t ns = value.count();
    return {static_cast<std::int32_t>(ns / kNsPerSecond),
            static_cast<std::uint32_t>(ns % kNsPerSecond)};
}

}  // namespace

dds::DataWriterQos writer_qos(std::int32_t depth, const DdsTopicQos& topic) {
    dds::DataWriterQos qos = dds::DATAWRITER_QOS_DEFAULT;
    set_common(qos, depth);
    if (topic.deadline.has_value()) {
        qos.deadline().period = duration_of(topic.deadline.value());
    }
    return qos;
}

dds::DataReaderQos reader_qos(std::int32_t depth) {
    dds::DataReaderQos qos = dds::DATAREADER_QOS_DEFAULT;
    set_common(qos, depth);
    return qos;
}

std::optional<SampleIdentity> write_identified(dds::DataWriter& writer, const void* sample) {
    eprosima::fastdds::rtps::WriteParams params;
    if (writer.write(sample, params) != dds::RETCODE_OK) {
        return std::nullopt;
    }
    return params.sample_identity();
}

bool write_related(dds::DataWriter& writer, const void* sample, const SampleIdentity& request) {
    eprosima::fastdds::rtps::WriteParams params;
    params.related_sample_identity(request);
    return writer.write(sample, params) == dds::RETCODE_OK;
}

const SampleIdentity& identity_of(const dds::SampleInfo& info) { return info.sample_identity; }

const SampleIdentity& related_identity_of(const dds::SampleInfo& info) {
    return info.related_sample_identity;
}

std::uint64_t sequence_of(const dds::SampleInfo& info) {
    return info.sample_identity.sequence_number().to64long();
}

}  // namespace argus::bus_dds::detail
