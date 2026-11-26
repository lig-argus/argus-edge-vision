// DdsBus 가 사용하는 Fast DDS 고유 기능을 모은 곳. 다른 DDS 구현체로 바꿀 때 다시 작성할 범위는 이
// 파일과 그 구현, 그리고 IDL 변환 표다. 라이브러리 밖에서는 include 하지 않는다
// - Data Sharing 을 끄는 QoS (DDS 표준에 없는 Fast DDS 의 기능)
// - 요청-응답의 표본 식별자: 쓴 표본의 식별자 얻기, 응답에 요청의 식별자 붙이기 (Fast DDS 의
//   WriteParams. 표준 DDS-RPC 의 SampleIdentity 에 대응)
#ifndef ARGUS_BUS_DDS_SRC_FASTDDS_SPECIFIC_HPP_
#define ARGUS_BUS_DDS_SRC_FASTDDS_SPECIFIC_HPP_

#include <cstdint>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/rtps/common/SampleIdentity.hpp>
#include <optional>

#include "argus/bus_dds/dds_topic_qos.hpp"

namespace argus::bus_dds::detail {

using SampleIdentity = eprosima::fastdds::rtps::SampleIdentity;

// RELIABLE, VOLATILE, KEEP_LAST(depth), Data Sharing 끔. 발행자는 topic 의 Deadline 을 더한다
// (argus/bus_dds/dds_topic_qos.hpp)
eprosima::fastdds::dds::DataWriterQos writer_qos(std::int32_t depth, const DdsTopicQos& topic);
eprosima::fastdds::dds::DataReaderQos reader_qos(std::int32_t depth);

// 표본을 송신하고 그 표본의 식별자를 반환한다. 송신하지 못하면 nullopt
std::optional<SampleIdentity> write_identified(eprosima::fastdds::dds::DataWriter& writer,
                                               const void* sample);

// 요청의 식별자를 붙여 응답 표본을 송신한다. 송신하지 못하면 false
bool write_related(eprosima::fastdds::dds::DataWriter& writer, const void* sample,
                   const SampleIdentity& request);

// 받은 표본의 식별자와, 그 표본이 응답이면 요청의 식별자
const SampleIdentity& identity_of(const eprosima::fastdds::dds::SampleInfo& info);
const SampleIdentity& related_identity_of(const eprosima::fastdds::dds::SampleInfo& info);

// 표본의 순번 (발행자마다 1 부터)
std::uint64_t sequence_of(const eprosima::fastdds::dds::SampleInfo& info);

}  // namespace argus::bus_dds::detail

#endif  // ARGUS_BUS_DDS_SRC_FASTDDS_SPECIFIC_HPP_
