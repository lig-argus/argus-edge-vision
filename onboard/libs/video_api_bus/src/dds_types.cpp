#include "argus/video_api_bus/dds_types.hpp"

#include <chrono>
#include <string>

#include "argus/protocol/topics.hpp"
#include "argus/video_api/types.hpp"
#include "argus_msg/video_stream_infoPubSubTypes.hpp"
#include "video_stream_info_idl.hpp"

namespace argus::video_api_bus {

bus_dds::DdsTypeTable dds_types() {
    bus_dds::DdsTypeTable table;
    bus_dds::add_dds_type<video_api::VideoStreamInfo, idl::VideoStreamInfoPubSubType>(
        table,
        [](const video_api::VideoStreamInfo& value, idl::VideoStreamInfo& sample) {
            to_idl_video_stream_info(value, sample);
            return true;
        },
        &from_idl_video_stream_info);
    return table;
}

bus_dds::DdsTopicQosTable dds_topic_qos() {
    return {{std::string(protocol::kVideoStreamInfo), {std::chrono::seconds{3}}}};
}

}  // namespace argus::video_api_bus
