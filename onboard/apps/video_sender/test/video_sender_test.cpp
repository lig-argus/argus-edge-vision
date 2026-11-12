#include "argus/video_sender/video_sender.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <string_view>

#include "argus/perception_api/types.hpp"
#include "argus/video_sender/collected_frame.hpp"
#include "argus/video_sender/onvif_metadata_serializer.hpp"
#include "fake_stream_sink.hpp"

namespace argus::video_sender {
namespace {

using perception_api::Track;
using perception_api::Tracks;

constexpr std::int64_t kUnixNs = 1'792'048'527'321'000'000;  // 2026-10-15T07:15:27.321Z

bool contains(std::string_view text, std::string_view part) {
    return text.find(part) != std::string_view::npos;
}

CollectedFrame make_collected_frame(std::uint64_t frame_id) {
    CollectedFrame collected;
    collected.frame.source = "camera-ir";
    collected.frame.frame_id = frame_id;
    collected.frame.unix_ns = kUnixNs;
    collected.frame.width = 640;
    collected.frame.height = 480;
    return collected;
}

Tracks make_tracks(std::uint64_t frame_id) {
    Tracks tracks;
    tracks.source = "camera-ir";
    tracks.frame_id = frame_id;
    tracks.image_width = 640;
    tracks.image_height = 480;
    Track track;
    track.track_id = 12;
    track.box = {100.0, 100.0, 150.0, 180.0};
    tracks.tracks.push_back(track);
    return tracks;
}

class VideoSenderTest : public ::testing::Test {
protected:
    FakeStreamSink sink_;
    OnvifMetadataSerializer serializer_;
    VideoSender sender_{serializer_, sink_};
};

TEST_F(VideoSenderTest, PushesFrameWithItsMetadataDocument) {
    CollectedFrame collected = make_collected_frame(100);
    collected.tracks = make_tracks(100);

    sender_.send(collected);

    ASSERT_EQ(sink_.pushed().size(), 1U);
    EXPECT_EQ(sink_.pushed()[0].frame.frame_id, 100U);
    EXPECT_EQ(sink_.pushed()[0].frame.unix_ns, kUnixNs);
    EXPECT_EQ(sink_.pushed()[0].metadata_document, serializer_.serialize(collected));
    EXPECT_TRUE(contains(sink_.pushed()[0].metadata_document, R"(<tt:Object ObjectId="12">)"));
}

TEST_F(VideoSenderTest, PushesFrameWithoutTracksWithObjectlessDocument) {
    sender_.send(make_collected_frame(100));

    ASSERT_EQ(sink_.pushed().size(), 1U);
    const std::string& document = sink_.pushed()[0].metadata_document;
    EXPECT_FALSE(contains(document, "<tt:Object"));
    EXPECT_TRUE(contains(document, "<argus:FrameId>100</argus:FrameId>"));
}

TEST_F(VideoSenderTest, PushesOncePerFrameInOrder) {
    sender_.send(make_collected_frame(1));
    sender_.send(make_collected_frame(2));

    ASSERT_EQ(sink_.pushed().size(), 2U);
    EXPECT_EQ(sink_.pushed()[0].frame.frame_id, 1U);
    EXPECT_EQ(sink_.pushed()[1].frame.frame_id, 2U);
    EXPECT_TRUE(contains(sink_.pushed()[1].metadata_document, "<argus:FrameId>2</argus:FrameId>"));
}

}  // namespace
}  // namespace argus::video_sender
