#include "argus/video_sender/onvif_metadata_serializer.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <string_view>

#include "argus/perception_api/types.hpp"
#include "argus/video_sender/collected_frame.hpp"

namespace argus::video_sender {
namespace {

using perception_api::Track;
using perception_api::Tracks;
using perception_api::TrackState;

// 2026-10-15T07:15:27.321Z
constexpr std::int64_t kUnixNs = 1'792'048'527'321'000'000;
constexpr std::int32_t kTankClassId = 0;

bool contains(std::string_view text, std::string_view part) {
    return text.find(part) != std::string_view::npos;
}

CollectedFrame make_collected_frame() {
    CollectedFrame collected;
    collected.frame.source = "camera-ir";
    collected.frame.frame_id = 100;
    collected.frame.unix_ns = kUnixNs;
    collected.frame.width = 640;
    collected.frame.height = 480;
    return collected;
}

Track make_track() {
    Track track;
    track.track_id = 12;
    track.classification.class_id = kTankClassId;
    track.classification.label = "tank";
    track.classification.confidence = 0.9;
    track.box = {100.0, 100.0, 150.0, 180.0};
    track.state = TrackState::Confirmed;
    return track;
}

Tracks make_tracks() {
    Tracks tracks;
    tracks.source = "camera-ir";
    tracks.frame_id = 100;
    tracks.generation = 1;
    tracks.image_width = 640;
    tracks.image_height = 480;
    return tracks;
}

class OnvifMetadataSerializerTest : public ::testing::Test {
protected:
    OnvifMetadataSerializer serializer_{{{kTankClassId, "Vehicle"}}};
};

// docs/spec/video.md 3.2 의 예시와 같은 입력
TEST_F(OnvifMetadataSerializerTest, SerializesTrackAsInSpecExample) {
    CollectedFrame collected = make_collected_frame();
    collected.tracks = make_tracks();
    collected.tracks.value().tracks.push_back(make_track());

    const std::string expected =
        R"(<tt:MetadataStream xmlns:tt="http://www.onvif.org/ver10/schema")"
        R"( xmlns:argus="urn:argus:metadata:1">)"
        R"(<tt:VideoAnalytics>)"
        R"(<tt:Frame UtcTime="2026-10-15T07:15:27.321Z" Source="camera-ir">)"
        R"(<tt:Transformation>)"
        R"(<tt:Translate x="-1.0" y="-1.0"/>)"
        R"(<tt:Scale x="0.003125" y="0.00416667"/>)"
        R"(</tt:Transformation>)"
        R"(<tt:Object ObjectId="12">)"
        R"(<tt:Appearance>)"
        R"(<tt:Shape>)"
        R"(<tt:BoundingBox left="100.0" top="380.0" right="150.0" bottom="300.0"/>)"
        R"(<tt:CenterOfGravity x="125.0" y="340.0"/>)"
        R"(</tt:Shape>)"
        R"(<tt:Class><tt:Type Likelihood="0.9">Vehicle</tt:Type></tt:Class>)"
        R"(</tt:Appearance>)"
        R"(<tt:Extension>)"
        R"(<argus:ClassId>0</argus:ClassId>)"
        R"(<argus:Label>tank</argus:Label>)"
        R"(<argus:Generation>1</argus:Generation>)"
        R"(<argus:State>CONFIRMED</argus:State>)"
        R"(</tt:Extension>)"
        R"(</tt:Object>)"
        R"(<tt:Extension>)"
        R"(<argus:FrameId>100</argus:FrameId>)"
        R"(<argus:ImageWidth>640</argus:ImageWidth>)"
        R"(<argus:ImageHeight>480</argus:ImageHeight>)"
        R"(</tt:Extension>)"
        R"(</tt:Frame>)"
        R"(</tt:VideoAnalytics>)"
        R"(</tt:MetadataStream>)";

    EXPECT_EQ(serializer_.serialize(collected), expected);
}

TEST_F(OnvifMetadataSerializerTest, FrameWithoutTracksHasNoObject) {
    const std::string xml = serializer_.serialize(make_collected_frame());

    EXPECT_FALSE(contains(xml, "<tt:Object"));
    EXPECT_TRUE(
        contains(xml, R"(<tt:Frame UtcTime="2026-10-15T07:15:27.321Z" Source="camera-ir">)"));
    EXPECT_TRUE(contains(xml, "<argus:FrameId>100</argus:FrameId>"));
}

TEST_F(OnvifMetadataSerializerTest, FrameWithoutTracksUsesFrameSize) {
    CollectedFrame collected = make_collected_frame();
    collected.frame.width = 1920;
    collected.frame.height = 1080;

    const std::string xml = serializer_.serialize(collected);

    EXPECT_TRUE(contains(xml, R"(<tt:Scale x="0.00104167" y="0.00185185"/>)"));
    EXPECT_TRUE(contains(xml, "<argus:ImageWidth>1920</argus:ImageWidth>"));
    EXPECT_TRUE(contains(xml, "<argus:ImageHeight>1080</argus:ImageHeight>"));
}

TEST_F(OnvifMetadataSerializerTest, EmptyTracksHasNoObject) {
    CollectedFrame collected = make_collected_frame();
    collected.tracks = make_tracks();

    const std::string xml = serializer_.serialize(collected);

    EXPECT_FALSE(contains(xml, "<tt:Object"));
    EXPECT_TRUE(contains(xml, "</tt:Transformation><tt:Extension>"));
}

TEST_F(OnvifMetadataSerializerTest, UsesImageSizeOfTracksForBoxes) {
    CollectedFrame collected = make_collected_frame();
    collected.tracks = make_tracks();
    collected.tracks.value().image_width = 320;
    collected.tracks.value().image_height = 240;
    Track track = make_track();
    track.box = {10.0, 20.0, 30.0, 60.0};
    collected.tracks.value().tracks.push_back(track);

    const std::string xml = serializer_.serialize(collected);

    EXPECT_TRUE(contains(xml, R"(<tt:Scale x="0.00625" y="0.00833333"/>)"));
    EXPECT_TRUE(contains(xml, R"(left="10.0" top="220.0" right="30.0" bottom="180.0")"));
    EXPECT_TRUE(contains(xml, "<argus:ImageWidth>320</argus:ImageWidth>"));
}

TEST_F(OnvifMetadataSerializerTest, SerializesEveryTrackInOrder) {
    CollectedFrame collected = make_collected_frame();
    collected.tracks = make_tracks();
    Track first = make_track();
    first.track_id = 3;
    Track second = make_track();
    second.track_id = 16'777'215;
    collected.tracks.value().tracks = {first, second};

    const std::string xml = serializer_.serialize(collected);

    const std::size_t first_position = xml.find(R"(<tt:Object ObjectId="3">)");
    const std::size_t second_position = xml.find(R"(<tt:Object ObjectId="16777215">)");
    ASSERT_NE(first_position, std::string::npos);
    ASSERT_NE(second_position, std::string::npos);
    EXPECT_LT(first_position, second_position);
}

TEST_F(OnvifMetadataSerializerTest, RoundsCoordinatesToOneDecimal) {
    CollectedFrame collected = make_collected_frame();
    collected.tracks = make_tracks();
    Track track = make_track();
    track.box = {100.26, 100.04, 150.74, 180.96};
    collected.tracks.value().tracks.push_back(track);

    const std::string xml = serializer_.serialize(collected);

    EXPECT_TRUE(contains(xml, R"(left="100.3" top="380.0" right="150.7" bottom="299.0")"));
}

TEST_F(OnvifMetadataSerializerTest, WritesGenerationAndPredictedState) {
    CollectedFrame collected = make_collected_frame();
    collected.tracks = make_tracks();
    collected.tracks.value().generation = 16'777'215;
    Track track = make_track();
    track.state = TrackState::Predicted;
    collected.tracks.value().tracks.push_back(track);

    const std::string xml = serializer_.serialize(collected);

    EXPECT_TRUE(contains(xml, "<argus:Generation>16777215</argus:Generation>"));
    EXPECT_TRUE(contains(xml, "<argus:State>PREDICTED</argus:State>"));
}

TEST_F(OnvifMetadataSerializerTest, OmitsClassForClassIdWithoutOnvifType) {
    CollectedFrame collected = make_collected_frame();
    collected.tracks = make_tracks();
    Track track = make_track();
    track.classification.class_id = 7;
    track.classification.label = "truck";
    collected.tracks.value().tracks.push_back(track);

    const std::string xml = serializer_.serialize(collected);

    EXPECT_FALSE(contains(xml, "<tt:Class>"));
    EXPECT_TRUE(contains(xml, "<argus:ClassId>7</argus:ClassId>"));
    EXPECT_TRUE(contains(xml, "<argus:Label>truck</argus:Label>"));
}

TEST_F(OnvifMetadataSerializerTest, WritesLikelihoodWithoutExponent) {
    CollectedFrame collected = make_collected_frame();
    collected.tracks = make_tracks();
    Track certain = make_track();
    certain.classification.confidence = 1.0;
    Track unlikely = make_track();
    unlikely.classification.confidence = 0.00001234;
    collected.tracks.value().tracks = {certain, unlikely};

    const std::string xml = serializer_.serialize(collected);

    EXPECT_TRUE(contains(xml, R"(Likelihood="1.0")"));
    EXPECT_TRUE(contains(xml, R"(Likelihood="0.00001234")"));
}

TEST_F(OnvifMetadataSerializerTest, EscapesXmlSpecialCharacters) {
    CollectedFrame collected = make_collected_frame();
    collected.frame.source = R"(ir<1>&"a"'b')";
    collected.tracks = make_tracks();
    Track track = make_track();
    track.classification.label = "a<b>&c";
    collected.tracks.value().tracks.push_back(track);

    const std::string xml = serializer_.serialize(collected);

    EXPECT_TRUE(contains(xml, R"(Source="ir&lt;1&gt;&amp;&quot;a&quot;&apos;b&apos;")"));
    EXPECT_TRUE(contains(xml, "<argus:Label>a&lt;b&gt;&amp;c</argus:Label>"));
}

TEST_F(OnvifMetadataSerializerTest, OmitsControlCharacters) {
    CollectedFrame collected = make_collected_frame();
    collected.frame.source = std::string{"ir\x01\x1F"} + "1";
    collected.tracks = make_tracks();
    Track track = make_track();
    track.classification.label = std::string{"ta\x02nk"} + std::string(1, '\0');
    collected.tracks.value().tracks.push_back(track);

    const std::string xml = serializer_.serialize(collected);

    EXPECT_TRUE(contains(xml, R"(Source="ir1")"));
    EXPECT_TRUE(contains(xml, "<argus:Label>tank</argus:Label>"));
}

TEST_F(OnvifMetadataSerializerTest, WritesTabAndLineBreaksAsCharacterReferences) {
    CollectedFrame collected = make_collected_frame();
    collected.frame.source = "a\tb\nc\rd";

    EXPECT_TRUE(contains(serializer_.serialize(collected), R"(Source="a&#x9;b&#xA;c&#xD;d")"));
}

TEST_F(OnvifMetadataSerializerTest, KeepsUtf8Text) {
    CollectedFrame collected = make_collected_frame();
    collected.frame.source = "카메라-ir";

    EXPECT_TRUE(contains(serializer_.serialize(collected), R"(Source="카메라-ir")"));
}

TEST_F(OnvifMetadataSerializerTest, TruncatesUtcTimeBelowMillisecond) {
    CollectedFrame collected = make_collected_frame();
    collected.frame.unix_ns = kUnixNs + 999'999;

    EXPECT_TRUE(
        contains(serializer_.serialize(collected), R"(UtcTime="2026-10-15T07:15:27.321Z")"));
}

TEST_F(OnvifMetadataSerializerTest, PadsUtcTimeFieldsWithZero) {
    CollectedFrame collected = make_collected_frame();
    collected.frame.unix_ns = 1'767'225'600'005'000'000;  // 2026-01-01T00:00:00.005Z

    EXPECT_TRUE(
        contains(serializer_.serialize(collected), R"(UtcTime="2026-01-01T00:00:00.005Z")"));
}

TEST_F(OnvifMetadataSerializerTest, ZeroImageSizeWritesZeroScale) {
    CollectedFrame collected = make_collected_frame();
    collected.frame.width = 0;
    collected.frame.height = 0;

    EXPECT_TRUE(contains(serializer_.serialize(collected), R"(<tt:Scale x="0.0" y="0.0"/>)"));
}

TEST(OnvifMetadataSerializerDefaultTest, DefaultHasNoOnvifTypes) {
    const OnvifMetadataSerializer serializer;
    CollectedFrame collected = make_collected_frame();
    collected.tracks = make_tracks();
    collected.tracks.value().tracks.push_back(make_track());

    EXPECT_FALSE(contains(serializer.serialize(collected), "<tt:Class>"));
}

}  // namespace
}  // namespace argus::video_sender
