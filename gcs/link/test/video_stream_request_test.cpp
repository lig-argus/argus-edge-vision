#include "argus/gcs/link/video_stream_request.hpp"

#include <QtTest>
#include <cmath>
#include <cstdint>

#include "argus/mavlink.h"

namespace argus::gcs::link {
namespace {

constexpr std::uint8_t kIrFlags =
    VIDEO_STREAM_STATUS_FLAGS_RUNNING | VIDEO_STREAM_STATUS_FLAGS_THERMAL;
constexpr std::uint8_t kEoFlags = VIDEO_STREAM_STATUS_FLAGS_RUNNING;

mavlink_message_t streamInfo(std::uint8_t streamId, std::uint8_t count, std::uint8_t flags,
                             const char* uri, std::uint8_t type = VIDEO_STREAM_TYPE_RTSP,
                             std::uint8_t compid = kGatewayCompid) {
    mavlink_message_t message{};
    mavlink_msg_video_stream_information_pack(
        kVehicleSysid, compid, &message, streamId, count, type, flags, 30.0F, 640, 480, 0, 0, 0,
        streamId == 1 ? "IR" : "EO", uri, VIDEO_STREAM_ENCODING_H264, 0);
    return message;
}

mavlink_message_t ack(std::uint8_t result, std::uint8_t targetSystem = kGcsSysidDefault) {
    mavlink_message_t message{};
    mavlink_msg_command_ack_pack(kVehicleSysid, kGatewayCompid, &message, MAV_CMD_REQUEST_MESSAGE,
                                 result, 0, 0, targetSystem, kGcsCompid);
    return message;
}

}  // namespace

class VideoStreamRequestTest : public QObject {
    Q_OBJECT

private slots:
    // begin 직후 바로 보내고, 요청은 mavlink_gateway 로 가는 REQUEST_MESSAGE(269, 모든 스트림)
    void sendsRequestToGateway() {
        VideoStreamRequest request;
        request.setSysid(251);
        QVERIFY(!request.sendDue(0));
        request.begin(0);
        const std::optional<mavlink_message_t> message = request.sendDue(0);
        QVERIFY(message.has_value());
        const mavlink_message_t sent = message.value_or(mavlink_message_t{});
        QCOMPARE(sent.sysid, std::uint8_t{251});
        QCOMPARE(sent.compid, kGcsCompid);
        mavlink_command_long_t command{};
        mavlink_msg_command_long_decode(&sent, &command);
        QCOMPARE(command.command, std::uint16_t{MAV_CMD_REQUEST_MESSAGE});
        QCOMPARE(command.target_system, kVehicleSysid);
        QCOMPARE(command.target_component, kGatewayCompid);
        QCOMPARE(command.confirmation, std::uint8_t{0});
        QCOMPARE(command.param1, static_cast<float>(MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION));
        QCOMPARE(command.param2, 0.0F);
    }

    // 1 s 간격, 최대 3 번 보내고 그 뒤 1 s 안에 다 모이지 않으면 응답 없음 (한 번만)
    void retriesThenReportsNoReply() {
        VideoStreamRequest request;
        request.begin(0);
        QVERIFY(request.sendDue(0));
        QVERIFY(!request.sendDue(kRetryIntervalNs - 1));
        QVERIFY(request.sendDue(kRetryIntervalNs));
        QVERIFY(request.sendDue(2 * kRetryIntervalNs));
        QVERIFY(!request.sendDue(3 * kRetryIntervalNs));
        QCOMPARE(request.sends(), kMaxSends);
        QCOMPARE(request.check(3 * kRetryIntervalNs - 1), VideoStreamRequest::Outcome::None);
        QCOMPARE(request.check(3 * kRetryIntervalNs), VideoStreamRequest::Outcome::NoReply);
        QCOMPARE(request.check(4 * kRetryIntervalNs), VideoStreamRequest::Outcome::None);
        QVERIFY(!request.waiting());
    }

    // count 개가 다 모이면 센서 순으로. THERMAL 이면 IR, 아니면 EO
    void collectsStreamsBySensor() {
        VideoStreamRequest request;
        request.begin(0);
        QVERIFY(!request.onStreamInformation(
            streamInfo(2, 2, kEoFlags, "rtsp://192.168.0.2:8554/argus/eo")));
        const std::optional<VideoStreams> streams = request.onStreamInformation(
            streamInfo(1, 2, kIrFlags, "rtsp://192.168.0.2:8554/argus/ir"));
        QVERIFY(streams.has_value());
        const VideoStreams collected = streams.value_or(VideoStreams{});
        QCOMPARE(collected.size(), std::size_t{2});
        QCOMPARE(collected.at(0).sensor, std::uint8_t{ARGUS_SENSOR_IR});
        QCOMPARE(collected.at(0).uri, QStringLiteral("rtsp://192.168.0.2:8554/argus/ir"));
        QCOMPARE(collected.at(0).name, QStringLiteral("IR"));
        QCOMPARE(collected.at(0).encoding, std::uint8_t{VIDEO_STREAM_ENCODING_H264});
        QCOMPARE(collected.at(1).sensor, std::uint8_t{ARGUS_SENSOR_EO});
        QVERIFY(!request.waiting());
        QVERIFY(!request.sendDue(kRetryIntervalNs));
    }

    // RTSP 가 아니거나 rtsp:// 가 아닌 주소는 열지 않는다. 같은 센서는 앞 번호 하나만
    void skipsUnusableStreams() {
        VideoStreamRequest request;
        request.begin(0);
        QVERIFY(!request.onStreamInformation(
            streamInfo(1, 3, kIrFlags, "udp://0.0.0.0:5600", VIDEO_STREAM_TYPE_RTPUDP)));
        QVERIFY(!request.onStreamInformation(streamInfo(2, 3, kIrFlags, "rtsp://h:8554/a")));
        const std::optional<VideoStreams> streams =
            request.onStreamInformation(streamInfo(3, 3, kIrFlags, "rtsp://h:8554/b"));
        QVERIFY(streams.has_value());
        const VideoStreams collected = streams.value_or(VideoStreams{});
        QCOMPARE(collected.size(), std::size_t{1});
        QCOMPARE(collected.at(0).uri, QStringLiteral("rtsp://h:8554/a"));

        mavlink_video_stream_information_t info{};
        info.type = VIDEO_STREAM_TYPE_RTSP;
        QVERIFY(!decodeVideoStream(info));  // 빈 주소
    }

    // mavlink_gateway 가 아닌 컴포넌트의 스트림과 기다리지 않을 때의 스트림은 무시한다
    void ignoresOtherSourcesAndIdleState() {
        VideoStreamRequest request;
        QVERIFY(!request.onStreamInformation(streamInfo(1, 1, kIrFlags, "rtsp://h:8554/ir")));
        request.begin(0);
        QVERIFY(!request.onStreamInformation(
            streamInfo(1, 1, kIrFlags, "rtsp://h:8554/ir", VIDEO_STREAM_TYPE_RTSP, kFcCompid)));
        QVERIFY(request.onStreamInformation(streamInfo(1, 1, kIrFlags, "rtsp://h:8554/ir")));
    }

    // 수락, 진행 중은 계속 기다리고 그 밖의 결과는 거절. 다른 통제기에 보낸 응답은 무시한다
    void handlesCommandAck() {
        VideoStreamRequest request;
        request.begin(0);
        QCOMPARE(request.onCommandAck(ack(MAV_RESULT_ACCEPTED)), VideoStreamRequest::Outcome::None);
        QCOMPARE(request.onCommandAck(ack(MAV_RESULT_DENIED, 253)),
                 VideoStreamRequest::Outcome::None);
        QVERIFY(request.waiting());
        QCOMPARE(request.onCommandAck(ack(MAV_RESULT_DENIED)), VideoStreamRequest::Outcome::Denied);
        QVERIFY(!request.waiting());
        QCOMPARE(request.onCommandAck(ack(MAV_RESULT_DENIED)), VideoStreamRequest::Outcome::None);
    }
};

}  // namespace argus::gcs::link

QTEST_GUILESS_MAIN(argus::gcs::link::VideoStreamRequestTest)
#include "video_stream_request_test.moc"
