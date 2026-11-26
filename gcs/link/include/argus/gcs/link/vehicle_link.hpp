// 통제기와 기체 사이 통신의 단일 진입점. 판단은 부품이 하고 이 클래스는 부품을 연결한다
// - 수신: 기체(sysid 1)의 FC, mavlink_gateway 메시지를 상태 해석, 링크 판단, 명령 응답, 비행 환경
//   파라미터, 영상 접속 정보 요청에 나눠 준다. 다른 시스템(QGC, 다른 통제기)의 메시지는 버린다
// - 송신: 명령은 송신 전 거부 판단을 거쳐 CommandSender 가 보낸다
// - 주기 작업은 tick() 에서 한다. app 의 타이머가 50 ms 마다 호출한다 (가상 조종 20 Hz)
// - 시각은 IClock, 송수신은 IMavlinkConnection 으로 받는다
// 부품: TelemetryDecoder(상태 해석), LinkMonitor(링크 판단), CommandSender(명령 절차),
// CommandPolicy(송신 전 거부), ManualControl(가상 조종), ModeChangeWatch(Offboard 이탈 경고),
// FlightEnvParams(비행 환경 파라미터), VideoStreamRequest(영상 접속 정보), DetectionStore(탐지
// 기록), StatusTextAssembler(STATUSTEXT 결합)
#ifndef ARGUS_GCS_LINK_VEHICLE_LINK_HPP_
#define ARGUS_GCS_LINK_VEHICLE_LINK_HPP_

#include <QObject>
#include <QString>
#include <cstdint>
#include <optional>
#include <vector>

#include "argus/gcs/core/clock.hpp"
#include "argus/gcs/link/command.hpp"
#include "argus/gcs/link/command_policy.hpp"
#include "argus/gcs/link/command_sender.hpp"
#include "argus/gcs/link/detection.hpp"
#include "argus/gcs/link/detection_store.hpp"
#include "argus/gcs/link/flight_conditions.hpp"
#include "argus/gcs/link/flight_env_params.hpp"
#include "argus/gcs/link/link_monitor.hpp"
#include "argus/gcs/link/link_queries.hpp"
#include "argus/gcs/link/log_entry.hpp"
#include "argus/gcs/link/manual_control.hpp"
#include "argus/gcs/link/mode_change_watch.hpp"
#include "argus/gcs/link/param_profile.hpp"
#include "argus/gcs/link/status_text.hpp"
#include "argus/gcs/link/vehicle_state.hpp"
#include "argus/gcs/link/video_stream_request.hpp"
#include "argus/gcs/mavlink_connection/mavlink_connection.hpp"
#include "argus/mavlink.h"

namespace argus::gcs::link {

class VehicleLink : public QObject,
                    public ICommandStatus,
                    public IManualControlStatus,
                    public IFlightEnvStatus,
                    public IVehicleQueries {
    Q_OBJECT

public:
    static constexpr std::int64_t kTickPeriodNs = 50 * kNsPerMs;

    // firstCommandNumber: 명령 번호의 시작값. 통제기 기동 시각(ms)을 2^24 로 나눈 나머지 (icd 4.1)
    VehicleLink(mavlink_connection::IMavlinkConnection& connection, const core::IClock& clock,
                std::uint32_t firstCommandNumber, QObject* parent = nullptr);

    // 이 통제기의 sysid (250~253). 응답의 대상 확인과 제어권 보유 판단에 쓴다. 범위 밖이면 false.
    // 연결(IMavlinkConnection)의 송신 sysid 와 같은 값이어야 한다
    bool setGcsSysid(std::uint8_t sysid);
    std::uint8_t gcsSysid() const { return state_.gcsSysid; }

    void tick();

    // 명령을 보낸다. 보낼 수 없으면 이유를 돌려주고 기록한다
    std::optional<QString> request(Command command, const CommandArgs& args = {});

    // 가상 조종. 켤 수 없으면 이유
    std::optional<QString> enableManualControl();
    void disableManualControl() { manual_.disable(); }
    void setManualAxes(const ManualAxes& axes) { manual_.setAxes(axes); }
    const ManualAxes& manualAxes() const { return manual_.axes(); }

    // 비행 환경 파라미터 묶음 (app 이 파일에서 읽어 넣는다)
    void setFlightEnvProfiles(std::vector<ParamProfile> profiles);
    std::optional<QString> applyFlightEnv(FlightEnv env);

    // 탐지 결과 한 프레임 (영상 수신 쪽이 RTSP 메타데이터를 바꿔 넣는다). 받아들이면 true
    bool addDetection(std::uint8_t sensor, DetectionFrame frame);
    // 송신 쪽 재시작 등으로 그 센서의 탐지 기록을 비운다
    void clearDetections(std::uint8_t sensor) { detections_.clear(sensor); }

    // ICommandStatus
    std::optional<QString> refusal(Command command, const CommandArgs& args) const override;
    std::optional<QString> specificRefusal(Command command, const CommandArgs& args) const override;
    bool awaitingReply(Command command) const override { return sender_.awaitingCommand(command); }
    CommandTrace commandTrace(Command command) const override { return sender_.trace(command); }

    // IManualControlStatus
    bool manualControlOn() const override { return manual_.enabled(); }
    std::optional<QString> manualControlRefusal() const override;
    std::optional<QString> specificManualControlRefusal() const override;

    // IFlightEnvStatus
    std::optional<FlightEnv> currentFlightEnv() const override { return envParams_.current(); }
    QString flightEnvText() const override;
    std::optional<QString> flightEnvRefusal(FlightEnv env) const override;
    bool paramWriteActive() const override { return envParams_.writing().has_value(); }
    bool rebootNeeded() const override { return envParams_.rebootNeeded(); }

    // IVehicleQueries
    const VehicleState& state() const override { return state_; }
    std::optional<DetectionFrame> detectionForFrame(std::uint8_t sensor,
                                                    std::uint32_t frameId) const override {
        return detections_.forFrame(sensor, frameId);
    }
    std::optional<DetectionFrame> latestDetection(std::uint8_t sensor) const override {
        return detections_.latest(sensor);
    }
    std::optional<float> groundDistance() const override { return conditions().groundDistance(); }
    bool horizontalPositionKnown() const override { return conditions().horizontalPositionKnown(); }

signals:
    void logged(const argus::gcs::link::LogEntry& entry);
    void detectionAdded(std::uint8_t sensor, const argus::gcs::link::DetectionFrame& frame);
    void videoStreamsReceived(const argus::gcs::link::VideoStreams& streams);

private:
    std::int64_t now() const { return clock_.nowMonotonicNs(); }
    FlightConditions conditions() const { return {state_, now()}; }
    CommandContext commandContext() const;
    void log(const LogEntry& entry) { emit logged(entry); }

    void onMessage(const mavlink_message_t& message);
    void onFcMessage(const mavlink_message_t& message, std::int64_t nowNs);
    void onGatewayMessage(const mavlink_message_t& message, std::int64_t nowNs);
    void onCommandAck(const mavlink_message_t& message, std::int64_t nowNs);
    void onCommandFinished(Command command, MAV_RESULT result);
    void logStatusText(const std::vector<StatusTextLine>& lines, LogKind kind, LogOrigin origin);
    void updateLinks(std::int64_t nowNs);
    void tickVideoStreams(std::int64_t nowNs);

    mavlink_connection::IMavlinkConnection& connection_;
    const core::IClock& clock_;
    VehicleState state_;
    LinkMonitor links_;
    CommandSender sender_;
    ManualControl manual_;
    ModeChangeWatch modeWatch_;
    FlightEnvParams envParams_;
    VideoStreamRequest videoStreams_;
    DetectionStore detections_;
    StatusTextAssembler fcText_;
    StatusTextAssembler gatewayText_;
};

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_VEHICLE_LINK_HPP_
