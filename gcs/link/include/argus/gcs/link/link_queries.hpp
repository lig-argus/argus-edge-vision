// 화면이 link 모듈에 묻는 것을 쓰임새마다 작은 인터페이스로 나눈다. 화면의 각 부분은 필요한
// 인터페이스만 받고, VehicleLink 가 모두 구현한다. 화면 테스트는 테스트용 구현을 넣는다
#ifndef ARGUS_GCS_LINK_LINK_QUERIES_HPP_
#define ARGUS_GCS_LINK_LINK_QUERIES_HPP_

#include <QString>
#include <cstdint>
#include <optional>

#include "argus/gcs/link/command.hpp"
#include "argus/gcs/link/command_sender.hpp"
#include "argus/gcs/link/detection.hpp"
#include "argus/gcs/link/param_profile.hpp"
#include "argus/gcs/link/vehicle_state.hpp"

namespace argus::gcs::link {

// 명령을 지금 보낼 수 있는가 (명령 버튼과 안내 줄)
class ICommandStatus {
public:
    virtual ~ICommandStatus() = default;

    // 보낼 수 없으면 이유 (버튼 끄기)
    virtual std::optional<QString> refusal(Command command, const CommandArgs& args) const = 0;

    // refusal 중 화면 전체에 이미 보이는 이유(링크 두절, 응답 대기)를 뺀 것 (안내 줄)
    virtual std::optional<QString> specificRefusal(Command command,
                                                   const CommandArgs& args) const = 0;

    virtual bool awaitingReply(Command command) const = 0;
    virtual CommandTrace commandTrace(Command command) const = 0;

protected:
    ICommandStatus() = default;
    ICommandStatus(const ICommandStatus&) = default;
    ICommandStatus& operator=(const ICommandStatus&) = default;
    ICommandStatus(ICommandStatus&&) = default;
    ICommandStatus& operator=(ICommandStatus&&) = default;
};

// 가상 조종 상태
class IManualControlStatus {
public:
    virtual ~IManualControlStatus() = default;

    virtual bool manualControlOn() const = 0;
    virtual std::optional<QString> manualControlRefusal() const = 0;

    // manualControlRefusal 중 FC 링크 두절(화면 전체에 보인다)을 뺀 이유
    virtual std::optional<QString> specificManualControlRefusal() const = 0;

protected:
    IManualControlStatus() = default;
    IManualControlStatus(const IManualControlStatus&) = default;
    IManualControlStatus& operator=(const IManualControlStatus&) = default;
    IManualControlStatus(IManualControlStatus&&) = default;
    IManualControlStatus& operator=(IManualControlStatus&&) = default;
};

// FC 비행 환경 파라미터 상태
class IFlightEnvStatus {
public:
    virtual ~IFlightEnvStatus() = default;

    virtual std::optional<FlightEnv> currentFlightEnv() const = 0;
    virtual QString flightEnvText() const = 0;
    virtual std::optional<QString> flightEnvRefusal(FlightEnv env) const = 0;
    virtual bool paramWriteActive() const = 0;
    virtual bool rebootNeeded() const = 0;  // 환경을 바꾼 뒤 FC 재부팅 전

protected:
    IFlightEnvStatus() = default;
    IFlightEnvStatus(const IFlightEnvStatus&) = default;
    IFlightEnvStatus& operator=(const IFlightEnvStatus&) = default;
    IFlightEnvStatus(IFlightEnvStatus&&) = default;
    IFlightEnvStatus& operator=(IFlightEnvStatus&&) = default;
};

// 기체 상태와 탐지 결과 조회 (영상, 지도, 상태 표시)
class IVehicleQueries {
public:
    virtual ~IVehicleQueries() = default;

    virtual const VehicleState& state() const = 0;

    // 그 센서의 frameId 탐지 결과 (최근 기록 안에서). 영상 프레임과 박스를 맞춘다
    virtual std::optional<DetectionFrame> detectionForFrame(std::uint8_t sensor,
                                                            std::uint32_t frameId) const = 0;
    virtual std::optional<DetectionFrame> latestDetection(std::uint8_t sensor) const = 0;

    // 지면까지 거리 (m). 거리 센서 값이 없거나 범위 밖이거나 오래되면 없음
    virtual std::optional<float> groundDistance() const = 0;

    // 위치 추정이 수평 위치를 알고 있다 (Position 모드와 그 ARM 에 필요)
    virtual bool horizontalPositionKnown() const = 0;

protected:
    IVehicleQueries() = default;
    IVehicleQueries(const IVehicleQueries&) = default;
    IVehicleQueries& operator=(const IVehicleQueries&) = default;
    IVehicleQueries(IVehicleQueries&&) = default;
    IVehicleQueries& operator=(IVehicleQueries&&) = default;
};

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_LINK_QUERIES_HPP_
