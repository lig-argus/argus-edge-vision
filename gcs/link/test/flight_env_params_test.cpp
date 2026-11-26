#include "argus/gcs/link/flight_env_params.hpp"

#include <QtTest>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <vector>

#include "argus/mavlink.h"

namespace argus::gcs::link {
namespace {

constexpr std::int64_t kMs = kNsPerMs;

ParamProfile indoorProfile() {
    return {FlightEnv::Indoor,
            {{QStringLiteral("SYS_HAS_GPS"), MAV_PARAM_TYPE_INT32, 0.0},
             {QStringLiteral("EKF2_OF_POS_X"), MAV_PARAM_TYPE_REAL32, 0.075}}};
}

ParamProfile outdoorProfile() {
    return {FlightEnv::Outdoor,
            {{QStringLiteral("SYS_HAS_GPS"), MAV_PARAM_TYPE_INT32, 1.0},
             {QStringLiteral("EKF2_OF_POS_X"), MAV_PARAM_TYPE_REAL32, 0.0}}};
}

// FC 가 보내는 PARAM_VALUE. INT32 는 float 칸에 비트 그대로 싣는다
mavlink_message_t paramValue(const char* name, double value, std::uint8_t type,
                             std::uint8_t compid = kFcCompid) {
    auto encoded = static_cast<float>(value);
    if (type == MAV_PARAM_TYPE_INT32) {
        const auto raw = static_cast<std::int32_t>(value);
        std::memcpy(&encoded, &raw, sizeof(encoded));
    }
    mavlink_message_t message{};
    mavlink_msg_param_value_pack(kVehicleSysid, compid, &message, name, encoded, type, 2, 0);
    return message;
}

template <std::size_t N>
QString idText(const char (&chars)[N]) {  // NOLINT(modernize-avoid-c-arrays): 생성된 구조체의 필드
    const char* end = std::find(std::begin(chars), std::end(chars), '\0');
    return QString::fromLatin1(chars, std::distance(std::begin(chars), end));
}

// 지상, 시동 꺼짐, FC 연결, 추종 없음
VehicleState writableState() {
    VehicleState state;
    state.fcLink.state = LinkState::Ok;
    state.fc.armed = false;
    state.fc.landedState = MAV_LANDED_STATE_ON_GROUND;
    state.gateway.received = true;
    state.gateway.missionState = MissionState::Idle;
    return state;
}

}  // namespace

class FlightEnvParamsTest : public QObject {
    Q_OBJECT

private slots:
    void init() {
        sent_.clear();
        logs_.clear();
    }

    // 이름을 check 마다 하나씩 묻고, 돌아온 값이 한 묶음과 모두 같으면 그 환경
    void readsNamesOneByOneAndTellsEnvironment() {
        FlightEnvParams params = make();
        params.queueReads(0);
        QCOMPARE(params.text(LinkState::Ok, 0), QStringLiteral("읽기 중"));
        params.check(0, LinkState::Ok);
        params.check(50 * kMs, LinkState::Ok);
        params.check(100 * kMs, LinkState::Ok);
        QCOMPARE(sent_.size(), std::size_t{2});
        QCOMPARE(sent_.at(0).msgid, static_cast<std::uint32_t>(MAVLINK_MSG_ID_PARAM_REQUEST_READ));
        mavlink_param_request_read_t read{};
        mavlink_msg_param_request_read_decode(&sent_.at(1), &read);
        QCOMPARE(idText(read.param_id), QStringLiteral("EKF2_OF_POS_X"));
        QCOMPARE(read.target_system, kVehicleSysid);
        QCOMPARE(read.target_component, kFcCompid);
        QCOMPARE(read.param_index, std::int16_t{-1});

        params.onMessage(paramValue("SYS_HAS_GPS", 0, MAV_PARAM_TYPE_INT32), 0);
        params.onMessage(paramValue("EKF2_OF_POS_X", 0.075, MAV_PARAM_TYPE_REAL32), 0);
        QCOMPARE(params.current(), std::optional<FlightEnv>(FlightEnv::Indoor));
        QCOMPARE(params.text(LinkState::Ok, 0), QStringLiteral("실내"));
    }

    // 어느 묶음과도 같지 않으면 실내와 다른 값을 보인다
    void showsMixedValues() {
        FlightEnvParams params = make();
        params.onMessage(paramValue("SYS_HAS_GPS", 1, MAV_PARAM_TYPE_INT32), 0);
        params.onMessage(paramValue("EKF2_OF_POS_X", 0.075, MAV_PARAM_TYPE_REAL32), 0);
        QCOMPARE(params.current(), std::nullopt);
        QCOMPARE(params.text(LinkState::Ok, 0),
                 QStringLiteral("섞임: 실내와 다른 값 SYS_HAS_GPS=1"));
        QCOMPARE(params.text(LinkState::Lost, 0), QStringLiteral("-"));
    }

    // 응답이 빠진 이름은 3 s 마다 다시 묻고, 횟수를 넘으면 미확인
    void retriesMissingNamesThenReportsUnknown() {
        FlightEnvParams params = make();
        params.queueReads(0);
        params.onMessage(paramValue("SYS_HAS_GPS", 0, MAV_PARAM_TYPE_INT32), 0);
        std::int64_t now = 0;
        for (std::int32_t round = 0; round < kMaxSends; ++round) {
            params.check(now, LinkState::Ok);
            params.check(now + 50 * kMs, LinkState::Ok);
            now += FlightEnvParams::kReadRetryNs;
        }
        params.check(now, LinkState::Ok);
        // 첫 읽기 2 개와 빠진 이름 다시 묻기 2 번
        QCOMPARE(sent_.size(), std::size_t{4});
        QCOMPARE(params.text(LinkState::Ok, now),
                 QStringLiteral("미확인: 응답 없음 EKF2_OF_POS_X"));
    }

    // FC 가 아닌 컴포넌트의 PARAM_VALUE 는 쓰지 않는다
    void ignoresOtherComponents() {
        FlightEnvParams params = make();
        params.onMessage(paramValue("SYS_HAS_GPS", 0, MAV_PARAM_TYPE_INT32, kGatewayCompid), 0);
        params.onMessage(paramValue("EKF2_OF_POS_X", 0.075, MAV_PARAM_TYPE_REAL32), 0);
        QCOMPARE(params.current(), std::nullopt);
    }

    // 값마다 PARAM_SET 을 보내고 같은 값의 PARAM_VALUE 를 받으면 다음 값을 보낸다
    void writesEachValueAfterConfirmation() {
        FlightEnvParams params = make();
        QCOMPARE(params.apply(FlightEnv::Outdoor, writableState(), 0), std::nullopt);
        QCOMPARE(params.writing(), std::optional<FlightEnv>(FlightEnv::Outdoor));
        QCOMPARE(sent_.size(), std::size_t{1});
        mavlink_param_set_t set{};
        mavlink_msg_param_set_decode(&sent_.at(0), &set);
        QCOMPARE(idText(set.param_id), QStringLiteral("SYS_HAS_GPS"));
        QCOMPARE(set.target_system, kVehicleSysid);
        QCOMPARE(set.target_component, kFcCompid);
        QCOMPARE(set.param_type, std::uint8_t{MAV_PARAM_TYPE_INT32});
        std::int32_t raw = 0;
        std::memcpy(&raw, &set.param_value, sizeof(raw));
        QCOMPARE(raw, 1);

        // 앞서 읽은 옛 값의 늦은 응답은 확인으로 치지 않는다
        params.onMessage(paramValue("SYS_HAS_GPS", 0, MAV_PARAM_TYPE_INT32), 10 * kMs);
        QCOMPARE(sent_.size(), std::size_t{1});
        params.onMessage(paramValue("SYS_HAS_GPS", 1, MAV_PARAM_TYPE_INT32), 20 * kMs);
        QCOMPARE(sent_.size(), std::size_t{2});
        params.onMessage(paramValue("EKF2_OF_POS_X", 0.0, MAV_PARAM_TYPE_REAL32), 30 * kMs);
        QCOMPARE(params.writing(), std::nullopt);
        QVERIFY(params.rebootNeeded());
        QCOMPARE(params.current(), std::optional<FlightEnv>(FlightEnv::Outdoor));
        QVERIFY(logs_.back().text.startsWith(QStringLiteral("실외 모드 파라미터 2개 적용")));

        params.onRebooting(40 * kMs);
        QVERIFY(!params.rebootNeeded());
        QCOMPARE(params.current(), std::nullopt);
    }

    // 확인이 없으면 1 s 마다 다시 보내고, 3 번 보낸 뒤에는 실패를 기록하고 멈춘다
    void retriesWriteThenGivesUp() {
        FlightEnvParams params = make();
        QCOMPARE(params.apply(FlightEnv::Indoor, writableState(), 0), std::nullopt);
        std::int64_t now = 0;
        for (std::int32_t i = 0; i < kMaxSends; ++i) {
            now += FlightEnvParams::kWriteRetryNs + kMs;
            params.check(now, LinkState::Ok);
        }
        QCOMPARE(sent_.size(), static_cast<std::size_t>(kMaxSends));
        QCOMPARE(params.writing(), std::nullopt);
        QCOMPARE(logs_.back().level, LogLevel::Serious);
        QVERIFY(logs_.back().text.startsWith(QStringLiteral("실내 모드 적용 실패")));
    }

    // 지상이고 시동이 꺼져 있고 추종, 스캔 중이 아닐 때만 쓴다 (icd 5.2)
    void refusesUnlessGroundDisarmedAndIdle() {
        FlightEnvParams params = make();
        VehicleState state = writableState();
        state.fc.armed = true;
        QCOMPARE(
            params.writeRefusal(FlightEnv::Indoor, state),
            std::optional<QString>(QStringLiteral("ARM 상태에서 변경 불가: 착륙 후 DISARM 필요")));
        state = writableState();
        state.fc.landedState = MAV_LANDED_STATE_IN_AIR;
        QCOMPARE(params.writeRefusal(FlightEnv::Indoor, state),
                 std::optional<QString>(QStringLiteral("비행 중 변경 불가: 착륙 후 가능")));
        state.fc.landedState = MAV_LANDED_STATE_UNDEFINED;
        QVERIFY(params.writeRefusal(FlightEnv::Indoor, state).has_value());
        state = writableState();
        state.gateway.missionState = MissionState::ScanActive;
        QCOMPARE(params.writeRefusal(FlightEnv::Indoor, state),
                 std::optional<QString>(QStringLiteral("추종, 스캔 중 변경 불가: [중단] 후 가능")));
        state.gateway.missionState = MissionState::FollowActive;
        QVERIFY(params.writeRefusal(FlightEnv::Indoor, state).has_value());
        state = writableState();
        state.fcLink.state = LinkState::Lost;
        QCOMPARE(params.writeRefusal(FlightEnv::Indoor, state),
                 std::optional<QString>(QStringLiteral("FC 미연결")));

        state = writableState();
        state.fc.armed = true;
        QVERIFY(params.apply(FlightEnv::Indoor, state, 0).has_value());
        QVERIFY(sent_.empty());
        QCOMPARE(logs_.back().level, LogLevel::Serious);
        QVERIFY(logs_.back().text.startsWith(QStringLiteral("실내 모드 전환 안 함.")));
    }

    // 쓰는 중에는 다시 쓰지 않고, 묶음이 없는 환경은 쓰지 않는다
    void refusesWhileWritingOrWithoutProfile() {
        FlightEnvParams params = make();
        QCOMPARE(params.apply(FlightEnv::Indoor, writableState(), 0), std::nullopt);
        QCOMPARE(params.writeRefusal(FlightEnv::Outdoor, writableState()),
                 std::optional<QString>(QStringLiteral("실내 모드 파라미터 기록 중")));

        FlightEnvParams onlyIndoor = make();
        onlyIndoor.setProfiles({indoorProfile()});
        QCOMPARE(onlyIndoor.writeRefusal(FlightEnv::Outdoor, writableState()),
                 std::optional<QString>(QStringLiteral("실외 파라미터 묶음 없음")));
    }

    // 보내는 메시지의 sysid 는 이 통제기의 sysid
    void sendsWithGcsSysid() {
        FlightEnvParams params = make();
        params.setSysid(252);
        params.queueReads(0);
        params.check(0, LinkState::Ok);
        QCOMPARE(sent_.at(0).sysid, std::uint8_t{252});
        QCOMPARE(sent_.at(0).compid, kGcsCompid);
    }

private:
    FlightEnvParams make() {
        FlightEnvParams params([this](const mavlink_message_t& m) { sent_.push_back(m); },
                               [this](const LogEntry& e) { logs_.push_back(e); });
        params.setProfiles({indoorProfile(), outdoorProfile()});
        return params;
    }

    std::vector<mavlink_message_t> sent_;
    std::vector<LogEntry> logs_;
};

}  // namespace argus::gcs::link

QTEST_GUILESS_MAIN(argus::gcs::link::FlightEnvParamsTest)
#include "flight_env_params_test.moc"
