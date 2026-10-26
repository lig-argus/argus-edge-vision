// 통제기의 MAVLink 연결 인터페이스 (mavlink-router 와 UDP 14550 으로 주고받음)
// 규약: docs/spec/icd.md 1.2 (RPi 라우터 ↔ 통제기), 2장, 3장, 4장
// 구현: (다음 PR) UdpMavlinkConnection — 통제기가 먼저 보내야 라우터가 통제기 주소를 안다,
//      테스트용 FakeMavlinkConnection (gcs/test)
// 사용: TelemetryModel, ArgusStatusModel, CommandSender, ManualControl
#ifndef ARGUS_GCS_MAVLINK_CONNECTION_HPP_
#define ARGUS_GCS_MAVLINK_CONNECTION_HPP_

#include <QObject>

#include "argus/mavlink.h"

Q_DECLARE_METATYPE(mavlink_message_t)

namespace argus::gcs {

// 메시지는 Qt signal 로 전달한다. TelemetryModel, ArgusStatusModel, CommandSender 가 connect() 로
// messageReceived 를 연결하고 msgid 는 slot 안에서 필터링한다. 연결한 QObject 가 소멸하면 Qt 가
// 연결을 자동으로 끊으므로 onboard 의 Subscription 에 해당하는 객체는 없다.
class IMavlinkConnection : public QObject {
    Q_OBJECT

public:
    explicit IMavlinkConnection(QObject* parent = nullptr) : QObject(parent) {}
    ~IMavlinkConnection() override;

    // 메시지 하나를 보낸다. 보내기를 기다리지 않고 바로 돌아온다. false 는 소켓 오류
    virtual bool send(const mavlink_message_t& message) = 0;

signals:
    // 메시지 하나를 받을 때마다 발생한다. Qt 이벤트 루프를 돌리는 스레드에서 발생한다
    void messageReceived(const mavlink_message_t& message);
};

}  // namespace argus::gcs

#endif  // ARGUS_GCS_MAVLINK_CONNECTION_HPP_
