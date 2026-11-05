// 테스트용 IMavlinkConnection. 보낸 메시지를 기록하고, 테스트가 넣은 메시지로 messageReceived
// signal 을 발생시킨다. 통제기 테스트만 쓰므로 gcs/test 에 둔다
#ifndef ARGUS_GCS_TEST_FAKE_MAVLINK_CONNECTION_HPP_
#define ARGUS_GCS_TEST_FAKE_MAVLINK_CONNECTION_HPP_

#include <QObject>
#include <QVector>

#include "argus/gcs/mavlink_connection.hpp"
#include "argus/mavlink.h"

namespace argus::gcs {

class FakeMavlinkConnection : public IMavlinkConnection {
    Q_OBJECT

public:
    explicit FakeMavlinkConnection(QObject* parent = nullptr) : IMavlinkConnection(parent) {}

    bool send(const mavlink_message_t& message) override {
        sent_.push_back(message);
        return sendResult_;
    }

    // 테스트용: 라우터에서 메시지 하나가 도착한 것으로 등록한다. signal 이 발생해 연결된 slot 이
    // 바로 불린다
    void simulateMessageReceived(const mavlink_message_t& message) {
        emit messageReceived(message);
    }

    // 테스트용: 보낸 메시지와 send() 의 반환값
    const QVector<mavlink_message_t>& sent() const { return sent_; }
    void setSendResult(bool result) { sendResult_ = result; }

private:
    QVector<mavlink_message_t> sent_;
    bool sendResult_ = true;
};

}  // namespace argus::gcs

#endif  // ARGUS_GCS_TEST_FAKE_MAVLINK_CONNECTION_HPP_
