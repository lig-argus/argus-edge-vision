#include "argus/gcs/net/mavlink_stream_parser.hpp"

namespace argus::gcs::net {

namespace {

using mavlink_connection::LossRecord;
using mavlink_connection::MessageStatistics;
using mavlink_connection::SenderStatistics;

constexpr std::int32_t kSeqRange = 256;

// 앞으로 이보다 크게 건너뛰면 seq 가 뒤로 간 것으로 본다 (한 바퀴의 절반)
constexpr std::int32_t kMaxForwardGap = 128;

// 뒤로 간 패킷이 이만큼 연속으로 도착하면 송신자가 재시작한 것으로 보고 seq 를 다시 맞춘다.
// 순서 바뀜과 중복은 앞선 패킷 사이에 간헐적으로 섞여 도착하므로 연속되지 않는다
constexpr std::int32_t kRestartRun = 3;

// 프레임 바이트 수: MAVLink 2 는 헤더 10 + CRC 2 (+ 서명 13), MAVLink 1 은 헤더 6 + CRC 2
constexpr std::uint64_t kV2Overhead = 12;
constexpr std::uint64_t kV1Overhead = 8;

std::uint64_t frameBytes(const mavlink_message_t& message) {
    if (message.magic != MAVLINK_STX) {
        return kV1Overhead + message.len;
    }
    const bool signedFrame = (message.incompat_flags & MAVLINK_IFLAG_SIGNED) != 0U;
    return kV2Overhead + message.len + (signedFrame ? MAVLINK_SIGNATURE_BLOCK_LEN : 0U);
}

std::int32_t seqDistance(std::uint8_t from, std::uint8_t to) {
    return (static_cast<std::int32_t>(to) - static_cast<std::int32_t>(from) + kSeqRange) %
           kSeqRange;
}

}  // namespace

void MavlinkStreamParser::feed(const QByteArray& bytes, const MessageHandler& handler) {
    for (const char byte : bytes) {
        mavlink_message_t message{};
        mavlink_status_t status{};
        const std::uint8_t result = mavlink_frame_char_buffer(
            &partial_, &status_, static_cast<std::uint8_t>(byte), &message, &status);
        if (result == MAVLINK_FRAMING_OK) {
            countSender(message);
            handler(message);
        } else if (result == MAVLINK_FRAMING_BAD_CRC || result == MAVLINK_FRAMING_BAD_SIGNATURE) {
            // 방언에 없는 메시지는 CRC 를 확인할 수 없어 BAD_CRC 로 나온다. 헤더는 읽을 수 있으므로
            // 송신자 통계에는 넣고 형식 오류로는 세지 않는다
            if (mavlink_get_msg_entry(message.msgid) == nullptr) {
                ++statistics_.unknown;
                countSender(message);
            } else {
                ++statistics_.badCrc;
            }
        }
    }
}

// 송신자별 seq 로 패킷 손실, 순서 바뀜, 중복을 센다.
// - 앞으로 1~kMaxForwardGap: 정상 진행. 건너뛴 수는 손실
// - 같거나 뒤로: 최근 한 바퀴에 받은 seq 면 중복, 아니면 늦게 도착한 것(순서 바뀜)이라 손실에서
// 뺀다
// - 뒤로 간 패킷이 kRestartRun 번 연속으로 도착하면 송신자 재시작으로 보고, 그동안 센 순서 바뀜과
//   중복을 되돌린 뒤 그 seq 부터 다시 센다
void MavlinkStreamParser::countSender(const mavlink_message_t& message) {
    const std::uint16_t key = mavlink_connection::senderKey(message.sysid, message.compid);
    SenderStatistics& sender = statistics_.senders[key];
    SeqTracker& tracker = trackers_[key];
    const std::uint8_t seq = message.seq;
    const std::uint32_t msgid = message.msgid;
    const std::uint64_t bytes = frameBytes(message);
    sender.bytes += bytes;
    MessageStatistics& byMessage = sender.messages[msgid];
    ++byMessage.received;
    byMessage.bytes += bytes;
    byMessage.lastReceivedNs = clock_.nowMonotonicNs();

    if (sender.received == 0) {
        restartSequence(sender, tracker, seq);
        ++sender.received;
        sender.lastMsgid = msgid;
        return;
    }
    ++sender.received;

    const std::int32_t forward = seqDistance(sender.lastSeq, seq);
    if (forward >= 1 && forward <= kMaxForwardGap) {
        const std::int32_t missing = forward - 1;
        if (missing > 0) {
            sender.lost += static_cast<std::uint64_t>(missing);
            recordLoss(sender, seq, msgid, missing);  // lastMsgid 는 아직 건너뛰기 전의 값
        }
        for (std::int32_t step = 1; step < forward; ++step) {
            tracker.seen.at(static_cast<std::size_t>((sender.lastSeq + step) % kSeqRange)) = false;
        }
        tracker.seen.at(seq) = true;
        sender.lastSeq = seq;
        tracker.backwardRun = 0;
        tracker.runReordered = 0;
        tracker.runDuplicated = 0;
        tracker.runLostRestored = 0;
        sender.lastMsgid = msgid;
        return;
    }

    if (tracker.seen.at(seq)) {
        ++sender.duplicated;
        ++tracker.runDuplicated;
    } else {
        ++sender.reordered;
        ++tracker.runReordered;
        tracker.seen.at(seq) = true;
        if (sender.lost > 0) {  // 건너뛸 때 손실로 센 패킷이 늦게 도착했다
            --sender.lost;
            ++tracker.runLostRestored;
            recordLate(sender, seq);
        }
    }
    if (++tracker.backwardRun >= kRestartRun) {
        sender.duplicated -= tracker.runDuplicated;
        sender.reordered -= tracker.runReordered;
        sender.lost += tracker.runLostRestored;
        restartSequence(sender, tracker, seq);
    }
    sender.lastMsgid = msgid;
}

void MavlinkStreamParser::restartSequence(SenderStatistics& sender, SeqTracker& tracker,
                                          std::uint8_t seq) {
    tracker.seen.fill(false);
    tracker.seen.at(seq) = true;
    tracker.backwardRun = 0;
    tracker.runReordered = 0;
    tracker.runDuplicated = 0;
    tracker.runLostRestored = 0;
    sender.lastSeq = seq;
}

void MavlinkStreamParser::recordLoss(SenderStatistics& sender, std::uint8_t seq,
                                     std::uint32_t msgid, std::int32_t missing) const {
    LossRecord record;
    record.detectedNs = clock_.nowMonotonicNs();
    record.lastSeq = sender.lastSeq;
    record.nextSeq = seq;
    record.missing = missing;
    record.beforeMsgid = sender.lastMsgid;
    record.afterMsgid = msgid;
    sender.losses.push_back(record);
    while (sender.losses.size() > SenderStatistics::kMaxLossRecords) {
        sender.losses.pop_front();
    }
}

// 늦게 도착한 seq 가 들어 있는 가장 최근 기록의 늦게 도착한 수를 올린다. 기록은 지우지 않는다
// (순서가 바뀐 위치도 통신 통계 화면에서 보인다)
void MavlinkStreamParser::recordLate(SenderStatistics& sender, std::uint8_t seq) {
    for (auto it = sender.losses.rbegin(); it != sender.losses.rend(); ++it) {
        const std::int32_t offset = seqDistance(it->lastSeq, seq);
        const std::int32_t span = seqDistance(it->lastSeq, it->nextSeq);
        if (offset > 0 && offset < span && it->late < it->missing) {
            ++it->late;
            return;
        }
    }
}

}  // namespace argus::gcs::net
