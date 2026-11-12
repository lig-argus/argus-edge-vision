// 테스트용 IStreamSink. push() 로 받은 프레임과 메타데이터 문서를 받은 순서대로 보관한다
#ifndef ARGUS_VIDEO_SENDER_TEST_FAKE_STREAM_SINK_HPP_
#define ARGUS_VIDEO_SENDER_TEST_FAKE_STREAM_SINK_HPP_

#include <string>
#include <string_view>
#include <vector>

#include "argus/perception_api/types.hpp"
#include "argus/video_sender/stream_sink.hpp"

namespace argus::video_sender {

class FakeStreamSink : public IStreamSink {
public:
    struct Pushed {
        perception_api::Frame frame;
        std::string metadata_document;
    };

    void push(const perception_api::Frame& frame, std::string_view metadata_document) override {
        pushed_.push_back({frame, std::string{metadata_document}});
    }

    // 테스트 확인용: push() 로 받은 것
    const std::vector<Pushed>& pushed() const { return pushed_; }

private:
    std::vector<Pushed> pushed_;
};

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_TEST_FAKE_STREAM_SINK_HPP_
