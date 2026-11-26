#include "argus/gcs/rtsp/rtsp_receiver.hpp"

namespace argus::gcs::rtsp {

// 소멸자를 .cpp 에 두어 vtable 과 moc 생성물이 이 라이브러리 안에 하나만 생기게 한다
IRtspReceiver::~IRtspReceiver() = default;

}  // namespace argus::gcs::rtsp
