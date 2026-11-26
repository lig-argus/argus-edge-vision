// RPi 안 DDS 구현(DdsBus)의 설정 기본값. DdsBus 를 만들 때 사용하며 bus 인터페이스에는 나오지
// 않는다 (설정은 구현별. bus/bus.hpp). 실제 값은 각 실행 파일이 실행 인자로 받고, 없을 때만 이 값을
// 사용한다. 규약: docs/spec/process_interfaces.md 3장
#ifndef ARGUS_PROTOCOL_DDS_SETTINGS_HPP_
#define ARGUS_PROTOCOL_DDS_SETTINGS_HPP_

#include <cstdint>

namespace argus::protocol {

// DDS domain. 같은 domain 의 DdsBus 끼리만 서로를 찾아 통신하므로 RPi 의 모든 프로세스가 같은 값을
// 사용한다. 0 은 DDS 의 기본 domain 이다
inline constexpr std::uint32_t kDefaultDdsDomain = 0;

}  // namespace argus::protocol

#endif  // ARGUS_PROTOCOL_DDS_SETTINGS_HPP_
