// SHA-256 (FIPS 180-4). 설정 출력 폴더 이름을 Python hashlib.sha256과 같게 만든다.
#pragma once

#include <string>

namespace argus::gateway {

// 입력 바이트열의 SHA-256 값을 소문자 16진수 64자로 돌려준다.
std::string sha256_hex(const std::string& data);

}  // namespace argus::gateway
