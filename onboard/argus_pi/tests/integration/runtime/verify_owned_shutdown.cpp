// Exercise a real pipe-full child. Normal EOF cleanup must run without signals.
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <thread>

#include "argus/runtime/include/owned_process.hpp"
int main() {
    const char* marker = "udp_owned_clean_shutdown.marker";
    std::remove(marker);
    argus::OwnedProcess child;
    child.start({"python3", "-c",
                 "import sys; "
                 "sys.stdout.buffer.write(b'x'*(2*1024*1024));sys.stdout.buffer.flush();sys.stdin."
                 "buffer.read();open('udp_owned_clean_shutdown.marker','w').write('clean')"},
                true);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto started = std::chrono::steady_clock::now();
    child.stop();
    auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    std::ifstream input(marker);
    std::string result;
    input >> result;
    if (result != "clean" || elapsed > 2.0 || child.running()) {
        return 1;
    }
    std::cout << "Normal child cleanup after full stdout pipe: " << elapsed << " seconds\n";
    return 0;
}
