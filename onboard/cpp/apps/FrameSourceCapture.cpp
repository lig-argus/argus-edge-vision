// Existing OpenCV decoding in an owned C++ helper isolates potentially stuck V4L2 reads.
#include "argus/ports/IClock.hpp"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <unistd.h>
#include <vector>
static void put(std::vector<std::uint8_t> &out, std::uint64_t v, unsigned n) {
    for (unsigned i = 0; i < n; ++i)
        out.push_back((v >> (8 * i)) & 255);
}
static void write_all(const std::vector<std::uint8_t> &data) {
    std::size_t offset{};
    while (offset < data.size()) {
        auto n = write(STDOUT_FILENO, data.data() + offset, data.size() - offset);
        if (n <= 0)
            throw std::runtime_error("capture pipe closed");
        offset += n;
    }
}
int main(int argc, char **argv) {
    try {
        if (argc != 7)
            throw std::invalid_argument("FrameSourceCapture requires kind,path,width,height,fps,fourcc");
        bool replay = std::string(argv[1]) == "replay";
        std::string path = argv[2], fourcc = argv[6];
        cv::VideoCapture camera;
        if (replay)
            camera.open(path, cv::CAP_ANY);
        else {
            bool index = !path.empty() &&
                         std::all_of(path.begin(), path.end(), [](char c) { return c >= '0' && c <= '9'; });
            if (index)
                camera.open(std::stoi(path), cv::CAP_V4L2);
            else
                camera.open(path, cv::CAP_V4L2);
            camera.set(cv::CAP_PROP_FRAME_WIDTH, std::stoi(argv[3]));
            camera.set(cv::CAP_PROP_FRAME_HEIGHT, std::stoi(argv[4]));
            camera.set(cv::CAP_PROP_FPS, std::stoi(argv[5]));
            camera.set(cv::CAP_PROP_BUFFERSIZE, 1);
            camera.set(cv::CAP_PROP_FOURCC,
                       cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]));
        }
        if (!camera.isOpened())
            throw std::runtime_error("frame source cannot open " + path);
        argus::SystemClock clock;
        std::uint64_t sequence{};
        unsigned failures{};
        for (;;) {
            cv::Mat image;
            if (!camera.read(image) || image.empty()) {
                if (replay) {
                    std::vector<std::uint8_t> end(40);
                    std::memcpy(end.data(), "END1", 4);
                    write_all(end);
                    break;
                }
                if (++failures >= 10)
                    throw std::runtime_error("camera returned 10 consecutive empty frames");
                continue;
            }
            failures = 0;
            auto stamp = clock.now().monotonic_ns;
            if (image.type() != CV_8UC3)
                throw std::runtime_error("source must supply BGR8; no RAW truncation");
            if (!image.isContinuous())
                image = image.clone();
            std::vector<std::uint8_t> packet{'C', 'V', 'A', '1'};
            put(packet, 1, 2);
            put(packet, 40, 2);
            put(packet, image.cols, 4);
            put(packet, image.rows, 4);
            put(packet, 24, 2);
            put(packet, 0, 2);
            put(packet, image.total() * 3, 4);
            put(packet, sequence++, 8);
            put(packet, stamp, 8);
            packet.insert(packet.end(), image.data, image.data + image.total() * 3);
            write_all(packet);
        }
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FrameSourceCapture: " << e.what() << '\n';
        return 1;
    }
}
