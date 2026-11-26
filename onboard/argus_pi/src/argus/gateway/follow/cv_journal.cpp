#include "argus/gateway/follow/include/cv_journal.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <iostream>
#include <random>

namespace argus::gateway {

namespace {

constexpr std::size_t kMaxBytes = std::size_t{8} * 1024U * 1024U;

std::string random_hex() {
    static constexpr std::array<char, 16> kDigits{'0', '1', '2', '3', '4', '5', '6', '7',
                                                  '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
    std::random_device device;
    std::mt19937_64 engine(device());
    std::string out;
    for (int i = 0; i < 32; ++i) {
        out += kDigits.at(engine() & 0x0fU);
    }
    return out;
}

std::int64_t monotonic_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace

CvJournal::CvJournal(std::optional<std::filesystem::path> directory, std::size_t queue_size)
    : queue_size_(queue_size) {
    if (directory) {
        path_ = *directory / ("cv-owner-" + random_hex() + ".jsonl");
        thread_ = std::thread([this] { run(); });
    }
}

CvJournal::~CvJournal() { close(); }

void CvJournal::record(const std::string& event, const Wire& fields) {
    if (closed_ || !thread_.joinable()) {
        return;
    }
    Wire item = Wire::object();
    wire_set(item, "schema", "argus.cv.owner_diagnostic.v1");
    wire_set(item, "event", event);
    wire_set(item, "queued_monotonic_ns", monotonic_ns());
    for (const auto& field : fields.items()) {
        wire_set(item, field.key(), field.value());
    }
    const std::scoped_lock lock(mutex_);
    if (pending_.size() >= queue_size_) {
        ++dropped_;
        return;
    }
    pending_.push_back(std::move(item));
    ready_.notify_one();
}

void CvJournal::run() {
    if (!path_) {
        return;
    }
    const std::filesystem::path path = *path_;
    try {
        std::filesystem::create_directories(path.parent_path());
        // 원본 open('x'): 이미 있으면 실패
        // POSIX open()은 mode 인수 때문에 가변 인수 함수다.
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
        const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
        if (fd < 0) {
            throw std::runtime_error("cv journal open failed");
        }
        std::size_t size = 0;
        for (;;) {
            Wire item;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                if (!ready_.wait_for(lock, std::chrono::milliseconds(100),
                                     [this] { return !pending_.empty(); })) {
                    if (closed_) {
                        break;
                    }
                    continue;
                }
                item = std::move(pending_.front());
                pending_.pop_front();
                wire_set(item, "dropped_before", dropped_);
                dropped_ = 0;
            }
            const std::string line =
                item.dump(-1, ' ', false, Wire::error_handler_t::replace) + "\n";
            size += line.size();
            if (size > kMaxBytes) {
                const std::scoped_lock lock(mutex_);
                ++dropped_;
                closed_ = true;
                std::cerr << "ARGUS_CV_JOURNAL_LIMIT_REACHED"
                          << std::endl;  // NOLINT(performance-avoid-endl)
                break;
            }
            if (::write(fd, line.data(), line.size()) != static_cast<ssize_t>(line.size())) {
                ::close(fd);
                throw std::runtime_error("cv journal write failed");
            }
        }
        ::close(fd);
    } catch (const std::exception& error) {
        closed_ = true;
        std::cerr << "ARGUS_CV_JOURNAL_FAILED: " << error.what()
                  << std::endl;  // NOLINT(performance-avoid-endl)
    }
}

void CvJournal::close() {
    closed_ = true;
    ready_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
}

}  // namespace argus::gateway
