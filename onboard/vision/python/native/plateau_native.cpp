// Double Plateau adaptive 전처리(argus_vision/double_plateau.py)의 C++17 + OpenMP 구현.
// 출력은 Python 경로와 바이트 단위로 같다. ctypes(argus_vision/plateau_native.py)로 부른다.
// 출처: 실기체 RPi `vendor/plateau_native/plateau_native.cpp` (형식만 clang-format 으로 맞춤).
//
// 프레임마다 Python 과 같은 순서, 같은 산술로:
//   1. estimate: 2×2 간격 표본의 p1, p99 (numpy linear 백분위수)와 가로, 세로 |차이|의
//      중앙값 → 이번 프레임의 (상한, 하한) 계수
//   2. 계수의 EMA (alpha 0.15), 프레임 순서대로
//   3. plateau: 히스토그램 → 상한 자르기, 하한 채우기 → 누적 분포 → LUT → 8 비트
//
// 병렬(OpenMP)은 픽셀을 도는 부분만. 스레드마다 정수 히스토그램을 따로 채우고 나중에 더하므로
// (픽셀마다 원자 연산이나 잠금 없음) 결과가 스레드 수와 상관없다. 16384 칸의 누적 분포, LUT 와
// EMA 는 순서가 바뀌면 부동소수점 결과가 달라지므로 순차로 한다.
//
// 빌드: onboard CMake 타깃 argus_plateau_native (onboard/vision/CMakeLists.txt). -ffp-contract=off:
// 곱셈과 덧셈을 합치지 않아 double 계산이 NumPy 와 같게 반올림된다.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

// OpenMP 런타임(libgomp, libomp)이 내보내는 C 함수. omp.h 대신 직접 선언한다: CI 의
// clang-tidy(pip 배포판)에는 omp.h 가 없어 include 하면 검사가 'omp.h' file not found 로 실패한다.
// #pragma omp 와 링크는 CMake 의 OpenMP::OpenMP_CXX 가 맡는다.
extern "C" {
int omp_get_thread_num();
int omp_get_max_threads();
}

namespace {
constexpr int kLevels = 16384;           // RAW14_LEVELS
constexpr double kRawMax = kLevels - 1;  // RAW14_MAX
constexpr double kUpperLo = 1.25, kUpperHi = 5.0, kLowerLo = 0.05, kLowerHi = 0.50;

double clip(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
double now_ms() {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// Value at sorted position k from a cumulative histogram (np.searchsorted(cum, k, side="right")).
int order_stat(const std::vector<int64_t>& cum, int64_t k) {
    return static_cast<int>(std::upper_bound(cum.begin(), cum.end(), k) - cum.begin());
}

struct Ctx {
    int w, h, threads;
    double alpha;
    bool has_state = false;
    double cur_upper = 0, cur_lower = 0;
    std::vector<uint32_t> hist_t{};  // threads x levels (full-frame histogram)
    std::vector<uint32_t> samp_t{};  // threads x levels (subsample histogram)
    std::vector<uint32_t> dx_t{},
        dy_t{};  // threads x levels (|diff| < levels because values < levels)
    std::vector<double> cdf{};
    std::vector<uint8_t> lut{};
    int levels = kLevels;
};

// numpy 1.26 _quantile(method="linear") on a float32 array, from an integer histogram.
double percentile_linear(const std::vector<int64_t>& cum, int64_t n, double q_percent) {
    const double q = q_percent / 100.0;
    const double virt = static_cast<double>(n) * q + (1.0 + q * (1.0 - 1.0 - 1.0)) - 1.0;
    int64_t prev = static_cast<int64_t>(std::floor(virt)), next = prev + 1;
    prev = std::clamp<int64_t>(prev, 0, n - 1);
    next = std::clamp<int64_t>(next, 0, n - 1);
    const auto a = static_cast<float>(order_stat(cum, prev)),
               b = static_cast<float>(order_stat(cum, next));
    const double t = virt - std::floor(virt);
    const float diff = b - a;  // float32 subtract, like NumPy
    return t >= 0.5 ? static_cast<double>(b) - static_cast<double>(diff) * (1.0 - t)
                    : static_cast<double>(a) + static_cast<double>(diff) * t;
}

// np.median of an integer-valued float32 array: middle value, or float32 mean of the two middles.
float median_from_hist(const std::vector<int64_t>& cum, int64_t n) {
    if (n % 2) {
        return static_cast<float>(order_stat(cum, n / 2));
    }
    const auto lo = static_cast<float>(order_stat(cum, n / 2 - 1)),
               hi = static_cast<float>(order_stat(cum, n / 2));
    return (lo + hi) / 2.0f;
}

template <class T>
void sum_threads(const std::vector<T>& per_thread, int threads, int len,
                 std::vector<int64_t>& out) {
    out.assign(len, 0);
    for (int t = 0; t < threads; ++t) {
        const T* p = per_thread.data() + static_cast<size_t>(t) * len;
        for (int i = 0; i < len; ++i) {
            out[i] += p[i];
        }
    }
}

void estimate(Ctx& c, const uint16_t* raw, double& upper, double& lower) {
    const int sw = (c.w + 1) / 2, sh = (c.h + 1) / 2;  // raw[::2, ::2]
    const int threads = c.threads;
    std::fill(c.samp_t.begin(), c.samp_t.end(), 0);
    std::fill(c.dx_t.begin(), c.dx_t.end(), 0);
    std::fill(c.dy_t.begin(), c.dy_t.end(), 0);
    int64_t sample_max = 0;
#pragma omp parallel num_threads(threads) reduction(max : sample_max)
    {
        const int tid = omp_get_thread_num();
        uint32_t* hs = c.samp_t.data() + static_cast<size_t>(tid) * c.levels;
        uint32_t* hx = c.dx_t.data() + static_cast<size_t>(tid) * c.levels;
        uint32_t* hy = c.dy_t.data() + static_cast<size_t>(tid) * c.levels;
#pragma omp for schedule(static)
        for (int y = 0; y < sh; ++y) {
            const uint16_t* row = raw + static_cast<size_t>(2 * y) * c.w;
            const uint16_t* next =
                (y + 1 < sh) ? raw + static_cast<size_t>(2 * (y + 1)) * c.w : nullptr;
            for (int x = 0; x < sw; ++x) {
                const size_t sx = 2 * static_cast<size_t>(x);  // raw[::2] 의 열
                const int v = row[sx];
                if (v > sample_max) {
                    sample_max = v;
                }
                if (v < c.levels) {
                    ++hs[v];
                }
                if (x + 1 < sw) {
                    ++hx[std::abs(static_cast<int>(row[sx + 2]) - v)];
                }
                if (next) {
                    ++hy[std::abs(static_cast<int>(next[sx]) - v)];
                }
            }
        }
    }
    (void)sample_max;  // pn_process already grew levels to the frame maximum
    std::vector<int64_t> cum;
    const int64_t n = static_cast<int64_t>(sw) * sh;
    sum_threads(c.samp_t, threads, c.levels, cum);
    for (int i = 1; i < c.levels; ++i) {
        cum[i] += cum[i - 1];
    }
    const double p1 = percentile_linear(cum, n, 1.0), p99 = percentile_linear(cum, n, 99.0);
    const double span = std::max(p99 - p1, 1.0);
    const double span_ratio = clip(span / kRawMax, 0.0, 1.0);
    float horizontal = 0.0f, vertical = 0.0f;
    if (sw > 1) {
        sum_threads(c.dx_t, threads, c.levels, cum);
        for (int i = 1; i < c.levels; ++i) {
            cum[i] += cum[i - 1];
        }
        horizontal = median_from_hist(cum, static_cast<int64_t>(sw - 1) * sh);
    }
    if (sh > 1) {
        sum_threads(c.dy_t, threads, c.levels, cum);
        for (int i = 1; i < c.levels; ++i) {
            cum[i] += cum[i - 1];
        }
        vertical = median_from_hist(cum, static_cast<int64_t>(sw) * (sh - 1));
    }
    const auto variation = static_cast<double>((horizontal + vertical) * 0.5f);
    const double variation_ratio = clip(variation / span, 0.0, 1.0);
    const double contrast_need = clip(1.0 - span_ratio / 0.20, 0.0, 1.0);
    const double noise_score = clip(variation_ratio / 0.12, 0.0, 1.0);
    upper = clip(2.0 + 3.0 * contrast_need - 1.25 * noise_score, kUpperLo, kUpperHi);
    lower = clip(0.08 + 0.35 * contrast_need * (1.0 - noise_score), kLowerLo, kLowerHi);
}

void plateau(Ctx& c, const uint16_t* raw, uint8_t* out, double upper_factor, double lower_factor) {
    const int64_t npx = static_cast<int64_t>(c.w) * c.h;
    const int threads = c.threads;
    std::fill(c.hist_t.begin(), c.hist_t.end(), 0);
#pragma omp parallel num_threads(threads)
    {
        uint32_t* hh = c.hist_t.data() + static_cast<size_t>(omp_get_thread_num()) * c.levels;
#pragma omp for schedule(static)
        for (int64_t i = 0; i < npx; ++i) {
            ++hh[raw[i]];
        }
    }
    std::vector<int64_t> hist;
    sum_threads(c.hist_t, threads, c.levels, hist);
    const double mean_count = static_cast<double>(npx) / kLevels;
    const double upper_cap = std::max(1.0, upper_factor * mean_count);
    const double lower = lower_factor * mean_count;
    double run = 0.0, first = 0.0;
    bool found = false;
    for (int v = 0; v < c.levels; ++v) {  // sequential, like np.cumsum
        const auto hv = static_cast<double>(hist[v]);
        double m = std::min(hv, upper_cap);
        if (hv > 0.0 && hv < lower) {
            m = lower;
        }
        run += m;
        c.cdf[v] = run;
        if (!found && m != 0.0) {
            first = run;
            found = true;
        }
    }
    const double denominator = c.cdf[c.levels - 1] - first;
    if (!found || denominator <= 0.0) {
        std::memset(out, 0, static_cast<size_t>(npx));
        return;
    }
    for (int v = 0; v < c.levels; ++v) {
        const double x = clip((c.cdf[v] - first) * 255.0 / denominator, 0.0, 255.0);
        c.lut[v] = static_cast<uint8_t>(std::nearbyint(x));  // round half to even, like np.rint
    }
    const uint8_t* lut = c.lut.data();
#pragma omp parallel for num_threads(threads) schedule(static)
    for (int64_t i = 0; i < npx; ++i) {
        out[i] = lut[raw[i]];
    }
}

void resize_levels(Ctx& c, int levels) {
    c.levels = levels;
    c.hist_t.assign(static_cast<size_t>(c.threads) * levels, 0);
    c.samp_t.assign(static_cast<size_t>(c.threads) * levels, 0);
    c.dx_t.assign(static_cast<size_t>(c.threads) * levels, 0);
    c.dy_t.assign(static_cast<size_t>(c.threads) * levels, 0);
    c.cdf.assign(levels, 0.0);
    c.lut.assign(levels, 0);
}
}  // namespace

extern "C" {
void* pn_create(int width, int height, int threads, double alpha) {
    auto* c = new Ctx{width, height, threads > 0 ? threads : omp_get_max_threads(), alpha};
    resize_levels(*c, kLevels);
    return c;
}
void pn_destroy(void* h) { delete static_cast<Ctx*>(h); }
void pn_reset(void* h) { static_cast<Ctx*>(h)->has_state = false; }
int pn_threads(void* h) { return static_cast<Ctx*>(h)->threads; }

// raw: width*height uint16 (C order). out: width*height uint8.
// factors[2] <- EMA (upper, lower) used for this frame. timings[2] <- estimate ms, plateau ms.
int pn_process(void* h, const uint16_t* raw, uint8_t* out, double* factors, double* timings) {
    Ctx& c = *static_cast<Ctx*>(h);
    const int64_t npx = int64_t(c.w) * c.h;
    int maxv = 0;
#pragma omp parallel for num_threads(c.threads) reduction(max : maxv) schedule(static)
    for (int64_t i = 0; i < npx; ++i) {
        maxv = std::max(maxv, int(raw[i]));
    }
    if (maxv >= c.levels) {
        resize_levels(c, maxv + 1);  // uint16 above RAW14 extends the histogram
    }
    const double t0 = now_ms();
    double tu = 0, tl = 0;
    estimate(c, raw, tu, tl);
    if (!c.has_state) {
        c.cur_upper = tu;
        c.cur_lower = tl;
        c.has_state = true;
    } else {
        const double a = c.alpha;
        c.cur_upper = (1 - a) * c.cur_upper + a * tu;
        c.cur_lower = (1 - a) * c.cur_lower + a * tl;
    }
    const double t1 = now_ms();
    plateau(c, raw, out, c.cur_upper, c.cur_lower);
    const double t2 = now_ms();
    if (factors) {
        factors[0] = c.cur_upper;
        factors[1] = c.cur_lower;
    }
    if (timings) {
        timings[0] = t1 - t0;
        timings[1] = t2 - t1;
    }
    return 0;
}
}
