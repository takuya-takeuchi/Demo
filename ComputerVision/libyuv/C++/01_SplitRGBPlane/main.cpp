#include <cstdint>
#include <fstream>
#include <istream>
#include <iterator>
#include <vector>

#include <benchmark/benchmark.h>
#include "libyuv/cpu_id.h"
#include "libyuv/planar_functions.h"

#define IMG_WIDTH 1920
#define IMG_HEIGHT 1080
#define IMG_CHANNELS 3

void SplitBGR24Planar(const uint8_t* data,
                      int width,
                      int height,
                      int stride,
                      int dstStride,
                      uint8_t* b8,
                      uint8_t* g8,
                      uint8_t* r8)
{
    for (int y = 0; y < height; ++y)
    {
        const uint8_t* s = data + y * stride;
        uint8_t* out_b = b8 + y * dstStride;
        uint8_t* out_g = g8 + y * dstStride;
        uint8_t* out_r = r8 + y * dstStride;

        for (int x = 0; x < width; ++x)
        {
            const uint8_t B = s[3 * x + 0];
            const uint8_t G = s[3 * x + 1];
            const uint8_t R = s[3 * x + 2];

            out_b[x] = B;
            out_g[x] = G;
            out_r[x] = R;
        }
    }
}

void SplitBGR24PlanarByLibYuv(const uint8_t* data,
                              int width,
                              int height,
                              int stride,
                              int dstStride,
                              uint8_t* b8,
                              uint8_t* g8,
                              uint8_t* r8)
{
    libyuv::SplitRGBPlane(data,
                          stride,
                          r8,
                          dstStride,
                          g8,
                          dstStride,
                          b8,
                          dstStride,
                          width,
                          height);
}

static void BM_SplitBGR24Planar(benchmark::State &state)
{
    std::vector<uint8_t> buf;
    buf.resize(IMG_WIDTH * IMG_HEIGHT * IMG_CHANNELS);
    std::vector<uint8_t> b8;
    b8.resize(IMG_WIDTH * IMG_HEIGHT);
    std::vector<uint8_t> g8;
    g8.resize(IMG_WIDTH * IMG_HEIGHT);
    std::vector<uint8_t> r8;
    r8.resize(IMG_WIDTH * IMG_HEIGHT);

    for (auto _ : state)
    {
        benchmark::DoNotOptimize(buf.data());
        benchmark::DoNotOptimize(b8.data());
        benchmark::DoNotOptimize(g8.data());
        benchmark::DoNotOptimize(r8.data());

        SplitBGR24Planar(buf.data(),
                         IMG_WIDTH,
                         IMG_HEIGHT,
                         IMG_WIDTH * IMG_CHANNELS,
                         IMG_WIDTH,
                         b8.data(),
                         g8.data(),
                         r8.data());

        benchmark::ClobberMemory();
    }
}

static void BM_SplitBGR24PlanarByLibYuvAVX2OFF(benchmark::State &state)
{
    libyuv::MaskCpuFlags(-1);
    libyuv::MaskCpuFlags(~libyuv::kCpuHasAVX2);

    std::vector<uint8_t> buf;
    buf.resize(IMG_WIDTH * IMG_HEIGHT * IMG_CHANNELS);
    std::vector<uint8_t> b8;
    b8.resize(IMG_WIDTH * IMG_HEIGHT);
    std::vector<uint8_t> g8;
    g8.resize(IMG_WIDTH * IMG_HEIGHT);
    std::vector<uint8_t> r8;
    r8.resize(IMG_WIDTH * IMG_HEIGHT);

    for (auto _ : state)
    {
        benchmark::DoNotOptimize(buf.data());
        benchmark::DoNotOptimize(b8.data());
        benchmark::DoNotOptimize(g8.data());
        benchmark::DoNotOptimize(r8.data());

        SplitBGR24PlanarByLibYuv(buf.data(),
                                 IMG_WIDTH,
                                 IMG_HEIGHT,
                                 IMG_WIDTH * IMG_CHANNELS,
                                 IMG_WIDTH,
                                 b8.data(),
                                 g8.data(),
                                 r8.data());

        benchmark::ClobberMemory();
    }
}

static void BM_SplitBGR24PlanarByLibYuvAVX2ON(benchmark::State &state)
{
    libyuv::MaskCpuFlags(-1);

    std::vector<uint8_t> buf;
    buf.resize(IMG_WIDTH * IMG_HEIGHT * IMG_CHANNELS);
    std::vector<uint8_t> b8;
    b8.resize(IMG_WIDTH * IMG_HEIGHT);
    std::vector<uint8_t> g8;
    g8.resize(IMG_WIDTH * IMG_HEIGHT);
    std::vector<uint8_t> r8;
    r8.resize(IMG_WIDTH * IMG_HEIGHT);

    for (auto _ : state)
    {
        benchmark::DoNotOptimize(buf.data());
        benchmark::DoNotOptimize(b8.data());
        benchmark::DoNotOptimize(g8.data());
        benchmark::DoNotOptimize(r8.data());

        SplitBGR24PlanarByLibYuv(buf.data(),
                                 IMG_WIDTH,
                                 IMG_HEIGHT,
                                 IMG_WIDTH * IMG_CHANNELS,
                                 IMG_WIDTH,
                                 b8.data(),
                                 g8.data(),
                                 r8.data());

        benchmark::ClobberMemory();
    }
}

BENCHMARK(BM_SplitBGR24Planar)->Unit(benchmark::kMillisecond)->MinTime(2.0)->Repetitions(5)->ReportAggregatesOnly(true);
BENCHMARK(BM_SplitBGR24PlanarByLibYuvAVX2OFF)->Unit(benchmark::kMillisecond)->MinTime(2.0)->Repetitions(5)->ReportAggregatesOnly(true);
BENCHMARK(BM_SplitBGR24PlanarByLibYuvAVX2ON)->Unit(benchmark::kMillisecond)->MinTime(2.0)->Repetitions(5)->ReportAggregatesOnly(true);

BENCHMARK_MAIN();