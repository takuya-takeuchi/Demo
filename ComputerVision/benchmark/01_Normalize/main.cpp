#include <cstdint>
#include <fstream>
#include <istream>
#include <iterator>
#include <vector>

#include <benchmark/benchmark.h>

#if defined(_MSC_VER)
  // MSVC
  #include <intrin.h>
  #include <immintrin.h>
#else
  // GCC / Clang
  #include <immintrin.h>
  #include <tmmintrin.h>
#endif

#define IMG_WIDTH 1920
#define IMG_HEIGHT 1080
#define IMG_CHANNELS 3

static constexpr float MEAN_R = 0.485f;
static constexpr float MEAN_G = 0.456f;
static constexpr float MEAN_B = 0.406f;
static constexpr float INVSTD_R = 1.0f / 0.229f;
static constexpr float INVSTD_G = 1.0f / 0.224f;
static constexpr float INVSTD_B = 1.0f / 0.225f;

static constexpr float INV255 = 1.0f / 255.0f;

void Scalar(const uint8_t* data,
            int width,
            int height,
            int stride,
            int dstStride,
            float* b8,
            float* g8,
            float* r8)
{
    for (int y = 0; y < height; ++y)
    {
        const uint8_t* s = data + y * stride;
        float* out_b = b8 + y * dstStride;
        float* out_g = g8 + y * dstStride;
        float* out_r = r8 + y * dstStride;

        for (int x = 0; x < width; ++x)
        {
            const float b = s[3 * x + 0] * INV255;
            const float g = s[3 * x + 1] * INV255;
            const float r = s[3 * x + 2] * INV255;

            out_b[x] = (b - MEAN_B) * INVSTD_B;
            out_g[x] = (g - MEAN_G) * INVSTD_G;
            out_r[x] = (r - MEAN_R) * INVSTD_R;
        }
    }
}

static void BM_Scalar(benchmark::State &state)
{
    std::vector<uint8_t> buf;
    buf.resize(IMG_WIDTH * IMG_HEIGHT * IMG_CHANNELS);
    std::vector<float> b8;
    b8.resize(IMG_WIDTH * IMG_HEIGHT);
    std::vector<float> g8;
    g8.resize(IMG_WIDTH * IMG_HEIGHT);
    std::vector<float> r8;
    r8.resize(IMG_WIDTH * IMG_HEIGHT);

    for (auto _ : state)
    {
        benchmark::DoNotOptimize(buf.data());
        benchmark::DoNotOptimize(b8.data());
        benchmark::DoNotOptimize(g8.data());
        benchmark::DoNotOptimize(r8.data());

        Scalar(buf.data(),
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

// Gather 8 pixels of a single channel from BGR24.
// channel_offset: B=0, G=1, R=2
static inline __m256 Load8_u8_as_f32_gather_AVX2(const uint8_t* base, __m256i byteOffsets)
{
    // Gather 8x 32-bit words starting at base + offset (scale=1 byte)
    __m256i v = _mm256_i32gather_epi32(reinterpret_cast<const int*>(base), byteOffsets, 1);

    // Keep only the lowest byte of each dword
    v = _mm256_and_si256(v, _mm256_set1_epi32(0xFF));

    // u8 -> i32 -> f32
    return _mm256_cvtepi32_ps(v);
}

static inline void Normalize8_f32_AVX2(__m256 x_u8_as_f32, float mean, float invStd, float* dst)
{
    const __m256 vScale = _mm256_set1_ps(INV255);
    const __m256 vMean  = _mm256_set1_ps(mean);
    const __m256 vInv   = _mm256_set1_ps(invStd);

    __m256 x = _mm256_mul_ps(x_u8_as_f32, vScale);
    x = _mm256_mul_ps(_mm256_sub_ps(x, vMean), vInv);
    _mm256_storeu_ps(dst, x);
}

void AVX2(const uint8_t* data,
          int width,
          int height,
          int strideBytes,
          int dstStride,
          float* bf,
          float* gf,
          float* rf)
{
    const __m256i offB0 = _mm256_setr_epi32(0,  3,  6,  9, 12, 15, 18, 21);
    const __m256i offG0 = _mm256_setr_epi32(1,  4,  7, 10, 13, 16, 19, 22);
    const __m256i offR0 = _mm256_setr_epi32(2,  5,  8, 11, 14, 17, 20, 23);

    const __m256i offB1 = _mm256_setr_epi32(24, 27, 30, 33, 36, 39, 42, 45);
    const __m256i offG1 = _mm256_setr_epi32(25, 28, 31, 34, 37, 40, 43, 46);
    const __m256i offR1 = _mm256_setr_epi32(26, 29, 32, 35, 38, 41, 44, 47);

    for (int y = 0; y < height; ++y)
    {
        const uint8_t* s = data + y * strideBytes;
        float* outB = bf + y * dstStride;
        float* outG = gf + y * dstStride;
        float* outR = rf + y * dstStride;

        int x = 0;

        // Process 16 pixels per iteration (2 x 8 pixels)
        for (; x + 16 <= width; x += 16)
        {
            const uint8_t* p = s + 3 * x;

            // First 8 pixels
            __m256 b0 = Load8_u8_as_f32_gather_AVX2(p, offB0);
            __m256 g0 = Load8_u8_as_f32_gather_AVX2(p, offG0);
            __m256 r0 = Load8_u8_as_f32_gather_AVX2(p, offR0);

            Normalize8_f32_AVX2(b0, MEAN_B, INVSTD_B, outB + x + 0);
            Normalize8_f32_AVX2(g0, MEAN_G, INVSTD_G, outG + x + 0);
            Normalize8_f32_AVX2(r0, MEAN_R, INVSTD_R, outR + x + 0);

            // Next 8 pixels (same base p, different offsets)
            __m256 b1 = Load8_u8_as_f32_gather_AVX2(p, offB1);
            __m256 g1 = Load8_u8_as_f32_gather_AVX2(p, offG1);
            __m256 r1 = Load8_u8_as_f32_gather_AVX2(p, offR1);

            Normalize8_f32_AVX2(b1, MEAN_B, INVSTD_B, outB + x + 8);
            Normalize8_f32_AVX2(g1, MEAN_G, INVSTD_G, outG + x + 8);
            Normalize8_f32_AVX2(r1, MEAN_R, INVSTD_R, outR + x + 8);
        }

        // Remainder (scalar)
        for (; x < width; ++x)
        {
            const float b = s[3 * x + 0] * INV255;
            const float g = s[3 * x + 1] * INV255;
            const float r = s[3 * x + 2] * INV255;

            outB[x] = (b - MEAN_B) * INVSTD_B;
            outG[x] = (g - MEAN_G) * INVSTD_G;
            outR[x] = (r - MEAN_R) * INVSTD_R;
        }
    }
}

static void BM_AVX2(benchmark::State &state)
{
    std::vector<uint8_t> buf;
    buf.resize(IMG_WIDTH * IMG_HEIGHT * IMG_CHANNELS);
    std::vector<float> bf;
    bf.resize(IMG_WIDTH * IMG_HEIGHT);
    std::vector<float> gf;
    gf.resize(IMG_WIDTH * IMG_HEIGHT);
    std::vector<float> rf;
    rf.resize(IMG_WIDTH * IMG_HEIGHT);

    for (auto _ : state)
    {
        benchmark::DoNotOptimize(buf.data());
        benchmark::DoNotOptimize(bf.data());
        benchmark::DoNotOptimize(gf.data());
        benchmark::DoNotOptimize(rf.data());

        AVX2(buf.data(),
             IMG_WIDTH,
             IMG_HEIGHT,
             IMG_WIDTH * IMG_CHANNELS,
             IMG_WIDTH,
             bf.data(),
             gf.data(),
             rf.data());

        benchmark::ClobberMemory();
    }
}

BENCHMARK(BM_Scalar)->Unit(benchmark::kMillisecond)->MinTime(2.0)->Repetitions(5)->ReportAggregatesOnly(true);
BENCHMARK(BM_AVX2)->Unit(benchmark::kMillisecond)->MinTime(2.0)->Repetitions(5)->ReportAggregatesOnly(true);

BENCHMARK_MAIN();