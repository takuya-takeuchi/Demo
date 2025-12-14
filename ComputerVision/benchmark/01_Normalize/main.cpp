#include <cstdint>
#include <fstream>
#include <istream>
#include <iterator>
#include <vector>

#include <benchmark/benchmark.h>

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

static inline void DeinterleaveBGR24_16px_SSSE3(const uint8_t* s, uint8_t* b, uint8_t* g, uint8_t* r)
{
    // Input layout (48 bytes):
    // [B0 G0 R0  B1 G1 R1  ...  B15 G15 R15]

    // Load 48 bytes into 3 x 16-byte registers.
    const __m128i x0 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(s +  0)); // bytes  0..15
    const __m128i x1 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(s + 16)); // bytes 16..31
    const __m128i x2 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(s + 32)); // bytes 32..47

    const __m128i Z = _mm_set1_epi8(static_cast<char>(0x80)); // pshufb: 0x80 => zero

    // ---- B channel ----
    // Bk comes from source index 3*k.
    // x0 provides B0..B5  (src 0,3,6,9,12,15) -> dst 0..5
    // x1 provides B6..B10 (src 18,21,24,27,30) -> dst 6..10 (x1 idx 2,5,8,11,14)
    // x2 provides B11..B15(src 33,36,39,42,45) -> dst 11..15 (x2 idx 1,4,7,10,13)
    const __m128i mb0 = _mm_setr_epi8(
        0, 3, 6, 9, 12, 15,
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80);

    const __m128i mb1 = _mm_setr_epi8(
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,
        2, 5, 8, 11, 14,
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80);

    const __m128i mb2 = _mm_setr_epi8(
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,
        1, 4, 7, 10, 13);

    __m128i vb = _mm_or_si128(_mm_shuffle_epi8(x0, mb0), _mm_shuffle_epi8(x1, mb1));
    vb = _mm_or_si128(vb, _mm_shuffle_epi8(x2, mb2));

    // ---- G channel ----
    // Gk comes from source index 3*k+1.
    // x0 provides G0..G4  (src 1,4,7,10,13) -> dst 0..4
    // x1 provides G5..G10 (src 16,19,22,25,28,31) -> dst 5..10 (x1 idx 0,3,6,9,12,15)
    // x2 provides G11..G15(src 34,37,40,43,46) -> dst 11..15 (x2 idx 2,5,8,11,14)
    const __m128i mg0 = _mm_setr_epi8(
        1, 4, 7, 10, 13,
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80);

    const __m128i mg1 = _mm_setr_epi8(
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,
        0, 3, 6, 9, 12, 15,
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80);

    const __m128i mg2 = _mm_setr_epi8(
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,
        2, 5, 8, 11, 14);

    __m128i vg = _mm_or_si128(_mm_shuffle_epi8(x0, mg0), _mm_shuffle_epi8(x1, mg1));
    vg = _mm_or_si128(vg, _mm_shuffle_epi8(x2, mg2));

    // ---- R channel ----
    // Rk comes from source index 3*k+2.
    // x0 provides R0..R4  (src 2,5,8,11,14) -> dst 0..4
    // x1 provides R5..R9  (src 17,20,23,26,29) -> dst 5..9 (x1 idx 1,4,7,10,13)
    // x2 provides R10..R15(src 32,35,38,41,44,47) -> dst 10..15 (x2 idx 0,3,6,9,12,15)
    const __m128i mr0 = _mm_setr_epi8(
        2, 5, 8, 11, 14,
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80);

    const __m128i mr1 = _mm_setr_epi8(
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,
        1, 4, 7, 10, 13,
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80);

    const __m128i mr2 = _mm_setr_epi8(
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,
        (char)0x80,(char)0x80,(char)0x80,(char)0x80,(char)0x80,
        0, 3, 6, 9, 12, 15);

    __m128i vr = _mm_or_si128(_mm_shuffle_epi8(x0, mr0), _mm_shuffle_epi8(x1, mr1));
    vr = _mm_or_si128(vr, _mm_shuffle_epi8(x2, mr2));

    // Store 16 bytes each (unaligned safe)
    _mm_storeu_si128(reinterpret_cast<__m128i*>(b), vb);
    _mm_storeu_si128(reinterpret_cast<__m128i*>(g), vg);
    _mm_storeu_si128(reinterpret_cast<__m128i*>(r), vr);
}

static inline void Normalize8_to_f32_AVX2(
    const uint8_t* src8,
    float* dst,
    int n,
    float mean,
    float invStd)
{
    const __m256 vScale = _mm256_set1_ps(INV255);
    const __m256 vMean  = _mm256_set1_ps(mean);
    const __m256 vInv   = _mm256_set1_ps(invStd);

    int i = 0;
    for (; i + 8 <= n; i += 8)
    {
        __m128i u8  = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(src8 + i));
        __m256i i32 = _mm256_cvtepu8_epi32(u8);
        __m256 f32  = _mm256_cvtepi32_ps(i32);

        __m256 x = _mm256_mul_ps(f32, vScale);
        x = _mm256_mul_ps(_mm256_sub_ps(x, vMean), vInv);
        _mm256_storeu_ps(dst + i, x);
    }

    for (; i < n; ++i)
    {
        const float x = src8[i] * INV255;
        dst[i] = (x - mean) * invStd;
    }
}

void AVX2(const uint8_t* data,
          int width,
          int height,
          int stride,
          int dstStride,
          uint8_t* b8,
          uint8_t* g8,
          uint8_t* r8,
          float* bf,
          float* gf,
          float* rf)
{
    for (int y = 0; y < height; ++y)
    {
        const uint8_t* s = data + y * stride;
        uint8_t* pb8 = b8 + y * dstStride;
        uint8_t* pg8 = g8 + y * dstStride;
        uint8_t* pr8 = r8 + y * dstStride;

        float* pbf = bf + y * dstStride;
        float* pgf = gf + y * dstStride;
        float* prf = rf + y * dstStride;

        int x = 0;
        for (; x + 16 <= width; x += 16)
        {
            DeinterleaveBGR24_16px_SSSE3(s + 3*x, pb8 + x, pg8 + x, pr8 + x);
        }

        for (; x < width; ++x)
        {
            pb8[x] = s[3 * x + 0];
            pg8[x] = s[3 * x + 1];
            pr8[x] = s[3 * x + 2];
        }

        Normalize8_to_f32_AVX2(pb8, pbf, width, MEAN_B, INVSTD_B);
        Normalize8_to_f32_AVX2(pg8, pgf, width, MEAN_G, INVSTD_G);
        Normalize8_to_f32_AVX2(pr8, prf, width, MEAN_G, INVSTD_G);
    }
}

static void BM_AVX2(benchmark::State &state)
{
    std::vector<uint8_t> buf;
    buf.resize(IMG_WIDTH * IMG_HEIGHT * IMG_CHANNELS);
    std::vector<uint8_t> b8;
    b8.resize(IMG_WIDTH * IMG_HEIGHT);
    std::vector<uint8_t> g8;
    g8.resize(IMG_WIDTH * IMG_HEIGHT);
    std::vector<uint8_t> r8;
    r8.resize(IMG_WIDTH * IMG_HEIGHT);
    std::vector<float> bf;
    bf.resize(IMG_WIDTH * IMG_HEIGHT);
    std::vector<float> gf;
    gf.resize(IMG_WIDTH * IMG_HEIGHT);
    std::vector<float> rf;
    rf.resize(IMG_WIDTH * IMG_HEIGHT);

    for (auto _ : state)
    {
        benchmark::DoNotOptimize(buf.data());
        benchmark::DoNotOptimize(b8.data());
        benchmark::DoNotOptimize(g8.data());
        benchmark::DoNotOptimize(r8.data());
        benchmark::DoNotOptimize(bf.data());
        benchmark::DoNotOptimize(gf.data());
        benchmark::DoNotOptimize(rf.data());

        AVX2(buf.data(),
             IMG_WIDTH,
             IMG_HEIGHT,
             IMG_WIDTH * IMG_CHANNELS,
             IMG_WIDTH,
             b8.data(),
             g8.data(),
             r8.data(),
             bf.data(),
             gf.data(),
             rf.data());

        benchmark::ClobberMemory();
    }
}

BENCHMARK(BM_Scalar)->Unit(benchmark::kMillisecond)->MinTime(2.0)->Repetitions(5)->ReportAggregatesOnly(true);
BENCHMARK(BM_AVX2)->Unit(benchmark::kMillisecond)->MinTime(2.0)->Repetitions(5)->ReportAggregatesOnly(true);

BENCHMARK_MAIN();