#include <iostream>

#include <emmintrin.h>

int32_t main(int32_t argc, const char** argv)
{
    const size_t length = 8;
    uint8_t src[length];
    uint8_t dst[length];
    std::memset(&src[0], 0, sizeof(uint8_t) * length);
    for (auto i = 0; i < length; i++)
        src[i] = (uint8_t)i;
    
    // Get lower 24bit and store as 32bit
    std::memset(&dst[0], 0, sizeof(uint8_t) * length);
    for (auto i = 0; i < length / 4; i++)
    {
        __m128i pixel32 = _mm_loadu_si128((__m128i*)(&src[0] + i * 4));
        __m128i pixel24 = _mm_and_si128(pixel32, _mm_set1_epi32(0x00ffffff));
        _mm_storeu_si128((__m128i*)(&dst[0] + i * 4), pixel24);            
    }

    std::cout << "[Info] Get lower 24bit and store as 32bit" << std::endl;
    for (auto i = 0; i < length; i++)
        std::cout << (int32_t)src[i] << " : " << (int32_t)dst[i] << std::endl;
    
    // Get lower 24bit and store as 24bit
    std::memset(&dst[0], 0, sizeof(uint8_t) * length);
    for (auto i = 0; i < length / 4; i++)
    {
        __m128i pixel32 = _mm_loadu_si128((__m128i*)(&src[0] + i * 4));
        __m128i pixel24 = _mm_and_si128(pixel32, _mm_set1_epi32(0x00ffffff));
        _mm_storeu_si128((__m128i*)(&dst[0] + i * 3), pixel24);            
    }

    std::cout << "[Info] Get lower 24bit and store as 24bit" << std::endl;
    for (auto i = 0; i < length; i++)
        std::cout << (int32_t)src[i] << " : " << (int32_t)dst[i] << std::endl;

    return 0;
}