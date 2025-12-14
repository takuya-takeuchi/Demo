#include <iostream>

#include "libyuv/cpu_id.h"

int32_t main(int32_t argc, const char** argv)
{
    libyuv::InitCpuFlags();
    const int has_avx2 = libyuv::TestCpuFlag(libyuv::kCpuHasAVX2);
    const int has_fma3 = libyuv::TestCpuFlag(libyuv::kCpuHasFMA3);
    printf("libyuv cpu: AVX2=%d FMA3=%d\n", has_avx2, has_fma3);
}