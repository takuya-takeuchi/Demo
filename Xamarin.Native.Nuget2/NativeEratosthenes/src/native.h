#ifndef _CPP_NATIVE_H_
#define _CPP_NATIVE_H_

#include "export.h"
#include <cstdint>
#include <vector>

DLLEXPORT int32_t native_get_prime_count(const int32_t N)
{
    std::vector<bool> isPrime(N + 1);
    for(int i = 0; i <= N; i++) isPrime[i] = true;

    std::vector<int> P;
    for(int i = 2; i <= N; i++)
    {
        if(isPrime[i])
        {
            for(int j = 2 * i; j <= N; j += i) isPrime[j] = false;
            P.emplace_back(i);
        }
    }

    return (int32_t)P.size();
}

#endif