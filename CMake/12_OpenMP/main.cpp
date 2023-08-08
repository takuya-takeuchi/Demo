#include <iostream>
#include <chrono>

#include <omp.h>

int32_t main(int32_t argc, const char** argv)
{
    const auto loop = atoi(argv[1]);

    const size_t length = 1024 * 1000;
    uint8_t src[length];

    auto start = std::chrono::high_resolution_clock::now();
    for (auto l = 0; l < loop; l++)
    {
        #pragma omp parallel for
        for (auto i = 0; i < length; i++)
        {
            src[i] = (uint8_t)length - i;
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << " Enable OpenMP: " << duration.count() << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    for (auto l = 0; l < loop; l++)
    {
        for (auto i = 0; i < length; i++)
        {
            src[i] = (uint8_t)length - i;
        }
    }
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Disable OpenMP: " << duration.count() << " ms" << std::endl;

    return 0;
}