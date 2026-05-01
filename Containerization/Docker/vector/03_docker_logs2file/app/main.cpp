#include <chrono>
#include <iostream>
#include <thread>

int32_t main(int32_t argc, const char **argv)
{
    const std::chrono::seconds interval(1);

    auto next_tick = std::chrono::steady_clock::now();

    std::cout << "[Info] Starting loop" << std::endl;

    for (int32_t i = 0; i < 5; ++i)
    {
        std::cout << "[Info] Tick " << i + 1 << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        next_tick += interval;
        std::this_thread::sleep_until(next_tick);
    }

    std::cout << "[Info] Finished loop" << std::endl;

    return 0;
}
