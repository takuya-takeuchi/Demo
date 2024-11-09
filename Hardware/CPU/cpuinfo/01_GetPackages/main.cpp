#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

#include <cpuinfo.h>

#if defined(WINAPI_FAMILY) && (WINAPI_FAMILY == WINAPI_FAMILY_APP)
#define UWP
#include <Windows.h>
#include <winrt/Windows.Foundation.h>
#endif

#if defined(UWP)
int main()
#else
int32_t main(int32_t argc, const char** argv)
#endif
{
#if defined(UWP)
    winrt::init_apartment();
#endif

    if (!cpuinfo_initialize())
    {
        std::cout << "[Error] Failed cpuinfo_initialize" << std::endl;
        return -1;
    }

    const uint32_t packages_count = cpuinfo_get_packages_count();
    for (auto index = 0u; index < packages_count; index++)
    {
        const cpuinfo_package* package = cpuinfo_get_package(0);
        if (package == nullptr)
        {
            std::cout << "[Error] Failed cpuinfo_get_package(" << index << ")" << std::endl;
            continue;
        }

        std::cout << "[info] name: " << package->name << std::endl;
    }

    std::cout << "[info] Wait to close console..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

#if defined(UWP)
    winrt::uninit_apartment();
#endif

    return 0;
}