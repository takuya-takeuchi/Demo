#include <cstdlib>
#include <iostream>

#include <cpuinfo.h>

int32_t main(int32_t argc, const char** argv)
{
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
}