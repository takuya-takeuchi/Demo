#include <iostream>

// override SODIUM_EXPORT if use libsodium as static
#define SODIUM_DLL_EXPORT
#include <sodium/core.h>

int main()
{    
    const int32_t ret = sodium_init();
    if (ret < 0)
    {
        std::cout << "[Error] libsodium couldn't be initialized with sodium_init()" << std::endl;
    }
    else
    {
        std::cout << "[Info] sodium_init: " << ret << std::endl;
    }
}