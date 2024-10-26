#include <cstring>
#include <iostream>
#include <iomanip>

#include <ifaddrs.h>
#include <net/if.h>
#ifdef __APPLE__
#include <net/if_dl.h>
#elif __linux__
#include <netpacket/packet.h>
#include <sys/types.h>
#endif

void PrintMacAddress(const uint8_t* addr, int len)
{
    for (int i = 0; i < len; i++)
    {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int32_t>(addr[i]);
        if (i != len - 1)
            std::cout << ":";
    }

    std::cout << std::dec << std::endl;
}

void EnumerateMacAddresses()
{
    struct ifaddrs* ifap;
    if (getifaddrs(&ifap) == -1)
    {
        std::cerr << "Error getting network interfaces." << std::endl;
        return;
    }

    for (struct ifaddrs* ifa = ifap; ifa != nullptr; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr == nullptr)
            continue;

#ifdef __APPLE__
        if (ifa->ifa_addr->sa_family != AF_LINK)
            continue;

        struct sockaddr_dl* sockaddr = (struct sockaddr_dl*)ifa->ifa_addr;
        if (sockaddr->sdl_type != IFT_ETHER)
            continue;

        uint8_t* addr = (uint8_t*)LLADDR(sockaddr);
        int len = sockaddr->sdl_alen;
#elif __linux__
        if (ifa->ifa_addr->sa_family != AF_PACKET)
            continue;

        struct sockaddr_ll* sockaddr = (struct sockaddr_ll*)ifa->ifa_addr;
        // Length of MAC address is 6
        if (sockaddr->sll_halen != 6)
            continue;

        uint8_t* addr = (uint8_t*)sockaddr->sll_addr;
        int len = sockaddr->sll_halen;
#endif

        std::cout << "Interface: " << ifa->ifa_name << " - MAC Address: ";
        PrintMacAddress(addr, len);
    }
    freeifaddrs(ifap);
}

int32_t main(int32_t argc, const char** argv)
{
    EnumerateMacAddresses();
    return 0;
}