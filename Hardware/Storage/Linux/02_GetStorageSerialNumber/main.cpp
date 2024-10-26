#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <libudev.h>

struct UdevDeleter {
    void operator()(udev* ptr) const { udev_unref(ptr); }
    void operator()(udev_device* ptr) const { udev_device_unref(ptr); }
};

void ShowDiskSerialNumber(const std::string& devNode)
{
    std::unique_ptr<udev, decltype(&udev_unref)> udev(udev_new(), udev_unref);
    if (!udev)
    {
        std::cerr << "Can't create udev" << std::endl;
        return;
    }

    std::unique_ptr<udev_device, UdevDeleter> dev(
        udev_device_new_from_subsystem_sysname(udev.get(), "block", devNode.c_str())
    );
    if (!dev)
    {
        std::cerr << "Cannot find device: " << devNode << std::endl;
        return;
    }

    const char* serial = udev_device_get_property_value(dev.get(), "ID_SERIAL");
    const char* serialShort = udev_device_get_property_value(dev.get(), "ID_SERIAL_SHORT");
    std::cout << "      ID_SERIAL: " << serial << std::endl;
    std::cout << "ID_SERIAL_SHORT: " << serialShort << std::endl;
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 2)
    {
        std::cout << "[Error] Demo <device node>" << std::endl;
        return -1;
    }

    ShowDiskSerialNumber(argv[1]);
    return 0;
}