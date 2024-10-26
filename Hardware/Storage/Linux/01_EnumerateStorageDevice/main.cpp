#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

#include <libudev.h>

void EnumerateStorageDevices()
{
    std::unique_ptr<udev, decltype(&udev_unref)> udev(udev_new(), udev_unref);
    if (!udev)
    {
        std::cerr << "Can't create udev" << std::endl;
        return;
    }

    std::unique_ptr<udev_enumerate, decltype(&udev_enumerate_unref)> enumerate(udev_enumerate_new(udev.get()), udev_enumerate_unref);
    if (!enumerate)
    {
        std::cerr << "Can't create udev enumerate" << std::endl;
        return;
    }

    udev_enumerate_add_match_subsystem(enumerate.get(), "block");
    udev_enumerate_scan_devices(enumerate.get());

    struct udev_list_entry *devices = udev_enumerate_get_list_entry(enumerate.get());
    struct udev_list_entry *deviceListEntry;

    udev_list_entry_foreach(deviceListEntry, devices)
    {
        const char *path = udev_list_entry_get_name(deviceListEntry);

        std::unique_ptr<udev_device, decltype(&udev_device_unref)> dev(udev_device_new_from_syspath(udev.get(), path), udev_device_unref);
        if (!dev)
            continue;

        std::cout << "Device Node Path: " << udev_device_get_devnode(dev.get());
        std::cout << ", Device Subsystem: " << udev_device_get_subsystem(dev.get());
        std::cout << ", Device Type: " << udev_device_get_devtype(dev.get()) << std::endl;
    }
}

int32_t main(int32_t argc, const char** argv)
{
    EnumerateStorageDevices();
    return 0;
}