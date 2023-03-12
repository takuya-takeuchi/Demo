#include <iomanip>
#include <iostream>
#include <string>
#include <sstream>

#include <windows.h>

std::string GetDriveSignature(const std::string& physicalDrivePath)
{
    // We do not need write permission.
    const HANDLE handle = ::CreateFileA(physicalDrivePath.c_str(),
                                        GENERIC_WRITE | GENERIC_READ,
                                        FILE_SHARE_READ | FILE_SHARE_WRITE,
                                        NULL,
                                        OPEN_EXISTING,
                                        0,
                                        NULL);
    if (handle == INVALID_HANDLE_VALUE)
        return {};

    PPARTITION_INFORMATION partitionInformation = {};
    DWORD returnedPartitionInformationSize = 0;
    // On Windows, Max parting of MBR is 4 (4 primary partitions or 3 primary partitions + 1 extended partition)
    DWORD size = sizeof(DRIVE_LAYOUT_INFORMATION) + sizeof(PARTITION_INFORMATION) * 4;
    DRIVE_LAYOUT_INFORMATION* pDriveLayoutInformation = (DRIVE_LAYOUT_INFORMATION*)malloc(size);
    std::unique_ptr<std::remove_pointer<DRIVE_LAYOUT_INFORMATION*>::type, void(*)(DRIVE_LAYOUT_INFORMATION*)> driveLayoutInformation
    {
        pDriveLayoutInformation, [](DRIVE_LAYOUT_INFORMATION* p)
        {
            free(p);
        }
    };
    const BOOL ret = DeviceIoControl(handle,
                                     IOCTL_DISK_GET_DRIVE_LAYOUT,
                                     NULL,
                                     0,
                                     driveLayoutInformation.get(),
                                     size,
                                     &returnedPartitionInformationSize,
                                     NULL);
    CloseHandle(handle);
    if (!ret)
        return {};

    std::stringstream ss;
	ss << std::hex << std::setw(8)<< std::setfill('0') << (int32_t)driveLayoutInformation.get()->Signature;
    return ss.str();
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 2)
    {
        std::cout << "Usage:" << std::endl;
        std::cout << "\tTest <physical drive name>" << std::endl;
        std::cout << std::endl;
        std::cout << "Example:" << std::endl;
        std::cout << "\tTest \\\\.\\PhysicalDrive0" << std::endl;
        return -1;
    }

    const auto physicalDrivePath = argv[1];
    const auto driveSignature = GetDriveSignature(physicalDrivePath);
    std::cout << "MBR Drive Signature: " << driveSignature << std::endl;

    return 0;
}