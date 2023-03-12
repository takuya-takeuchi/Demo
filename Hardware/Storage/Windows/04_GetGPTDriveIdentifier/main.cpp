#include <iomanip>
#include <iostream>
#include <string>
#include <sstream>

#include <windows.h>

std::string GetDriveSignature(const std::string& physicalDrivePath)
{
    // We do not need write permission.
    const HANDLE handle = ::CreateFileA(physicalDrivePath.c_str(),
                                        0,
                                        FILE_SHARE_READ | FILE_SHARE_WRITE,
                                        NULL,
                                        OPEN_EXISTING,
                                        0,
                                        NULL);
    if (handle == INVALID_HANDLE_VALUE)
        return {};

    DWORD returnedpartitionInformationExSize = 0;
    // On Windows, Max parting of GPT is 128
    DWORD size = sizeof(DRIVE_LAYOUT_INFORMATION_EX) + sizeof(PARTITION_INFORMATION_EX) * 128;
    DRIVE_LAYOUT_INFORMATION_EX* pDriveLayoutInformationEx = (DRIVE_LAYOUT_INFORMATION_EX*)malloc(size);
    std::unique_ptr<std::remove_pointer<DRIVE_LAYOUT_INFORMATION_EX*>::type, void(*)(DRIVE_LAYOUT_INFORMATION_EX*)> driveLayoutInformationEx
    {
        pDriveLayoutInformationEx, [](DRIVE_LAYOUT_INFORMATION_EX* p)
        {
            free(p);
        }
    };
    memset(pDriveLayoutInformationEx, 0, size);
    const BOOL ret = DeviceIoControl(handle,
                                     IOCTL_DISK_GET_DRIVE_LAYOUT_EX,
                                     NULL,
                                     0,
                                     driveLayoutInformationEx.get(),
                                     size,
                                     &returnedpartitionInformationExSize,
                                     NULL);
    CloseHandle(handle);
    if (!ret)
        return {};

    std::stringstream ss;
    ss << std::hex << std::setw(8)<< std::setfill('0') << (uint32_t)pDriveLayoutInformationEx->Gpt.DiskId.Data1;
    ss << "-";
    ss << std::hex << std::setw(4)<< std::setfill('0') << (uint32_t)pDriveLayoutInformationEx->Gpt.DiskId.Data2;
    ss << "-";
    ss << std::hex << std::setw(4)<< std::setfill('0') << (uint32_t)pDriveLayoutInformationEx->Gpt.DiskId.Data3;
    ss << "-";
    ss << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pDriveLayoutInformationEx->Gpt.DiskId.Data4[0];
    ss << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pDriveLayoutInformationEx->Gpt.DiskId.Data4[1];
    ss << "-";
    ss << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pDriveLayoutInformationEx->Gpt.DiskId.Data4[2];
    ss << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pDriveLayoutInformationEx->Gpt.DiskId.Data4[3];
    ss << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pDriveLayoutInformationEx->Gpt.DiskId.Data4[4];
    ss << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pDriveLayoutInformationEx->Gpt.DiskId.Data4[5];
    ss << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pDriveLayoutInformationEx->Gpt.DiskId.Data4[6];
    ss << std::hex << std::setw(2)<< std::setfill('0') << (uint32_t)pDriveLayoutInformationEx->Gpt.DiskId.Data4[7];
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
    std::cout << "GPT Drive Identifier: " << driveSignature << std::endl;

    return 0;
}