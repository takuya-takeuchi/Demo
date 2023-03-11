#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <windows.h>

HRESULT HasNoSeekPenalty(const std::string& physicalDrivePath)
{
    // We do not need write permission.
    const HANDLE handle = ::CreateFileA(physicalDrivePath.c_str(),
                                        FILE_READ_ATTRIBUTES,
                                        FILE_SHARE_READ | FILE_SHARE_WRITE,
                                        NULL,
                                        OPEN_EXISTING,
                                        FILE_ATTRIBUTE_NORMAL,
                                        NULL);
    if (handle == INVALID_HANDLE_VALUE)
        return E_FAIL;

    STORAGE_PROPERTY_QUERY querySeekPenalty =
    {
        StorageDeviceSeekPenaltyProperty,  // PropertyId
        PropertyStandardQuery,             // QueryType,
    };
    DEVICE_SEEK_PENALTY_DESCRIPTOR querySeekPenaltyDesc = {};

    DWORD returnedQuerySeekPenaltySize = 0;
    const BOOL querySeekPenaltyResult = DeviceIoControl(handle,
                                                        IOCTL_STORAGE_QUERY_PROPERTY,
                                                        &querySeekPenalty,
                                                        sizeof(querySeekPenalty),
                                                        &querySeekPenaltyDesc,
                                                        sizeof(querySeekPenaltyDesc),
                                                        &returnedQuerySeekPenaltySize,
                                                        NULL);
    CloseHandle(handle);
    if (!querySeekPenaltyResult)
        return E_FAIL;

    return !querySeekPenaltyDesc.IncursSeekPenalty ? S_OK : S_FALSE;
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
    switch (HasNoSeekPenalty(physicalDrivePath))
    {
        case S_OK:
            std::cout << physicalDrivePath << " has no seek penalty. It's solid state drive." << std::endl;
            break;
        case S_FALSE:
            std::cout << physicalDrivePath << " has seek penalty. It's not solid state drive." << std::endl;
            break;
        default:
            std::cout << "Failed to retrieve the status." << std::endl;
            break;
    }

    return 0;
}