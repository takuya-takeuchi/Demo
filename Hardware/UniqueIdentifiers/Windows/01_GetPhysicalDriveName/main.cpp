#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <windows.h>

std::vector<int32_t> GetExtentsFromPath(const std::string& drive_name)
{
    std::vector<int32_t> extents;

    char mountPoint[1024];
    if (!GetVolumePathNameA(drive_name.c_str(),
                            mountPoint,
                            ARRAYSIZE(mountPoint)))
        return extents;

    char volumeName[1024];
    if (!GetVolumeNameForVolumeMountPointA(mountPoint,
                                           volumeName,
                                           ARRAYSIZE(volumeName)))
        return extents;

    // remove trailing '\\'
    std::string volume = volumeName;
    volume.resize(volume.size() - 1);

    const HANDLE h = CreateFileA(volume.c_str(),
                                 FILE_READ_ATTRIBUTES,
                                 FILE_SHARE_READ | FILE_SHARE_WRITE,
                                 NULL,
                                 OPEN_EXISTING,
                                 FILE_ATTRIBUTE_NORMAL,
                                 NULL);
    if(h == INVALID_HANDLE_VALUE)
        return extents;

    std::unique_ptr<std::remove_pointer<HANDLE>::type, void(*)(HANDLE)> volumeHandle
    {
        h, [](HANDLE handle)
        {
            CloseHandle(handle);
        }
    };

    VOLUME_DISK_EXTENTS initialBuffer = {};
    DWORD returnedSize = 0;
    const BOOL getVolumeDiskResult = DeviceIoControl(volumeHandle.get(),
                                                     IOCTL_VOLUME_GET_VOLUME_DISK_EXTENTS,
                                                     NULL,
                                                     0,
                                                     &initialBuffer,
                                                     sizeof(initialBuffer),
                                                     &returnedSize,
                                                     NULL);

    const DWORD querySizeError = GetLastError();
    if (getVolumeDiskResult != FALSE && initialBuffer.NumberOfDiskExtents == 1)
    {
        extents.push_back(initialBuffer.Extents[0].DiskNumber);
        return extents;
    }

    if (querySizeError != ERROR_MORE_DATA)
        return extents;

    const size_t bufferSize = sizeof(initialBuffer.NumberOfDiskExtents) + sizeof(initialBuffer.Extents) * initialBuffer.NumberOfDiskExtents;
    std::unique_ptr<char[]> underlaying_buffer
    {
        new char[bufferSize]{}
    };
    VOLUME_DISK_EXTENTS* queryBuffer = reinterpret_cast<VOLUME_DISK_EXTENTS *>(&underlaying_buffer[0]);
    const BOOL devideIoControlResult = DeviceIoControl(volumeHandle.get(),
                                                       IOCTL_VOLUME_GET_VOLUME_DISK_EXTENTS,
                                                       NULL,
                                                       0,
                                                       queryBuffer,
                                                       (DWORD)bufferSize,
                                                       &returnedSize,
                                                       NULL);

    const DWORD deviceDetailResultError = ::GetLastError();
    if (!!devideIoControlResult)
        for (DWORD i = 0; i < queryBuffer->NumberOfDiskExtents; ++i)
            extents.push_back(queryBuffer->Extents[i].DiskNumber);

    return extents;
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 2)
    {
        std::cout << "Usage:" << std::endl;
        std::cout << "\tTest <drive>" << std::endl;
        std::cout << std::endl;
        std::cout << "Example:" << std::endl;
        std::cout << "\tTest C:" << std::endl;
        return -1;
    }

    const auto drive = argv[1];
    const auto indecies = GetExtentsFromPath(drive);

    for (auto index : indecies)
    {        
        std::stringstream ss;
        ss << "\\\\.\\PhysicalDrive";
        ss << index;
        std::cout << ss.str() << std::endl;
    }

    return 0;
}