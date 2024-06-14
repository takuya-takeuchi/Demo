#include <iostream>
#include <string>
#include <sstream>

#include <windows.h>

std::string GetLastErrorMessage(DWORD errorMessageID)
{
    if(errorMessageID == 0)
        return std::string();

    LPSTR messageBuffer = nullptr;
    size_t size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                                 NULL,
                                 errorMessageID,
                                 MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                                 (LPSTR)&messageBuffer,
                                 0,
                                 NULL);

    std::string message(messageBuffer, size);
    LocalFree(messageBuffer);
    return message;
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 2)
    {
        std::cout << "Usage:" << std::endl;
        std::cout << "\tTest <drive>" << std::endl;
        std::cout << std::endl;
        std::cout << "Example:" << std::endl;
        std::cout << "\tTest \"\\\\?\\usbstor#disk&ven_tdk_lor&prod_tf10&rev_pmap#0703448b91511325&0#{53f56307-b6bf-11d0-94f2-00a0c91efb8b}\"" << std::endl;
        return -1;
    }

    const auto drive = argv[1];
    HANDLE hDevice = CreateFile(drive, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);
    if (hDevice == INVALID_HANDLE_VALUE)
    {
        std::cerr << "Failed to open device: " << GetLastErrorMessage(GetLastError()) << std::endl;
        return -11;
    }

    DWORD bytesReturned;
    BOOL result = DeviceIoControl(hDevice, IOCTL_VOLUME_OFFLINE, NULL, 0, NULL, 0, &bytesReturned, NULL);
    if (result)
    {
        std::cout << "IOCTL_VOLUME_OFFLINE supported and executed successfully." << std::endl;
    }
    else
    {
        std::cerr << "IOCTL_VOLUME_OFFLINE failed: " << GetLastErrorMessage(GetLastError()) << std::endl;
    }

    CloseHandle(hDevice);
    return 0;
}