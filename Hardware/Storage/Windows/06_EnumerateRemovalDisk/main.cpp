#include <iostream>

#include <windows.h>
#include <setupapi.h>
#include <cfgmgr32.h>
#include <initguid.h>
#include <devguid.h>
#include <devpkey.h>

int32_t main(int32_t argc, const char** argv)
{
    HDEVINFO deviceInfoSet = SetupDiGetClassDevs(&GUID_DEVINTERFACE_DISK, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (deviceInfoSet == INVALID_HANDLE_VALUE)
    {
        std::cerr << "Failed to get device list" << std::endl;
        return -11;
    }

    SP_DEVINFO_DATA deviceInfoData;
    deviceInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

    for (int i = 0; SetupDiEnumDeviceInfo(deviceInfoSet, i, &deviceInfoData); i++)
    {
        DWORD dataType, actualSize = 0;
        BYTE buffer[1024];

        if (!SetupDiGetDeviceRegistryProperty(deviceInfoSet, &deviceInfoData, SPDRP_REMOVAL_POLICY, &dataType, buffer, sizeof(buffer), &actualSize))
            continue;

        ULONG policy = *reinterpret_cast<PULONG>(buffer);
        if (!(policy == CM_REMOVAL_POLICY_EXPECT_SURPRISE_REMOVAL || policy == CM_REMOVAL_POLICY_EXPECT_ORDERLY_REMOVAL))
            continue;

        SP_DEVICE_INTERFACE_DATA interfaceData;
        interfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
        if (!SetupDiEnumDeviceInterfaces(deviceInfoSet, &deviceInfoData, &GUID_DEVINTERFACE_DISK, 0, &interfaceData))
            continue;

        SP_DEVICE_INTERFACE_DETAIL_DATA *detailData = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA *>(buffer);
        detailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
        if (!SetupDiGetDeviceInterfaceDetail(deviceInfoSet, &interfaceData, detailData, sizeof(buffer), NULL, NULL))
            continue;
        
        std::cout << "Found Removable Disk: " << detailData->DevicePath << std::endl;
    }

    SetupDiDestroyDeviceInfoList(deviceInfoSet);
    return 0;
}