#include <iostream>
#include <string>
#include <sstream>

#include <windows.h>
#include <sddl.h>

std::string GetComputerSecurityIdentifier()
{
    char computerName[MAX_COMPUTERNAME_LENGTH + 2];
    DWORD computerNameSize = sizeof(computerName);
    if (!GetComputerNameA(computerName, &computerNameSize))
        return {};

    // Get buffer size
    DWORD sidLength = 0;
    char domain[256];
    DWORD domainLength = sizeof(domain);
    SID_NAME_USE snu;
    LookupAccountNameA(NULL,
                       computerName,
                       NULL,
                       &sidLength,
                       domain,
                       &domainLength,
                       &snu);
    
    const HANDLE handle = GetProcessHeap();
    if (handle == NULL)
        return {};

    PSID pSid = (PSID)HeapAlloc(handle, 0, sidLength);
    if (pSid == NULL)
        return {};

    if (!LookupAccountNameA(NULL,
                            computerName,
                            pSid,
                            &sidLength,
                            domain,
                            &domainLength,
                            &snu))
        return {};

    char *stringSid;
    if (!ConvertSidToStringSidA(pSid, &stringSid))
    {
        HeapFree(handle, 0, pSid);
        return {};
    }

    HeapFree(handle, 0, pSid);

    const std::string sidString = std::string(stringSid);
    LocalFree(stringSid);

    return sidString;
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 1)
    {
        std::cout << "Usage:" << std::endl;
        std::cout << "\tTest" << std::endl;
        return -1;
    }

    const auto computerSid = GetComputerSecurityIdentifier();
    std::cout << "Computer SID: " << computerSid << std::endl;

    return 0;
}