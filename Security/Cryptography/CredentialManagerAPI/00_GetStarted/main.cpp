#include <iostream>
#include <sstream>
#include <string>

#include <windows.h>
#include <wincred.h>

#define MODE_GET    0
#define MODE_SAVE   1
#define MODE_DELETE 2

bool SaveCredential(const std::wstring& targetName, const std::wstring& userName, const std::wstring& password)
{
    CREDENTIALW cred = { 0 };
    cred.Type = CRED_TYPE_GENERIC;
    cred.TargetName = const_cast<LPWSTR>(targetName.c_str());
    cred.UserName = const_cast<LPWSTR>(userName.c_str());
    cred.CredentialBlobSize = static_cast<DWORD>(password.size() * sizeof(wchar_t));
    cred.CredentialBlob = (LPBYTE)password.c_str();
    cred.Persist = CRED_PERSIST_LOCAL_MACHINE;
    cred.AttributeCount = 0;

    return CredWriteW(&cred, 0);
}

bool GetCredential(const std::wstring& targetName, std::wstring& userName, std::wstring& password)
{
    PCREDENTIALW pCredential = nullptr;
    if (!CredReadW(targetName.c_str(), CRED_TYPE_GENERIC, 0, &pCredential))
        return false;

    userName = pCredential->UserName;
    password.assign((wchar_t*)pCredential->CredentialBlob, pCredential->CredentialBlobSize / sizeof(wchar_t));
    CredFree(pCredential);

    return true;
}

bool DeleteCredential(const std::wstring& targetName)
{
    return CredDeleteW(targetName.c_str(), CRED_TYPE_GENERIC, 0);
}

void OutputErrorFormatMessage(DWORD error)
{
    wchar_t lpBuffer[512];
    setlocale(LC_ALL, setlocale(LC_CTYPE, ""));

    LPWSTR errorMessage = nullptr;
    const DWORD ret = FormatMessageW(
        FORMAT_MESSAGE_FROM_SYSTEM,
        nullptr,
        error,
        MAKELANGID(LANG_ENGLISH, SUBLANG_ENGLISH_US), // If want to show your language, set MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        lpBuffer,
        sizeof(lpBuffer)/sizeof(lpBuffer[0]),
        nullptr
    );
    
    if (ret == 0)
        std::wcerr << L"ErrorCode: " << error << std::endl;
    else
        std::wcerr << L"ErrorCode: " << error << ", Error: " << lpBuffer << std::endl;
}

int32_t wmain(int32_t argc, const wchar_t** argv)
{
    if (argc != 3 && argc != 5)
    {
        std::cerr << "[Error] Demo <mode: 0 is get> <target name>" << std::endl;
        std::cerr << "        Demo <mode: 1 is save> <target name> <user name> <password>" << std::endl;
        std::cerr << "        Demo <mode: 2 is delete> <target name>" << std::endl;
        return -1;
    }

    int mode;
    std::wstring str = argv[1];
    std::wistringstream wiss(str);
    wiss >> mode;
    if (wiss.fail() || (mode != MODE_GET && mode != MODE_SAVE && mode != MODE_DELETE))
    {
        std::cerr << "[Error] " << argv[1] << " is not integer, " << MODE_GET << ", " << MODE_SAVE << " or " << MODE_DELETE << std::endl;
        return -1;
    }

    switch (mode)
    {
        case MODE_GET:
            {
                std::wstring targetName = argv[2];

                std::wstring retrievedUserName, retrievedPassword;
                if (!GetCredential(targetName, retrievedUserName, retrievedPassword))
                {
                    const DWORD errorCode = GetLastError();
                    std::wcerr << L"[Error] Failed to retrieve credential. ";
                    OutputErrorFormatMessage(errorCode);
                    return -1;
                }

                std::wcout << L"[Info] Retrieved Username: " << retrievedUserName << std::endl;
                std::wcout << L"[Info] Retrieved Password: " << retrievedPassword << std::endl;
            }
            break;
        case MODE_SAVE:
            {
                std::wstring targetName = argv[2];
                std::wstring userName = argv[3];
                std::wstring password = argv[4];

                if (!SaveCredential(targetName, userName, password))
                {
                    const DWORD errorCode = GetLastError();
                    std::wcerr << L"[Error] Failed to save credential. ";
                    OutputErrorFormatMessage(errorCode);
                    return -1;
                }

                std::wcout << L"[Info] Credential saved successfully." << std::endl;
            }
            break;
        case MODE_DELETE:
            {
                std::wstring targetName = argv[2];

                if (!DeleteCredential(targetName))
                {
                    const DWORD errorCode = GetLastError();
                    std::wcerr << L"[Error] Failed to delete credential. ";
                    OutputErrorFormatMessage(errorCode);
                    return -1;
                }

                std::wcout << L"[Info] Credential deleted successfully." << std::endl;
            }
            break;
    }

    return 0;
}