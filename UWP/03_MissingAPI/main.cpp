#include <iostream>

#include <Windows.h>
#include <winrt/Windows.Foundation.h>

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

int main()
{
    winrt::init_apartment();

    GROUP_AFFINITY thread_affinity = {};
    thread_affinity.Group = static_cast<WORD>(0);
    thread_affinity.Mask = 0;

    if (SetThreadGroupAffinity(GetCurrentThread(), &thread_affinity, nullptr))
    {
        std::wcout << L"SetThreadGroupAffinity done for thread: " << GetCurrentThreadId()
                   << ", group_id: " << thread_affinity.Group
                   << ", mask: " << thread_affinity.Mask;
    }
    else
    {
        const auto error_code = GetLastError();
        OutputErrorFormatMessage(error_code);
    }

    winrt::uninit_apartment();

    return 0;
}