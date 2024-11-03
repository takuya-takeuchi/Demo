#include <cstdlib>

#if defined(WINAPI_FAMILY) && (WINAPI_FAMILY == WINAPI_FAMILY_APP)
#include <sstream>
#include <Windows.h>

void WriteToConsole(const std::wstring& message)
{
    OutputDebugStringW(message.c_str());
}

template <typename T>
void Output(const T& value)
{
    std::wstringstream ss;
    ss << value;
    WriteToConsole(ss.str());
}

template <typename T, typename... Args>
void Output(const T& first, const Args&... args)
{
    std::wstringstream ss;
    ss << first << L" ";
    WriteToConsole(ss.str());
    Output(args...);
}

template <typename... Args>
void OutputLine(const Args&... args)
{
    Output(args...);
    WriteToConsole(L"\n");
}
    
void Wait(int ms)
{
    Sleep(ms);
}
#else
#include <chrono>
#include <iostream>
#include <thread>

template <typename T>
void Output(const T& value)
{
    std::cout << value;
}

template <typename T, typename... Args>
void Output(const T& first, const Args&... args)
{
    std::cout << first;
    Output(args...);
}

template <typename... Args>
void OutputLine(const Args&... args)
{
    Output(args...);
    std::cout << std::endl;
}

void Wait(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
#endif

#include <cpuinfo.h>

#if defined(WINAPI_FAMILY) && (WINAPI_FAMILY == WINAPI_FAMILY_APP)
int main(Platform::Array<Platform::String^>^ args)
#else
int32_t main(int32_t argc, const char** argv)
#endif
{
    if (!cpuinfo_initialize())
    {
        OutputLine("[Error] Failed cpuinfo_initialize");
        return -1;
    }

    const uint32_t packages_count = cpuinfo_get_packages_count();
    for (auto index = 0u; index < packages_count; index++)
    {
        const cpuinfo_package* package = cpuinfo_get_package(0);
        if (package == nullptr)
        {
            OutputLine("[Error] Failed cpuinfo_get_package(", index, ")");
            continue;
        }

        OutputLine("[info] name: ", package->name);
    }

    OutputLine("[info] Wait to close console...");
    Wait(5000);

    return 0;
}