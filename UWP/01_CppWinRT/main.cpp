#include <iostream>

#include <Windows.h>
#include <winrt/Windows.Foundation.h>

int main()
{
    winrt::init_apartment();

    winrt::Windows::Foundation::Uri uri(L"http://www.example.com");
    std::wcout << L"URI: " << uri.AbsoluteUri().c_str() << std::endl;

    winrt::uninit_apartment();

    return 0;
}