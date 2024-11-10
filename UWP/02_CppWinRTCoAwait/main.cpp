#include <chrono>
#include <coroutine>
#include <iostream>

#include <Windows.h>
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.ApplicationModel.Core.h>

using namespace std::chrono_literals;

winrt::Windows::Foundation::IAsyncAction delay()
{
    co_await winrt::resume_after(std::chrono::seconds(5));
    // 待機後の処理
}

int main()
{
    winrt::init_apartment();

    std::wcout << L"Hello, world" << std::endl;

    auto action = delay();
    action.get();

    // // To show console window and wait delay
    // winrt::Windows::ApplicationModel::Core::CoreApplication::Run(winrt::make<App>());

    // Need not to invoke uninit_apartment
    winrt::uninit_apartment();

    return 0;
}