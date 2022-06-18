// Demo.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。
//

#include <iostream>

// It occurs compile error
//#define SPDLOG_WCHAR_FILENAMES
// enable std::wstring
#define SPDLOG_WCHAR_TO_UTF8_SUPPORT
#include <spdlog_setup/conf.h>

int main()
{
    try
    {
        // Consoled can not output as UTF-8 if comment out it
        SetConsoleOutputCP(CP_UTF8);

        // spdlog_setup::setup_error thrown if file not found
        spdlog_setup::from_file("logging.toml");

        // setup logger
        auto logger = spdlog::get("root");

        logger->info(L"Hello World!");
    }
    catch (const spdlog_setup::setup_error& e)
    {
        std::cout << e.what() << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
}