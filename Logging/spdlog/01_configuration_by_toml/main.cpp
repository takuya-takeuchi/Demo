#include <iostream>

// It occurs compile error
//#define SPDLOG_WCHAR_FILENAMES
// enable std::wstring
#if defined(_WIN32) || defined(_WIN64)
#define SPDLOG_WCHAR_TO_UTF8_SUPPORT
#endif
#include <spdlog_setup/conf.h>

int main()
{
    try
    {
#if defined(_WIN32) || defined(_WIN64)
        // Consoled can not output as UTF-8 if comment out it
        SetConsoleOutputCP(CP_UTF8);
#endif

        // spdlog_setup::setup_error thrown if file not found
        spdlog_setup::from_file("logging.toml");

        // setup logger
        auto logger = spdlog::get("root");

        logger->info("Hello World!");
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