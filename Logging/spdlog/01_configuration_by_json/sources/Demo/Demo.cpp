// Demo.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。
//

#include <iostream>

// enable std::wstring
#define SPDLOG_WCHAR_FILENAMES
#define SPDLOG_WCHAR_TO_UTF8_SUPPORT
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

int main()
{
    // Consoled can not output as UTF-8 if comment out it
    SetConsoleOutputCP(CP_UTF8);

    // setup logger
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);
    console_sink->set_pattern("[%^%l%$] %v");

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(L"logs/application.log", true);
    file_sink->set_level(spdlog::level::info);

    spdlog::logger logger("multi_sink", { console_sink, file_sink });
    logger.set_level(spdlog::level::info);

    logger.info(L"Hello World!");
}