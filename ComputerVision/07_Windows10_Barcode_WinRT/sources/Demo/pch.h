#pragma once
#include <chrono>
#include <fstream>
#include <iostream>

#include <winrt/Windows.Foundation.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// enable std::wstring
#define SPDLOG_WCHAR_FILENAMES
#define SPDLOG_WCHAR_TO_UTF8_SUPPORT
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

#include <ZXing/ReadBarcode.h>
#include <ZXing/TextUtfEncoding.h>
