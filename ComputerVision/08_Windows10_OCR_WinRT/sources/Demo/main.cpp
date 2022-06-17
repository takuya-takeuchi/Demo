#include "pch.h"

using namespace winrt;

int wmain(int argc, wchar_t* argv[], wchar_t* /* envp */[])
{
	init_apartment();

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

    // check args
	if (argc != 3)
	{
        logger.error(L"Invalid argument: <language> <input image path>");
		return -1;
	}

    const auto languageTag = argv[1];
    const auto inputImage = argv[2];
    const auto inputImagePath = std::filesystem::absolute(inputImage);

    logger.info(L"        Language: {}", std::wstring(languageTag));
    logger.info(L"Input Image Path: {}", std::wstring(inputImagePath));

    const auto language = Windows::Globalization::Language(languageTag);
    if (!Windows::Media::Ocr::OcrEngine::IsLanguageSupported(language))
    {
        logger.error(L"{} is not supported", std::wstring(languageTag));
        return -1;
    }

    std::chrono::system_clock::time_point start, end;

    start = std::chrono::system_clock::now();
    // GetFileFromPathAsync does not accept relative path
    const auto file = Windows::Storage::StorageFile::GetFileFromPathAsync(inputImagePath.wstring()).get();
    const auto stream = file.OpenAsync(Windows::Storage::FileAccessMode::Read).get();
    end = std::chrono::system_clock::now();

    auto total = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    logger.info(L"Read file: {} ms", total);

    start = std::chrono::system_clock::now();
    const auto decoder = Windows::Graphics::Imaging::BitmapDecoder::CreateAsync(stream).get();
    const auto bitmap = decoder.GetSoftwareBitmapAsync().get();
    end = std::chrono::system_clock::now();

    total = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    logger.info(L"Decode image: {} ms", total);

    const auto engine = Windows::Media::Ocr::OcrEngine::TryCreateFromLanguage(language);

    start = std::chrono::system_clock::now();
    const auto result = engine.RecognizeAsync(bitmap).get();
    end = std::chrono::system_clock::now();

    total = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    logger.info(L"Recognize: {} ms", total);

    const auto text = result.Text();
    logger.info(L"{}", std::wstring(text));
}
