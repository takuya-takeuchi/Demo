#include "pch.h"

using namespace winrt;

inline ZXing::ImageView ImageViewFromMat(const cv::Mat& image)
{
    using ZXing::ImageFormat;

    auto fmt = ImageFormat::None;
    switch (image.channels())
	{
	    case 1: fmt = ImageFormat::Lum; break;
	    case 3: fmt = ImageFormat::BGR; break;
	    case 4: fmt = ImageFormat::BGRX; break;
    }

    if (image.depth() != CV_8U || fmt == ImageFormat::None)
        return { nullptr, 0, 0, ImageFormat::None };

    return {image.data, image.cols, image.rows, fmt };
}

bool ReadFile(const std::wstring& file, std::vector<uint8_t>& buffer)
{
    // open file and get file size
    std::ifstream ifs(file.c_str(), std::ios::in | std::ios::binary);
    if (!ifs.is_open())
        return false;

    ifs.seekg(0, std::fstream::end);
    const auto eof = ifs.tellg();
    ifs.seekg(0, std::ifstream::beg);
    const auto bof = ifs.tellg();

    const auto size = eof - bof;

    // read binary data
    buffer.resize(size);
    ifs.read(reinterpret_cast<char*>(&buffer[0]), size);
    ifs.close();

    return true;
}

int wmain(int argc, wchar_t* argv[], wchar_t* /* envp */[])
{
	init_apartment();

    // setup logger
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);
    console_sink->set_pattern("[%^%l%$] %v");

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(L"logs/application.log", true);
    file_sink->set_level(spdlog::level::info);

    spdlog::logger logger("multi_sink", { console_sink, file_sink });
    logger.set_level(spdlog::level::info);

    // check args
	if (argc != 2)
	{
        logger.error(L"Invalid argument: <input image path>");
		return -1;
	}

    const auto inputImagePath = argv[1];
    logger.info(L"Input Image Path: {}", std::wstring(inputImagePath));

    std::chrono::system_clock::time_point start, end;

    // load image
    start = std::chrono::system_clock::now();
    std::vector<uint8_t> buffer;
    if (!ReadFile(inputImagePath, buffer))
    {
        logger.error(L"Failed to load {}", std::wstring(inputImagePath));
        return -1;
    }
    end = std::chrono::system_clock::now();
    auto total = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    logger.info(L"Read file: {} ms", total);

    start = std::chrono::system_clock::now();
    const auto image = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
    if (image.empty())
    {
        logger.error(L"Failed to decode {}", std::wstring(inputImagePath));
        return -2;
    }
    end = std::chrono::system_clock::now();
    total = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    logger.info(L"Decode image: {} ms", total);

    // read barcode
    start = std::chrono::system_clock::now();
    const auto imageView = ImageViewFromMat(image);
    const auto result = ZXing::ReadBarcode(imageView);
    end = std::chrono::system_clock::now();
    total = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    logger.info(L"Read barcode: {} ms", total);

    if (result.isValid())
    {
        const auto position = result.position();
        const auto zx2cv = [](ZXing::PointI p) { return cv::Point(p.x, p.y); };
        const auto contour = std::vector{ zx2cv(position[0]), zx2cv(position[1]), zx2cv(position[2]), zx2cv(position[3]) };
        const auto* pts = contour.data();
        const auto npts = static_cast<int>(contour.size());
        
        cv::polylines(image, &pts, &npts, 1, true, CV_RGB(0, 255, 0));
        cv::putText(image,
                    ZXing::TextUtfEncoding::ToUtf8(result.text()),
                    cv::Point(10, 20),
                    cv::FONT_HERSHEY_DUPLEX,
                    0.5,
                    CV_RGB(0, 255, 0));
        
        logger.info(L"Please enter key to exit on image window");
        while (true)
        {
            cv::imshow("Display window", image);
            const auto key = cv::waitKey(33);
            // enter
            if (key == 27)
                break;
        }
    }
    else
    {
        logger.info(L"Barcode is not found");
    }
}
