#include <codecvt>
#include <locale>
#include <numeric>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/freetype.hpp>

int32_t main(int32_t argc, const char** argv)
{
    cv::Mat mat(480, 640, CV_8UC3);

    auto pixel = (uint8_t*)mat.data;
    const auto totalBytes = mat.total() * mat.elemSize();

    // clear by white color
    std::fill(pixel, pixel + totalBytes, 255);

    cv::Ptr<cv::freetype::FreeType2> freetype2 = cv::freetype::createFreeType2();
    freetype2->loadFontData("fonts/NotoSansJP-VariableFont_wght.ttf", 0);
    
    std::wstring_convert<std::codecvt_utf8<wchar_t>> conv;
    const auto str = conv.to_bytes(L"こんにちは。これは日本語です。");
    freetype2->putText(mat, str, cv::Point(20, 20), 32, cv::Scalar(0, 0, 0), 1, cv::LINE_AA, false);
    // Can not pass non-ascii string directly!!
    // freetype2->putText(mat, "こんにちは。これは日本語です。", cv::Point(20, 20), 32, cv::Scalar(0, 0, 0), 3, cv::LINE_AA, false);    

    cv::imwrite("japanese.png", mat);

    return 0;
}