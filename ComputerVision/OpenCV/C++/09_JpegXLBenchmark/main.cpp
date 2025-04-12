#include <iostream>


// int32_t main(int32_t argc, const char** argv)
// {
//     const int32_t width = 102;
//     const int32_t height = 100;
//     const int32_t width2 = width / 2;
//     const int32_t height2 = height / 2;
//     const int32_t channel = sizeof(uint8_t);
//     const int32_t step = width * channel;
//     const int32_t stride = step + (4 - step % 4) % 4;
//     const int32_t step2 = width2 * channel;
//     const int32_t stride2 = step2 + (4 - step2 % 4) % 4;

//     auto src = (uint8_t*)malloc(height * stride);
//     memset(src, 0, height * stride);
//     for (auto h = 0; h < height; h++)
//     for (auto w = 0; w < width; w++)
//         src[h * stride + w] = w % 2 == 0 ? 0 : 255;
//     auto srcMat = cv::Mat(height, width, CV_8UC1, src, stride);

//     auto dst = (uint8_t*)malloc(height2 * stride2);
//     memset(dst, 0, height2 * stride2);
//     auto dstMat = cv::Mat(height2, width2, CV_8UC1, dst, stride2);

//     cv::resize(srcMat, dstMat, cv::Size(width2, height2));

//     std::cout << "srcMat.steps: " << srcMat.step << std::endl;
//     std::cout << "dstMat.steps: " << dstMat.step << std::endl;

//     cv::imwrite("srcMat.png", srcMat);
//     cv::imwrite("dstMat.png", dstMat); // stride of dstMat is not changed!!

//     auto srcMatWithPadding = cv::Mat(height, stride, CV_8UC1, src, stride);
//     auto dstMatWithPadding = cv::Mat(height2, stride2, CV_8UC1, dst, stride2);

//     cv::imwrite("srcMatWithPadding.png", srcMatWithPadding);
//     cv::imwrite("dstMatWithPadding.png", dstMatWithPadding);

//     // dstMat of pixels corresponds to dst buffer
//     for (auto h = 0; h < height2; h++)
//     for (auto w = 0; w < width2; w++)
//     {
//         if (dst[h * stride2 + w] != 128)
//         {
//             std::cout << "[Warn] effective pixel value. dst[" <<  h * stride2 + w << "]: " << dst[h * stride2 + w] << std::endl;
//             return -1;
//         }
//     }

//     // padding area is not modified!!
//     for (auto h = 0; h < height2; h++)
//     for (auto w = width2; w < stride2; w++)
//     {
//         if (dst[h * stride2 + w] != 0)
//         {
//             std::cout << "[Warn] padding value. dst[" <<  h * stride2 + w << "]: " << dst[h * stride2 + w] << std::endl;
//             return -1;
//         }
//     }

//     return 0;
// }

#include <benchmark/benchmark.h>
#include <opencv2/opencv.hpp>

void EncodeToJxl(const cv::Mat& mat)
{
    std::vector<unsigned char> buf;
    cv::imencode(".jxl", mat, buf);
}

void EncodeToJpeg(const cv::Mat& mat)
{
    std::vector<unsigned char> buf;
    cv::imencode(".jpg", mat, buf);
}

static void BM_EncodeToJxl(benchmark::State &state) {
    const auto mat = cv::imread("lenna.png");
    for (auto _ : state)
        EncodeToJxl(mat);
}

static void BM_EncodeToJpeg(benchmark::State &state) {
    const auto mat = cv::imread("lenna.png");
    for (auto _ : state)
        EncodeToJpeg(mat);
}

BENCHMARK(BM_EncodeToJxl);
BENCHMARK(BM_EncodeToJpeg);

BENCHMARK_MAIN();