#include <cstdint>
#include <fstream>
#include <istream>
#include <iterator>
#include <vector>

#include <benchmark/benchmark.h>
#include <opencv2/opencv.hpp>

void readFile(const char* filename, std::vector<uint8_t>& vec)
{
    std::ifstream file(filename, std::ios::binary);
    file.unsetf(std::ios::skipws);

    std::streampos fileSize;
    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    vec.reserve(fileSize);
    vec.insert(vec.begin(),
               std::istream_iterator<uint8_t>(file),
               std::istream_iterator<uint8_t>());
}

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

void DecodeJxl(const std::vector<uint8_t>& buf)
{
    const auto ret = cv::imdecode(cv::Mat(buf), cv::IMREAD_COLOR);
}

void DecodeJpeg(const std::vector<uint8_t>& buf)
{
    const auto ret = cv::imdecode(cv::Mat(buf), cv::IMREAD_COLOR);
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

static void BM_DecodeJxl(benchmark::State &state) {
    std::vector<uint8_t> buf;
    readFile("lenna.jxl", buf);
    for (auto _ : state)
        DecodeJxl(buf);
}

static void BM_DecodeJpeg(benchmark::State &state) {
    std::vector<uint8_t> buf;
    readFile("lenna.jpg", buf);
    for (auto _ : state)
        DecodeJpeg(buf);
}

BENCHMARK(BM_EncodeToJxl);
BENCHMARK(BM_EncodeToJpeg);
BENCHMARK(BM_DecodeJxl);
BENCHMARK(BM_DecodeJpeg);

BENCHMARK_MAIN();