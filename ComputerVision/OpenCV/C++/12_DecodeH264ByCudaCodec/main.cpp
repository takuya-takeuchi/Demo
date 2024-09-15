#include <chrono>
#include <iostream>
#include <string>

// Define HAVE_CUDA to reolve this error messaeg
// Error: The function/feature is not implemented (The called functionality is disabled for current build or platform) in throw_no_cuda
#define HAVE_CUDA
#define WITH_NVCUVID

#include <opencv2/opencv.hpp>
#include <opencv2/cudacodec.hpp>

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 2)
    {
        std::cout << "[Error] Demo </path/to/movie>" << std::endl;
        return -1;
    }

    const auto numDevices = cv::cuda::getCudaEnabledDeviceCount();
    if (numDevices == 0)
    {
        std::cout << "[Error] No available GPUs" << std::endl;
        return -1;
    }

    std::cout << "[Info] Available GPUs: " << numDevices << std::endl;

    const std::string inputVideoPath = std::string(argv[1]);
    const cv::Ptr<cv::cudacodec::VideoReader> videoReader = cv::cudacodec::createVideoReader(inputVideoPath);
    const cv::cudacodec::FormatInfo format = videoReader->format();
    
    const bool valid = format.valid;
    if (!valid)
    {
        std::cout << "[Error] Failed to open '" << inputVideoPath << "'." << std::endl;
        return -1;
    }

    const cv::cudacodec::Codec codec = format.codec;
    const int width = format.width;
    const int height = format.height;
    const double fps = format.fps;

    std::cout << "[Info]  width: " << width << std::endl;
    std::cout << "[Info] height: " << height << std::endl;
    std::cout << "[Info]    fps: " << fps << std::endl;
    switch (codec)
    {
    	case cv::cudacodec::Codec::MPEG1:
            std::cout << "[Info]  codec: " << "MPEG1" << std::endl;
            break;
        case cv::cudacodec::Codec::MPEG2:
            std::cout << "[Info]  codec: " << "MPEG2" << std::endl;
            break;
        case cv::cudacodec::Codec::MPEG4:
            std::cout << "[Info]  codec: " << "MPEG4" << std::endl;
            break;
        case cv::cudacodec::Codec::VC1:
            std::cout << "[Info]  codec: " << "VC1" << std::endl;
            break;
        case cv::cudacodec::Codec::H264:
            std::cout << "[Info]  codec: " << "H264" << std::endl;
            break;
        case cv::cudacodec::Codec::JPEG:
            std::cout << "[Info]  codec: " << "JPEG" << std::endl;
            break;
        case cv::cudacodec::Codec::H264_SVC:
            std::cout << "[Info]  codec: " << "H264_SVC" << std::endl;
            break;
        case cv::cudacodec::Codec::H264_MVC:
            std::cout << "[Info]  codec: " << "H264_MVC" << std::endl;
            break;
        case cv::cudacodec::Codec::HEVC:
            std::cout << "[Info]  codec: " << "HEVC" << std::endl;
            break;
        case cv::cudacodec::Codec::VP8:
            std::cout << "[Info]  codec: " << "VP8" << std::endl;
            break;
        case cv::cudacodec::Codec::VP9:
            std::cout << "[Info]  codec: " << "VP9" << std::endl;
            break;
        case cv::cudacodec::Codec::AV1:
            std::cout << "[Info]  codec: " << "AV1" << std::endl;
            break;
        case cv::cudacodec::Codec::NumCodecs:
            std::cout << "[Info]  codec: " << "NumCodecs" << std::endl;
            break;
        case cv::cudacodec::Codec::Uncompressed_YUV420:
            std::cout << "[Info]  codec: " << "Uncompressed_YUV420" << std::endl;
            break;
        case cv::cudacodec::Codec::Uncompressed_YV12:
            std::cout << "[Info]  codec: " << "Uncompressed_YV12" << std::endl;
            break;
        case cv::cudacodec::Codec::Uncompressed_NV12:
            std::cout << "[Info]  codec: " << "Uncompressed_NV12" << std::endl;
            break;
        case cv::cudacodec::Codec::Uncompressed_YUYV:
            std::cout << "[Info]  codec: " << "Uncompressed_YUYV" << std::endl;
            break;
        case cv::cudacodec::Codec::Uncompressed_UYVY:
            std::cout << "[Info]  codec: " << "Uncompressed_UYVY" << std::endl;
            break;
    }

    
    const std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

    uint32_t total = 0;;
    cv::cuda::GpuMat frame;
    while (true)
    {
        if (!videoReader->nextFrame(frame))
            break;
        total++;
    }

    const std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / total;
    std::cout << "[Info] frames: " << total << std::endl;
    std::cout << "[Info]     ms: " << ms << " (per frame)" << std::endl;

    return 0;
}