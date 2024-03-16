#include <iostream>

#include <opencv2/opencv.hpp>

bool checkBackend(cv::VideoCapture& videoCapture, const char* inputVideoPath, int apiPreference)
{
    videoCapture.open(inputVideoPath, apiPreference);
    const auto isOpened = videoCapture.isOpened();    
    videoCapture.release();

    return isOpened;
}

bool tryGrabFrame(cv::VideoCapture& videoCapture, const char* inputVideoPath, int apiPreference)
{
    videoCapture.open(inputVideoPath, apiPreference);   
    if (!videoCapture.isOpened())
        return false;

    cv::Mat frame;
    videoCapture >> frame;
    if (frame.empty())
        return false;

    const auto isRetrieved =  videoCapture.retrieve(frame);
    videoCapture.release();

    return isRetrieved;
}

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 2)
    {
        std::cout << "[Error] Demo </path/to/movie>" << std::endl;
        return -1;
    }

    cv::VideoCapture videoCapture;   
    const auto inputVideoPath = argv[1];
    if (checkBackend(videoCapture, inputVideoPath, cv::CAP_FFMPEG))
    {
        std::cout << "[Error] VideoCapture was successful to open '" << inputVideoPath << "'. There is something to wrong because built opencv binary should be configured without ffmpeg." << std::endl;
        return -1;
    }
    else
    {
        std::cout << "[Info] Succeeded to fail opening '" << inputVideoPath << "' by using ffmpeg backend." << std::endl;
    }

    if (!checkBackend(videoCapture, inputVideoPath, cv::CAP_ANY))
    {
        std::cout << "[Error] Failed to find '" << inputVideoPath << "' by using Any backend." << std::endl;
        return -1;
    }
    else
    {
        std::cout << "[Info] Succeeded to open'" << inputVideoPath << "' by using Any backend." << std::endl;
    }

    if (!tryGrabFrame(videoCapture, inputVideoPath, cv::CAP_ANY))
    {
        std::cout << "[Error] Failed to read frame by using Any backend." << std::endl;
        return -1;
    }
    else
    {
        std::cout << "[Info] Succeeded to read frame from '" << inputVideoPath << "' by using Any backend." << std::endl;
    }

#ifdef defined _WIN32 || defined _WIN64
    if (!checkBackend(videoCapture, inputVideoPath, cv::CAP_MSMF))
    {
        std::cout << "[Error] Failed to find '" << inputVideoPath << "' by using Microsoft Media Foundation backend." << std::endl;
        return -1;
    }
    else
    {
        std::cout << "[Info] Succeeded to open'" << inputVideoPath << "' by using Microsoft Media Foundation backend." << std::endl;
    }

    if (!tryGrabFrame(videoCapture, inputVideoPath, cv::CAP_MSMF))
    {
        std::cout << "[Error] Failed to read frame by using Microsoft Media Foundation backend." << std::endl;
        return -1;
    }
    else
    {
        std::cout << "[Info] Succeeded to read frame from '" << inputVideoPath << "' by using Microsoft Media Foundation backend." << std::endl;
    }

    if (!tryGrabFrame(videoCapture, inputVideoPath, cv::CAP_DSHOW))
    {
        std::cout << "[Warning] Failed to find '" << inputVideoPath << "' by using DirectShow backend. But System does not have proper codec to play video for DirectShow." << std::endl;
    }
    else
    {
        std::cout << "[Info] Succeeded to open'" << inputVideoPath << "' by using DirectShow backend." << std::endl;

        if (!tryGrabFrame(videoCapture, inputVideoPath, cv::CAP_DSHOW))
        {
            std::cout << "[Error] Failed to read frame by using DirectShow backend." << std::endl;
            return -1;
        }
        else
        {
            std::cout << "[Info] Succeeded to read frame from '" << inputVideoPath << "' by using DirectShow backend." << std::endl;
        }
    }
#elif __APPLE__
    if (!checkBackend(videoCapture, inputVideoPath, cv::CAP_AVFOUNDATION))
    {
        std::cout << "[Error] Failed to find '" << inputVideoPath << "' by using AVFoundation backend." << std::endl;
        return -1;
    }
    else
    {
        std::cout << "[Info] Succeeded to open'" << inputVideoPath << "' by using AVFoundation backend." << std::endl;
    }

    if (!tryGrabFrame(videoCapture, inputVideoPath, cv::CAP_AVFOUNDATION))
    {
        std::cout << "[Error] Failed to read frame by using AVFoundation backend." << std::endl;
        return -1;
    }
    else
    {
        std::cout << "[Info] Succeeded to read frame from '" << inputVideoPath << "' by using AVFoundation backend." << std::endl;
    }
#endif 

    return 0;
}