#include <iostream>

#include <opencv2/opencv.hpp>

int32_t main(int32_t argc, const char** argv)
{
    if (argc != 2)
    {
        std::cout << "[Error] Demo </path/to/movie>" << std::endl;
        return -1;
    }

    const auto inputVideoPath = argv[1];
    cv::VideoCapture videoCapture(inputVideoPath, cv::CAP_FFMPEG);   
    if (!videoCapture.isOpened())
    {
        std::cerr << "[Error] VideoCapture was not successful to open '" << inputVideoPath << "'. There is something to wrong because built opencv binary should be configured without ffmpeg." << std::endl;
        return -2;
    }

    size_t count = 0;
    while (true)
    {
        cv::Mat frame;
        if (!videoCapture.read(frame))
        {
            std::cout << "[Info] Video has finished." << std::endl;
            break;
        }

        count++;
    }
    
    std::cout << "[Info] Total frame count: " << count << std::endl;

    return 0;
}