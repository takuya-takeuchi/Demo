#include <iostream>
#include <opencv2/opencv.hpp>

#include "export.hpp"

DLLEXPORT void showBuildInformation()
{
    std::cout << cv::getBuildInformation() << std::endl;
}