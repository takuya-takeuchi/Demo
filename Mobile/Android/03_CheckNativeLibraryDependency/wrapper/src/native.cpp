#include "../include/native.h"

#include <iostream>

#include <opencv2/opencv.hpp>

void showBuildInformation()
{
    std::cout << cv::getBuildInformation() << std::endl;
}