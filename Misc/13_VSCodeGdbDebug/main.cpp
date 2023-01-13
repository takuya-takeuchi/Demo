#include <iostream>

#include <opencv2/core/utility.hpp>

int32_t main(int32_t argc, const char* argv[])
{
    const auto info = cv::getBuildInformation();
    std::cout << info << std::endl;
    return 0;
}