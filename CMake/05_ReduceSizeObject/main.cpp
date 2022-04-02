#include <stdio.h>
#include <opencv2/opencv.hpp>

int main(int argc, char* argv[])
{
    std::string pattern = "*.jpg";
    std::vector<cv::String> files;
    cv::glob(pattern, files);

    if(files.size() == 0) {
        std::cout << "erorr" <<std::endl;
        exit(-1);   
    }

    cv::Mat image = cv::imread(files[0]);
    std::cout << files[0] << std::endl;
    std::cout << image.size () << std::endl;
    cv::imshow("show", image);
    cv::waitKey (0);
}