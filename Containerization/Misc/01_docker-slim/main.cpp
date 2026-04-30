#include <chrono>
#include <iostream>
#include <thread>

#include <opencv2/opencv.hpp>

int32_t main(int32_t argc, const char **argv)
{
    const std::chrono::seconds interval(5);

    auto next_tick = std::chrono::steady_clock::now();

    std::cout << "[Info] Starting loop" << std::endl;

    try
    {
        const auto path = argv[1];
        while (true)
        {
            next_tick += interval;

            const auto image = cv::imread(path, cv::IMREAD_COLOR);
            if (image.empty())
            {
                std::cerr << "[Error] Failed to read image: " << path << std::endl;
                return -1;
            }

            std::cout << "[Info] " << path << " (w: " << image.cols << ", h: " << image.rows << ")" << std::endl;

            std::this_thread::sleep_until(next_tick);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[Error] " << e.what() << std::endl;
        return -1;
    }

    std::cout << "[Info] Finished loop" << std::endl;

    return 0;
}
