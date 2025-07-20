#include <chrono>
#include <iostream>
#include <thread>

#include <tbb/parallel_pipeline.h>
#include <tbb/global_control.h>

using namespace std;
using namespace tbb;
using namespace std::chrono;

void heavy_task(int data, const char* stage_name, int ms = 50)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void run_pipeline(bool parallel_mode, int num_items, int token_num) {
    auto start = steady_clock::now();
    int count = 0;

    if (parallel_mode)
    {
        std::cout << "[Parallel] token_num = " << token_num << std::endl;

        parallel_pipeline(
            token_num,
            make_filter<void, int>(
                filter_mode::serial_in_order,
                [&](flow_control& fc) -> int {
                    if (count >= num_items) {
                        fc.stop();
                        return 0;
                    }
                    return count++;
                }
            ) &
            make_filter<int, int>(
                filter_mode::parallel,
                [](int x) -> int {
                    heavy_task(x, "Stage A");
                    return x;
                }
            ) &
            make_filter<int, int>(
                filter_mode::parallel,
                [](int x) -> int {
                    heavy_task(x, "Stage B");
                    return x;
                }
            ) &
            make_filter<int, void>(
                filter_mode::serial_in_order,
                [](int x) {
                    heavy_task(x, "Stage C");
                }
            )
        );
    }
    else
    {
        std::cout << "[Sequential] (for loop)" << std::endl;

        for (int i = 0; i < num_items; ++i)
        {
            heavy_task(i, "Stage A");
            heavy_task(i, "Stage B");
            heavy_task(i, "Stage C");
        }
    }

    auto end = steady_clock::now();
    double elapsed = duration_cast<duration<double>>(end - start).count();
    std::cout << "Processed " << num_items << " items in " << elapsed << " seconds." << std::endl;
    std::cout << "Throughput: " << (num_items / elapsed) << " items/sec" << std::endl;
}

int main()
{
    int num_items = 100;

    std::vector<int> token_list = {1, 2, 4, 8};

    for (int token : token_list)
        run_pipeline(true, num_items, token);

    run_pipeline(false, num_items, 4);

    return 0;
}
