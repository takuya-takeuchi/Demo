#include <climits>
#include <iostream>
#include <string.h>
#include <thread>
#include <vector>

#ifdef _WINDOWS
#include <Windows.h>
#endif

void setThreadAffinity(std::thread& th, int cpu_id)
{
    auto handle = th.native_handle();
    
#ifdef _WINDOWS
    DWORD_PTR affinity_mask = 1 << cpu_id;
    if (SetThreadAffinityMask(handle, affinity_mask) == 0)
#else
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);
    if (pthread_setaffinity_np(handle, sizeof(cpu_set_t), &cpuset) != 0)
#endif
        std::cerr << "Error: Unable to set thread affinity: " << strerror(errno) << '\n';
}

void consumeCPU()
{
    while (true)
    {
    }
}

int32_t main(int32_t argc, const char** argv)
{
    // get cpu affinity list
    std::vector<int> ids;
    for (size_t i = 1; i < argc; i++)
    {
        char* endptr;
        const long id  = strtol(argv[i], &endptr, 10);
        if (errno == ERANGE && (id == LONG_MAX || id == LONG_MIN) || *endptr != '\0')
        {
            std::cerr << "The number is out of range.\n";
            return -1;
        }

        ids.push_back(id);
    }

    const size_t id_size = ids.size();

    // create thread and set thread affinity
    std::vector<std::thread> threads;
    if (id_size == 0)
    {
        unsigned int n = std::thread::hardware_concurrency();
        std::cout << "Create " << n << " threads" << std::endl;

        threads.resize(n);
        for (size_t i = 0; i < n; i++)
            threads[i] = std::thread(consumeCPU);
    }
    else
    {
        threads.resize(id_size);
        for (size_t i = 0; i < id_size; i++)
        {
            const auto id = ids[i];
            const auto number = i + 1;
            std::cout << "Thread '" << number << "' is assigned to cpu '" << id << "'" << std::endl;

            threads[i] = std::thread(consumeCPU);
            setThreadAffinity(threads[i], id); 
        }
    }

    // wait
    for (auto& thread : threads)
        thread.join();

    return 0;
}