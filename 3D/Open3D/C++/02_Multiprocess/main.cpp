#include <cmath>
#include <iostream>
#include <set>
#include <sys/wait.h>
#include <unistd.h>

#include <open3d/Open3D.h>

void worker(int32_t id, double eps, int min_points)
{
    std::cout << "[Info] Worker " << id << " started, Process ID: " << getpid() << std::endl;

    // load sample data
    open3d::data::PLYPointCloud ply_data;
    std::string ply_path = ply_data.GetPath();
    auto pcd = open3d::io::CreatePointCloudFromFile(ply_path);
    if (pcd == nullptr || pcd->IsEmpty())
    {
        std::cerr << "[Error] Failed to load point cloud." << std::endl;
        return;
    }

    // apply dbscan clustering
    std::vector<int> labels = pcd->ClusterDBSCAN(eps, min_points, true);

    // get cluster num (-1 is noize)
    std::set<int> uniqueLabels(labels.begin(), labels.end());
    const auto num_clusters = uniqueLabels.find(-1) != uniqueLabels.end() ? uniqueLabels.size() - 1 : uniqueLabels.size();
    std::cout << "[Info] Number of clusters detected: " << num_clusters << std::endl;

    std::cout << "[Info] Worker " << id << " finished" << std::endl;
}

int main()
{
    const int num_processes = 5;
    const double eps = 0.02;
    const int min_points = 10;

    for (int i = 0; i < num_processes; ++i)
    {
        pid_t pid = fork();
        if (pid == 0)
        {
            // child process
            worker(i, eps, min_points);
            // kill child process
            return 0;
        }
        else if (pid > 0)
        {
            // Parent process entries into next loop
        }
        else
        {
            std::cerr << "[Error] Failed to fork process" << std::endl;
            return 1;
        }
    }

    for (int i = 0; i < num_processes; ++i)
        wait(nullptr);

    std::cout << "[Info] All processes completed." << std::endl;
    return 0;
}