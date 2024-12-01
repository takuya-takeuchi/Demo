#include <iostream>
#include <unordered_set>
#include <cmath>

#include <open3d/Open3D.h>

void run(double eps, int min_points)
{
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
    std::unordered_set<int> unique_labels(labels.begin(), labels.end());
    int num_clusters = static_cast<int>(unique_labels.size()) - (unique_labels.count(-1) ? 1 : 0);
    std::cout << "[Info] Number of clusters detected: " << num_clusters << std::endl;

    // colorize for each cluster
    std::vector<Eigen::Vector3d> colors(pcd->points_.size(), Eigen::Vector3d(0, 0, 0)); // noize is black
    for (size_t i = 0; i < labels.size(); ++i)
    {
        if (labels[i] < 0)
            continue;

        // create color like tab20 color
        double r = (labels[i] * 37 % 255) / 255.0;
        double g = (labels[i] * 73 % 255) / 255.0;
        double b = (labels[i] * 113 % 255) / 255.0;
        colors[i] = Eigen::Vector3d(b, g, r);
    }
    pcd->colors_ = colors;

    // visualize
    open3d::visualization::DrawGeometries({pcd});
}

int main()
{
    double eps = 0.02;
    int min_points = 10;

    run(eps, min_points);

    return 0;
}