#include <open3d/Open3D.h>

int main() {
    using namespace open3d;

    // PointCloudの読み込み
    auto cloud = io::CreatePointCloudFromFile("example.ply");
    if (!cloud) {
        std::cerr << "Failed to load point cloud." << std::endl;
        return -1;
    }

    // 情報表示
    std::cout << "Loaded " << cloud->points_.size() << " points." << std::endl;

    // 可視化
    visualization::DrawGeometries({cloud}, "PointCloud Viewer", 800, 600);

    return 0;
}