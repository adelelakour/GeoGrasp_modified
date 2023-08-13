#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/YCB_PCD_Files/cracker_box.pcd", *cloud1) == -1) {
        std::cerr << "Couldn't read point cloud file: cracker_box.pcd" << std::endl;
        return 1;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/YCB_PCD_Files/tomato_soup_can.pcd", *cloud2) == -1) {
        std::cerr << "Couldn't read point cloud file: tomato_soup_can.pcd" << std::endl;
        return 1;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud2);

    double maxHausdorffDistance = 0.0;

    for (const auto& point : cloud1->points) {
        std::vector<int> indices(1);
        std::vector<float> sqrDistances(1);
        kdtree.nearestKSearch(point, 1, indices, sqrDistances);

        double distance = std::sqrt(sqrDistances[0]);
        if (distance > maxHausdorffDistance) {
            maxHausdorffDistance = distance;
        }
    }

    std::cout << "Hausdorff Distance: " << maxHausdorffDistance << std::endl;

    return 0;
}
