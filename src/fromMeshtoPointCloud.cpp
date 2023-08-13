#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense> // Include the Eigen library
#include <random>      // Include the random library
#include <pcl/kdtree/kdtree_flann.h> // Include KDTree header
#include <pcl/io/pcd_io.h>

int main() {


    pcl::PolygonMesh mesh;
    pcl::io::loadPLYFile("../data/Mesh/sugar_box.ply", mesh);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);

    // Initialize a random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(0, cloud->size() - 1);

    // Sample two random points from the mesh
    int randomIndex1 = dist(gen);
    int randomIndex2 = dist(gen);
    pcl::PointXYZ samplePoint1 = cloud->points[randomIndex1];
    pcl::PointXYZ samplePoint2 = cloud->points[randomIndex2];

//    // Calculate the position of sampled points with respect to the centroid
//    Eigen::Vector3f position1 = samplePoint1.getVector3fMap() - centroid.head<3>();
//    Eigen::Vector3f position2 = samplePoint2.getVector3fMap() - centroid.head<3>();
//
//    // Print results
//    std::cout << "Position of Sample Point 1 with respect to Centroid: " << position1.transpose() << std::endl;
//    std::cout << "Position of Sample Point 2 with respect to Centroid: " << position2.transpose() << std::endl;


    // Load the point cloud from a file
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../data/YCB_PCD_Files/sugar_box.pcd", *cloud); // Replace with your point cloud file

    // Create a KDTree for the point cloud
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // Find the indices of the nearest neighbors for the sampled points
    std::vector<int> indices1(1);
    std::vector<float> distances1(1);
    kdtree.nearestKSearch(samplePoint1, 1, indices1, distances1);

    std::vector<int> indices2(1);
    std::vector<float> distances2(1);
    kdtree.nearestKSearch(samplePoint2, 1, indices2, distances2);

    // Print the indices of the corresponding points in the point cloud
    int correspondingIndex1 = indices1[0];
    int correspondingIndex2 = indices2[0];
    std::cout << "Corresponding Index of Sample Point 1 in Point Cloud: " << correspondingIndex1 << std::endl;
    std::cout << "Corresponding Index of Sample Point 2 in Point Cloud: " << correspondingIndex2 << std::endl;

    // Access the corresponding points in the point cloud using the indices
    pcl::PointXYZ correspondingPoint1 = cloud->points[correspondingIndex1];
    pcl::PointXYZ correspondingPoint2 = cloud->points[correspondingIndex2];

    // Print the corresponding points' coordinates
    std::cout << "Corresponding Point 1 in Point Cloud: " << correspondingPoint1.x << " "
              << correspondingPoint1.y << " " << correspondingPoint1.z << std::endl;
    std::cout << "Corresponding Point 2 in Point Cloud: " << correspondingPoint2.x << " "
              << correspondingPoint2.y << " " << correspondingPoint2.z << std::endl;


    return 0;
}
