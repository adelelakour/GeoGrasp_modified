//
// Created by adelelakour on 22.07.23.
//

#include <iostream>
#include <vector>
#include <Eigen/Dense> // Make sure you have Eigen library installed

using namespace std;
using namespace Eigen;

typedef Vector3f Point3D;
typedef vector<Point3D, aligned_allocator<Point3D>> PointCloud;

// Function to calculate the centroid of a point cloud
Point3D calculateCentroid(const PointCloud& point_cloud) {
    Point3D centroid = Point3D::Zero();
    int num_points = point_cloud.size();

    for (const Point3D& point : point_cloud) {
        centroid += point;
    }

    centroid /= num_points;
    return centroid;
}

// Function to transform the point cloud to its centroid
void transformToCentroid(PointCloud& point_cloud) {
    Point3D centroid = calculateCentroid(point_cloud);

    for (Point3D& point : point_cloud) {
        point -= centroid;
    }
}

int main() {
    // Load the point cloud data (replace this with your actual data)
    PointCloud point_cloud;

    // Add your point cloud data here
    // For example:
    // point_cloud.push_back(Point3D(1.0, 2.0, 3.0));
    // point_cloud.push_back(Point3D(4.0, 5.0, 6.0));
    // ...

    // Step 1: Calculate the centroid
    Point3D centroid = calculateCentroid(point_cloud);

    // Step 2: Transform the point cloud to its centroid
    transformToCentroid(point_cloud);

    // Print the centroid and the first few points of the transformed point cloud
    cout << "Centroid: " << centroid.transpose() << endl;
    cout << "Transformed Point Cloud:" << endl;
    for (int i = 0; i < min(5, static_cast<int>(point_cloud.size())); ++i) {
        cout << point_cloud[i].transpose() << endl;
    }

    return 0;
}
