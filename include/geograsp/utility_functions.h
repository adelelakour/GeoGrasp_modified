#ifndef UTILITY_FUNCTIONS
#define UTILITY_FUNCTIONS

#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include "example.hpp"
// Struct for managing rotation of pointcloud view
struct state {
    double yaw;
    double pitch;
    double last_x;
    double last_y;
    bool ml;
    float offset_x;
    float offset_y;

    state();

};

using pcl_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;

// Helper functions

pcl_ptr points_to_pcl(const rs2::points& points);

#endif  // EXAMPLE_H
