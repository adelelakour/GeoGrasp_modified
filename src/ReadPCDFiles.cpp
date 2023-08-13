//
// Created by adelelakour on 23.07.23.
//

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "../include/geograsp/GeoGrasp.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/visualization/cloud_viewer.h>



struct GraspingPoints_Predetermined{
    std::string object_name;
    pcl::PointXYZ first_grasping_point;
    pcl::PointXYZ second_grasping_point;

};

std::vector<GraspingPoints_Predetermined> grasping_data;


void ReadPCDFiles() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr master_chef_can(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cracker_box(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sugar_box(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tomato_soup_can(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mustard_bottle(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tuna_fish_can(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pudding_box(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr gelatin_box(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr potted_meat_can(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr apple(new pcl::PointCloud<pcl::PointXYZ>);


    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/YCB_PCD_Files/002_master_chef_can.pcd", *master_chef_can) == -1) {
        PCL_ERROR("Couldn't read 002_master_chef_can.pcd\n");
        GraspingPoints_Predetermined master_chef_can;
        master_chef_can.object_name = "master_chef_can",
        master_chef_can.first_grasping_point.x =
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/YCB_PCD_Files/003_cracker_box.pcd", *cracker_box) == -1) {
        PCL_ERROR("Couldn't read 003_cracker_box.pcd\n");
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/YCB_PCD_Files/004_sugar_box.pcd", *sugar_box) == -1) {
        PCL_ERROR("Couldn't read 004_sugar_box.pcd\n");
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/YCB_PCD_Files/005_tomato_soup_can.pcd", *tomato_soup_can) == -1) {
        PCL_ERROR("Couldn't read 005_tomato_soup_can.pcd\n");
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/YCB_PCD_Files/006_mustard_bottle.pcd", *mustard_bottle) == -1) {
        PCL_ERROR("Couldn't read 003_cracker_box.pcd\n");
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/YCB_PCD_Files/002_master_chef_can.pcd", *tuna_fish_can) == -1) {
        PCL_ERROR("Couldn't read 003_cracker_box.pcd\n");
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/YCB_PCD_Files/002_master_chef_can.pcd", *pudding_box) == -1) {
        PCL_ERROR("Couldn't read 003_cracker_box.pcd\n");
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/YCB_PCD_Files/002_master_chef_can.pcd", *gelatin_box) == -1) {
        PCL_ERROR("Couldn't read 003_cracker_box.pcd\n");
    }


    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/YCB_PCD_Files/002_master_chef_can.pcd", *potted_meat_can) == -1) {
        PCL_ERROR("Couldn't read 003_cracker_box.pcd\n");
    }


    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/YCB_PCD_Files/002_master_chef_can.pcd", *apple) == -1) {
        PCL_ERROR("Couldn't read 003_cracker_box.pcd\n");
    }
}

