#include <iostream>
#include <string>
#include <vector>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;


std::vector<std::pair<std::shared_ptr<PointCloudXYZ>, std::string>> ReadPCDFiles() {
    const std::vector<std::string> fileNames = {
            "cracker_box.pcd", "master_chef_can.pcd", "sugar_box.pcd",
            "tomato_soup_can.pcd", "mustard_bottle.pcd", "tuna_fish_can.pcd",
            "pudding_box.pcd", "gelatin_box.pcd", "potted_meat_can.pcd",
            "apple.pcd"
    };

    std::vector<std::pair<std::shared_ptr<PointCloudXYZ>, std::string>> Cloud_DataBase;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


    std::string Cloud_Name;

    for (auto i: fileNames) {
        pcl::io::loadPCDFile<pcl::PointXYZ>("../data/YCB_PCD_Files/"+i, *cloud);
        auto Dot = i.find('.');
        Cloud_Name = i.substr(0, Dot);
        Cloud_DataBase.push_back(std::make_pair(cloud, Cloud_Name));
    }

    return Cloud_DataBase;
}



//****************
int main() {

    auto DataBase = ReadPCDFiles();

    std::vector<std::pair<double, std::string>> Sorted_object;

    pcl::PointCloud<pcl::PointXYZ>::Ptr Object1(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/YCB_PCD_Files/tuna_fish_can.pcd", *Object1) == -1)
    {
        PCL_ERROR("Couldn't read the source point cloud file.\n");
        return -1;
    }
/*
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/YCB_PCD_Files/tuna_fish_can.pcd", *cracker_box) == -1)
    {
        PCL_ERROR("Couldn't read the target point cloud file.\n");
        return -1;
    }
*/

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    icp.setInputSource(Object1);
    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;

    for (auto i : DataBase)
    {
        icp.setInputTarget(i.first);
        icp.align(aligned_cloud);


        if (icp.hasConverged())
        {
            double error = icp.getFitnessScore();

            Sorted_object.push_back(std::make_pair(error, i.second));

        }
        else
        {
            PCL_ERROR("ICP did not converge.\n");
            return -1;
        }
    }


    std::sort(Sorted_object.end(), Sorted_object.begin());

    std::cout <<"The most similar object is " << Sorted_object[0].second;

    return 0;

}

//                std::cout << "The most similar object is " + i.second << std::endl;
//                std::cout << "Error (Fitness Score): " << error << std::endl;