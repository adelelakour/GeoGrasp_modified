#include <iostream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>
#include <algorithm>
#include <pcl/search/kdtree.h>
#include <chrono>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudXYZ;


template<typename TreeT, typename PointT>
float nearestDistance(const TreeT& tree, const PointT& pt)
{
    const int k = 1;
    std::vector<int> indices (k);
    std::vector<float> sqr_distances (k);

    tree.nearestKSearch(pt, k, indices, sqr_distances);

    return sqr_distances[0];
}


template<typename CloudT>
float _similarity(const CloudT& cloudA, const CloudT& cloudB, float threshold)
{
    // compare B to A
    int num_outlier = 0;
    pcl::search::KdTree<PointT> tree (new pcl::search::KdTree<PointT>);
    tree.setInputCloud(cloudA); // Use the point cloud object directly

    auto sum = std::accumulate(cloudB->begin(), cloudB->end(), 0.0f, [&](auto current_sum, const auto& pt) {
        const auto dist = nearestDistance(tree, pt);

        if(dist < threshold)
        {
            return current_sum + dist;
        }
        else
        {
            num_outlier++;
            return current_sum;
        }
    });

    return sum / (cloudB->size() - num_outlier);
}



template<typename CloudT>
float similarity(const CloudT& cloudA, const CloudT& cloudB, float threshold = std::numeric_limits<float>::max())
{
    // compare B to A
    const auto similarityB2A = _similarity(cloudA, cloudB, threshold);
    // compare A to B
    const auto similarityA2B = _similarity(cloudB, cloudA, threshold);

    return (similarityA2B * 0.5f) + (similarityB2A * 0.5f);
}


// To read all PCD files and add them into a vector
std::vector<std::pair<PointCloudXYZ::Ptr, std::string>> ReadPCDFiles() {
    const std::vector<std::string> fileNames = {
            "cracker_box.pcd", "master_chef_can.pcd", "sugar_box.pcd",
            "tomato_soup_can.pcd", "mustard_bottle.pcd", "tuna_fish_can.pcd",
            "pudding_box.pcd", "gelatin_box.pcd", "potted_meat_can.pcd",
            "apple.pcd"
    };

    std::vector<std::pair<PointCloudXYZ::Ptr, std::string>> Cloud_DataBase;

    for (const auto& fileName : fileNames) {
        PointCloudXYZ::Ptr cloud(new PointCloudXYZ);  // Create a new point cloud for each file

        pcl::io::loadPCDFile<pcl::PointXYZ>("../data/YCB_PCD_Files/" + fileName, *cloud);

        std::string cloudName = fileName.substr(0, fileName.find('.'));
        Cloud_DataBase.push_back(std::make_pair(cloud, cloudName));
    }

    return Cloud_DataBase;
}






//****************
int main() {

    auto start = std::chrono::high_resolution_clock::now();

    auto DataBase = ReadPCDFiles();
    PointCloudXYZ::Ptr Object1 (new PointCloudXYZ);

    std::vector<std::pair<double, std::string>> Sorted_object;


    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/YCB_PCD_Files/tuna_fish_can.pcd", *Object1) == -1)
    {
        PCL_ERROR("Couldn't read the source point cloud file.\n");
        return -1;
    }

    cout << "Similarity_value: " << similarity(Object1, DataBase[0].first, 10.0) << endl;

    double similarity_value;

    for (auto const i : DataBase)
    {
        similarity_value = similarity(Object1, i.first, 10.0);
        Sorted_object.push_back(std::make_pair(similarity_value, i.second));
    }
    std::sort(Sorted_object.begin(), Sorted_object.end());

    std::cout << "The most fimilar object is : " << Sorted_object[0].second << endl;

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Execution time: " << duration << " ms" << std::endl;


    return 0;

}

