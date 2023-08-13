#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#include <geograsp/GeoGrasp.h>
#include <librealsense2/rs.hpp>
#include "../include/geograsp/example.hpp"

#include "../include/geograsp/utility_functions.h"


using pcl_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;

// Helper functions
//void register_glfw_callbacks(window& app, state& app_state);


pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud viewer"));

// callback signature
void cloudCallback(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

    // Remove NaN values and make it dense
    std::vector<int> nanIndices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndices);

    // Remove background points (PassThrough on X,Y,Z)
    pcl::PassThrough<pcl::PointXYZRGB> ptFilter;
    ptFilter.setInputCloud(cloud);
    ptFilter.setFilterFieldName("z");
    ptFilter.setFilterLimits(0.0, 1.5);
    ptFilter.filter(*cloud);

    ptFilter.setInputCloud(cloud);
    ptFilter.setFilterFieldName("y");
    ptFilter.setFilterLimits(-0.55, 0.40);
    ptFilter.filter(*cloud);

    ptFilter.setInputCloud(cloud);
    ptFilter.setFilterFieldName("x");
    ptFilter.setFilterLimits(-0.50, 0.50);
    ptFilter.filter(*cloud);

    /* Create the segmentation object for the planar model and set all the parameters*/
    pcl::SACSegmentation<pcl::PointXYZRGB> sacSegmentator;
        //Sample Consensus Segmentation
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZRGB>());

    sacSegmentator.setModelType(pcl::SACMODEL_PLANE);
    sacSegmentator.setMethodType(pcl::SAC_RANSAC);      //Random Sample Consensus Segmentation
    sacSegmentator.setMaxIterations(50);
    sacSegmentator.setDistanceThreshold(0.02);
    sacSegmentator.setInputCloud(cloud);
    sacSegmentator.segment(*inliers, *coefficients);
    // we send this function two arguments. 1) the pointer in which it stores the inliers of the model. and 2) The
    // pointer on which it stores the model coefficients


    /*Separate the planar model / point cloud in two different clouds */
    pcl::ExtractIndices<pcl::PointXYZRGB> indExtractor;
    indExtractor.setInputCloud(cloud);
    indExtractor.setIndices(inliers);
    indExtractor.setNegative(false);  // default: keep inliers and remove the rest
    indExtractor.filter(*cloudPlane);
    //
    indExtractor.setNegative(true);  // keep the rest and remove the inliers
    indExtractor.filter(*cloud);

//****************** Now we have table in a cloud called (cloudPlane) and the rest in a cloud called (cloud)

    /* Now, we want to seperate the objects in the scene. Each object in a separate cloud
     * Creating the KdTree object for the search method of the extraction */

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);     // cloud ohne planar model

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ecExtractor;
    ecExtractor.setClusterTolerance(0.01);
    ecExtractor.setMinClusterSize(750);
    ecExtractor.setSearchMethod(tree);
    ecExtractor.setInputCloud(cloud);
    ecExtractor.extract(clusterIndices);     // clusterIndices is a vector of the indices of each object

    // |cluster1 | cluster2| cluster3| cluster4|

    if (clusterIndices.empty()) {
        // Visualize the result
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> planeColor(cloudPlane, 0, 255, 0);

        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "Main cloud");
        viewer->addPointCloud<pcl::PointXYZRGB>(cloudPlane, planeColor, "Plane");

        viewer->spinOnce();
    }
    else {
        std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin();
        int objectNumber = 0;

        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Every cluster found is considered an object
        for (it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

            for (std::vector<int>::const_iterator pit = it->indices.begin();
                 pit != it->indices.end(); ++pit)
                objectCloud->points.push_back(cloud->points[*pit]);
                                // add points corresponding to current index to the objectCloud
                                // think of one index as an array of int, each element in this array is a sign to
                                // a set of points in the cloud

            objectCloud->width = objectCloud->points.size();
            objectCloud->height = 1;
            objectCloud->is_dense = true;


            /* So far, we have put the points of the first object in the (objectCloud). The next step is to apply
            * the algorithm on this object's cloud
             *
             * Reminer: so far we have two clouds  1) objectCloud        2) cloudPlane (that holds the table points)
             */

            // Create and initialise GeoGrasp


            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPlaneXYZ(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
            // copy clouds
            pcl::copyPointCloud(*cloudPlane, *cloudPlaneXYZ);
            pcl::copyPointCloud(*objectCloud, *objectCloudXYZ);


            GeoGrasp geoGraspPoints;
            geoGraspPoints.setBackgroundCloud(cloudPlaneXYZ);
            geoGraspPoints.setObjectCloud(objectCloudXYZ);
            geoGraspPoints.setGripTipSize(25); // 25mm grip width
            geoGraspPoints.setGrasps(1); // Keep track only of the best

            // Calculate grasping points
            geoGraspPoints.compute();         //compute function computes main axis, centroid, subcloud D , etc.

            // Extract best pair of points
            GraspContacts bestGrasp = geoGraspPoints.getBestGrasp();      //GraspContacts is a struct, getBestGrasp return a struct
            GraspPose bestPose = geoGraspPoints.getBestGraspPose();

            // Visualize the result
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(objectCloud);
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> planeRGB(cloudPlane);

            std::string objectLabel = "";
            std::ostringstream converter;

            converter << objectNumber;
            objectLabel += converter.str();
            objectLabel += "-";

            viewer->setBackgroundColor (0, 250, 0);
            // add always receives ( pointcloud, colorHandler (or directly rgb values), label_of_this_cloud)
            viewer->addPointCloud<pcl::PointXYZRGB>(objectCloud, rgb, objectLabel + "Object");
            viewer->addPointCloud<pcl::PointXYZRGB>(cloudPlane, planeRGB, objectLabel + "Plane");

            viewer->addSphere(bestGrasp.firstPoint, 0.009, 0, 0, 255, objectLabel + "First best grasp point");
            viewer->addSphere(bestGrasp.secondPoint, 0.009, 255, 0, 0, objectLabel + "Second best grasp point");
                                                                // 0.01 (shpere radium)


            /* THIS CODE IS WRITTEN BY ME */
            /*
            //
            cout << "Grap Point Oc" << endl;
            cout << bestGrasp.firstPoint << endl;
            cout << bestGrasp.secondPoint << endl;

            pcl::PointXYZRGB centroid;
            pcl::computeCentroid(*objectCloud, centroid);
            cout << "Centroiiiiid" << centroid << endl;

            for (size_t i = 0; i < objectCloud->size(); ++i) {
                cloud->points[i].x -= centroid.x;
                cloud->points[i].y -= centroid.y;
                cloud->points[i].z -= centroid.z;
            }
*/
            cout <<"BestGrasp firstPoint w.r.t Centroid" << bestGrasp.firstPoint << endl;
            cout << "BestGrasp secondPoint w.r.t Centroid" << bestGrasp.secondPoint << endl;
            //
            cout << "Best Grasp Pose" << endl;
            cout << "thumb should be at" << bestPose.firstPoint.transpose() << endl;
            cout << "middle should be at" << bestPose.secondPoint.transpose() << endl;
            cout << "Hand_Frame should be at " << bestPose.midPointPose.translation().transpose() << endl;
            cout << "Hand_Frame should be oriented " << bestPose.midPointPose.linear() << endl;








            //these coming line are just to draw axes of graspingPose
            pcl::ModelCoefficients axeX;
            axeX.values.resize (6);    // We need 6 values
            axeX.values[0] = bestPose.midPointPose.translation()[0];
            axeX.values[1] = bestPose.midPointPose.translation()[1];
            axeX.values[2] = bestPose.midPointPose.translation()[2];
            axeX.values[3] = bestPose.midPointPose.linear()(0, 0);
            axeX.values[4] = bestPose.midPointPose.linear()(1, 0);
            axeX.values[5] = bestPose.midPointPose.linear()(2, 0);


            viewer->addLine(axeX, objectLabel + "Pose axeX");


            pcl::ModelCoefficients axeY;
            axeY.values.resize (6);    // We need 6 values
            axeY.values[0] = bestPose.midPointPose.translation()[0];
            axeY.values[1] = bestPose.midPointPose.translation()[1];
            axeY.values[2] = bestPose.midPointPose.translation()[2];
            axeY.values[3] = bestPose.midPointPose.linear()(0, 1);
            axeY.values[4] = bestPose.midPointPose.linear()(1, 1);
            axeY.values[5] = bestPose.midPointPose.linear()(2, 1);

            viewer->addLine(axeY, objectLabel + "Pose axeY");

            pcl::ModelCoefficients axeZ;
            axeZ.values.resize (6);    // We need 6 values
            axeZ.values[0] = bestPose.midPointPose.translation()[0];
            axeZ.values[1] = bestPose.midPointPose.translation()[1];
            axeZ.values[2] = bestPose.midPointPose.translation()[2];
            axeZ.values[3] = bestPose.midPointPose.linear()(0, 2);
            axeZ.values[4] = bestPose.midPointPose.linear()(1, 2);
            axeZ.values[5] = bestPose.midPointPose.linear()(2, 2);

            viewer->addLine(axeZ, objectLabel + "Pose axeZ");

            objectNumber++;
        }

        while (!viewer->wasStopped())
            viewer->spinOnce(100);
    }
}


int main(int argc, char **argv) {


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("../data/creeper-isolated.pcd", *pointCloud) == -1)
    {
        std::cout << "Error: Failed to load the point cloud file." << std::endl;
        return -1;
    }
    cout << pointCloud->size() << endl;
	cloudCallback(pointCloud);

/*
    window app(1280, 720, "RealSense Pointcloud Example");
    glfw_state app_state;
    register_glfw_callbacks(app, app_state);

    rs2::pointcloud pc;
    rs2::points points;

    rs2::pipeline pipe;
    pipe.start();

    while (app)
    {
        auto frames = pipe.wait_for_frames();

        auto color = frames.get_color_frame();

        if (!color)
            color = frames.get_infrared_frame();

        pc.map_to(color);

        auto depth = frames.get_depth_frame();

        points = pc.calculate(depth);

        auto PCL_pointCloud = points_to_pcl(points);

        app_state.tex.upload(color);

        draw_pointcloud(app.width(), app.height(), app_state, points);

    }
*/
    return 0;
}


