#include <iostream>
#include <string>
#include <boost/filesystem.hpp>

#include "pclProcess.h"
#include "pclSeg.h"
#include "tools.h"
#include "pclCluster.h"

/// @brief A test Function.
/// @return 
int main() {
    std::string folderPath = "/home/funnywii/Documents/LIDAR/MyPclTest/data/newStream";  // Replace with the actual folder path
    std::vector<std::string> filepaths;
    int cloudID = 0;

    // Iterate through the files in the folder and store their paths in the vector
    for (const auto& entry : boost::filesystem::directory_iterator(folderPath)) {
        if (boost::filesystem::is_regular_file(entry.path()) && entry.path().extension() == ".pcd") {
            filepaths.push_back(entry.path().string());
        }
    }
    // Sort the file paths
    std::sort(filepaths.begin(), filepaths.end());
    PclTools PclTools;
    PclProcess PclProcess;
    PclCluster PclCluster;
    SegPointClouds SegPointClouds;
    
    pcl::visualization::PCLVisualizer::Ptr viewer_Original(new pcl::visualization::PCLVisualizer("Point Cloud Viewer Original"));
    pcl::visualization::PCLVisualizer::Ptr viewer_Sampled(new pcl::visualization::PCLVisualizer("Point Cloud Viewer Sampled"));
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    PclProcess::CameraAngle setAngle = PclProcess::CameraAngle::TopDown;
    PclProcess.initCamera(setAngle,viewer_Original);
    PclProcess.initCamera(setAngle,viewer_Sampled);
    PclProcess.initCamera(setAngle,viewer);
    // Read PCD Stream.
    // Iterate through the sorted file paths, load and visualize the point clouds.
    for (const auto& filepath : filepaths) {
        // Record the time - start.
        auto startTime = std::chrono::steady_clock::now();
        // New a pc ptr.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud) == -1) {
            std::cerr << "Couldn't read file " << filepath << std::endl;
            break;
        }
        // pcd file exists. Processing.
        else{ 
            cloud_origin = cloud;
            // Uniform Sample.
            const double radius = 0.6;
            const float leaf = 1.0;
            pcl::PointCloud<pcl::PointXYZ>::Ptr sampleResult(new pcl::PointCloud<pcl::PointXYZ>);
            PclProcess.uniformSample(cloud, sampleResult, radius);

            // PC ROI
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudROI(new pcl::PointCloud<pcl::PointXYZ>);
            // 设置截取区域 X->(-10,200)
            Eigen::Vector4f minPoint,maxPoint;
            minPoint << -200.0, -200.0, -4.0, 1.0;
            maxPoint << 200.0, 200.0, 5.0, 1.0;
            // ROI Process
            PclProcess.processROI(sampleResult,cloudROI,minPoint,maxPoint);

            // Ground Segmentation ----------------------------------------------------------------
            int maxIterations = 200;
            float distanceThreshold = 0.6;
            float heightThreshold = 0.5;
            std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResults = SegPointClouds.SegGroundRANSAC(cloudROI,maxIterations,distanceThreshold);
            pcl::PointCloud<pcl::PointXYZ>::Ptr obstCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZ>);
            obstCloud = segResults.first;
            planeCloud = segResults.second;

            // Cluster 
            float clusterTolerance = 2.0;
            int minSize = 3;
            int maxSize = 50;
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = PclCluster.Clustering(obstCloud, clusterTolerance, minSize, maxSize);

            // After clustering, generate Bounding Box accoring to the cluster results.
            // PclCluster.bBoxGenerator(cloudClusters,viewer);
            // Add the point cloud to the viewer
            viewer_Original->addPointCloud(cloud_origin, std::to_string(cloudID));
            viewer_Original->spinOnce(60);  // Update the viewer to display the new point cloud
            viewer_Sampled->addPointCloud(sampleResult, std::to_string(cloudID));
            viewer_Sampled->spinOnce(60);  // Update the viewer to display the new point cloud
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(obstCloud, 255, 0, 0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(obstCloud, 0, 255, 0);
            viewer->addPointCloud(obstCloud, color1,std::to_string(cloudID));
            viewer->addPointCloud(planeCloud, color2,std::to_string(cloudID + 1));
            viewer->spinOnce(60);  // Update the viewer to display the new point cloud
            
        }
        viewer_Original->removeAllPointClouds();
        viewer_Original->removeAllShapes();
        viewer_Sampled->removeAllPointClouds();
        viewer_Sampled->removeAllShapes();
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "Segmentation Process took " << elapsedTime.count() << " milliseconds" << std::endl;
        cloudID++;
    }
       


    return 0;
}