#include "tools.h"
#include "pclProcess.h"
#include "pclSeg.h"

typedef pcl::PointXYZ PointT;
int main(int argc, char const *argv[]){

 // Load point cloud data from a .pcd file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/tsari1.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }

    // 记录处理时间
    auto startTime = std::chrono::steady_clock::now();
//     // 创建一个滤波对象，这是PCL中内置的滤波类，是按照立方体模型进行滤波；
//     // 简单的理解，建立一个立方体，用这个立方体在一帧数据上按照一定的规律进行移动，一个立方体内留下一个点代表所有的点，其他点删掉；
//     pcl::VoxelGrid<pcl::PointXYZ> sor;
//     // 开辟一块新的内存区域，用于存放滤波后的点云数据；
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    
//     // Size 的选择？
//     float filterRes = 0.1;
//     // 设置输入数据；
//     sor.setInputCloud(cloud);
//     // 设置立方体的尺寸；
//     sor.setLeafSize(filterRes,filterRes,filterRes);
//     // 将滤波后的数据存放到上面开辟的内存空间中
//     sor.filter(*cloudFiltered);

    PclProcess PclProcess;
    PclProcess::CameraAngle setAngle = PclProcess::CameraAngle::XY;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer Test"));
    PclProcess.initCamera(setAngle,viewer);
    viewer->addPointCloud(cloud, "point_cloud_origin");

    // Voxel Grid Filter
    const float leaf = 0.3; 
    pcl::PointCloud<PointT>::Ptr voxelResult(new pcl::PointCloud<PointT>);
    // PclProcess.VoxelGridFilter(cloud,voxelResult,leaf);

    // Uniform Sample Filter (DownSample)
    const double radius = 0.25;
    pcl::PointCloud<PointT>::Ptr uniformResult(new pcl::PointCloud<PointT>);
    PclProcess.uniformSample(cloud,uniformResult,radius);

    pcl::visualization::PCLVisualizer::Ptr viewer_filterred(new pcl::visualization::PCLVisualizer("Point Cloud Viewer Filterred"));
    PclProcess.initCamera(setAngle,viewer_filterred);
    viewer_filterred->addPointCloud(voxelResult, "point_cloud_Filterred");

    // ROI, 删除一些离本车距离远的点,比如相隔多个车道的数据
    pcl::PointCloud<PointT>::Ptr cloudROI(new pcl::PointCloud<PointT>);
    // 设置截取区域
    Eigen::Vector4f minPoint,maxPoint;
    minPoint << -0.0, -20.0, -10.0, 1.0;
    maxPoint << 100.0, 20.0, 10.0, 1.0;
    // ROI Process
    PclProcess.processROI(cloud,cloudROI,minPoint,maxPoint);
    
    // Ground Segmentation ----------------------------------------------------------------
    cloud = cloudROI;
    int maxIterations = 100;
    double distanceThreshold = 0.2;
	pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    // TODO:: Fill in this function to find inliers for the cloud.
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (maxIterations);
	seg.setDistanceThreshold (distanceThreshold);

	// Segment the largest planar component from the remaining cloud
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);
	if (inliers->indices.size () == 0)
	{
	  std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	}

    // --------
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

	for(int it : inliers->indices)
	{
		planeCloud->points.push_back(cloud->points[it]);
	}

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);

    pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer("Point Cloud Viewer No_Ground"));
    PclProcess.initCamera(setAngle,viewer2);
    viewer2->addPointCloud(obstCloud, "point_cloud_no_ground");
    // Ground Segmentation End ----------------------------------------------------------------


    // Calculate the process time
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Process took " << elapsedTime.count() << " milliseconds" << std::endl;

    // Display the point cloud
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }

    return 0;
    
}
