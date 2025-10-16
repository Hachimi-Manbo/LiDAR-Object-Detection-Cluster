#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

class PclProcess
{
private:
    /* data */
   

public:
    PclProcess(/* args */);
    ~PclProcess();
    typedef pcl::PointXYZ PointT;
    // X: Forward the vehicle's front
    // Y: Left side of vehicle
    // Z: Vertically upward.
    enum CameraAngle{
        XY, 
        TopDown, 
        Side, 
        FPS
    };

    void initCamera(CameraAngle setAngle,pcl::visualization::PCLVisualizer::Ptr& viewer);
    void VoxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud,float leaf);
    void uniformSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &uniformResult,double radius);
    void processROI(pcl::PointCloud<PointT>::Ptr cloudFiltered,pcl::PointCloud<PointT>::Ptr &cloudROI,Eigen::Vector4f minPoint,Eigen::Vector4f maxPoint);
};

