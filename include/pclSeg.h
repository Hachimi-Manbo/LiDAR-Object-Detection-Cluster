#include <utility>
#include <iostream>
#include <thread>
#include <unordered_set>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>

class SegPointClouds{
    private:

    public:
    //
    SegPointClouds();
    ~SegPointClouds();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> SegmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceThreshold);
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> SegGroundRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIter, float distanceThres);
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> SegGroundHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float heightThres);
};
