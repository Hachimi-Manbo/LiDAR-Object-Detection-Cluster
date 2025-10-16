#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
// #include "render/box.h"
#include <unordered_set>
#include <Eigen/Geometry> 
#include <pcl/visualization/cloud_viewer.h>

struct BBoxQ
{
	Eigen::Vector3f bboxTransform;
	Eigen::Quaternionf bboxQuaternion;
	float cube_length;
    float cube_width;
    float cube_height;
};
struct BBox
{
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;

	float x_ctr;
	float y_ctr;
	float z_ctr;
	float dist2ctr;

};
struct Color
{
	float r, g, b;
	Color(float setR, float setG, float setB)
		: r(setR), g(setG), b(setB)
	{}
};


class PclCluster
{
private:

    /* data */
public:
    PclCluster(/* args */);
    ~PclCluster();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float clusterTolerance = 1.0, int minSize = 3, int maxSize = 30);
    void bBoxGenerator(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters, pcl::visualization::PCLVisualizer::Ptr& viewer);
	void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, BBox box, int id, Color color = Color(1,0,0), float opacity=1);

};

