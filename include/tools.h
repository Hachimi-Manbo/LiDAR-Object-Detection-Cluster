#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>


class PclTools
{
private:
    /* data */
public:
    PclTools(/* args */);
    ~PclTools();
    void pcVisualization();
    // Read Stream pcd files in folder.
    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
    pcl::PointCloud<pcl::PointXYZ>::Ptr loadPcd(std::string file);
    // void pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* viewer_void);

};


