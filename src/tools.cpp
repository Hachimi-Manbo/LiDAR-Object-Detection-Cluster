#include "tools.h"

PclTools::PclTools(){
}
PclTools::~PclTools(){
}

void PclTools::pcVisualization(){
    // Read point cloud data from file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../data/tsari1.pcd", *cloud);

    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {
        // Do nothing, just wait for the viewer to close
    }
}


pcl::PointCloud<pcl::PointXYZ>::Ptr PclTools::loadPcd(std::string file)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

std::vector<boost::filesystem::path> PclTools::streamPcd(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}

// Mouse clike event sync.
// void PclTools::pointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* viewer_void) {
//     if (event.getPointIndex() == -1) {
//         return;
//     }

//     pcl::visualization::PCLVisualizer* viewer = static_cast<pcl::visualization::PCLVisualizer*>(viewer_void);
//     float x, y, z;
//     event.getPoint(x, y, z);

//     // Do something with the picked point, such as highlighting it in the other window
//     // For example, you can add a sphere at the picked point in the other window
//     viewer->addSphere(pcl::PointXYZ(x, y, z), 0.1, "picked_point");
// }
