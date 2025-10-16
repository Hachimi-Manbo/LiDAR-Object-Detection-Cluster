#include "pclCluster.h"

PclCluster::PclCluster(/* args */)
{
}

PclCluster::~PclCluster()
{
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PclCluster::Clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float clusterTolerance, int minSize, int maxSize){
    // 记录时间
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
//  生成一个Tree对象
    typename pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
// 将点云数据按照Tree的方式进行存储，方便后续遍历
    tree -> setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusterIndices;
// 生成欧氏距离聚类对象；
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
// 设置欧式距离的buffer
    ec.setClusterTolerance(clusterTolerance);
// 设置一个类别的最小数据点数
    ec.setMinClusterSize(minSize);
//  设置一个类别的最大数据点数
    ec.setMaxClusterSize(maxSize);
// 搜索方法，Tree
    ec.setSearchMethod(tree);
//  要搜索的输入
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);
// 将符合要求点存储起来；
    for(pcl::PointIndices getIndices: clusterIndices){
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZ>);
        for(int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height =1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);

    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

void PclCluster::bBoxGenerator(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters, pcl::visualization::PCLVisualizer::Ptr& viewer){
    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters){
    // for(auto cluster : cloudClusters){
        // std::cout << "The Cluster Size : " << cluster->points.size() << std::endl;
        // std::cout << "The Cluster ID :   " << clusterId << std::endl;
        
        // Calculate the 8 vertix of BBox
        pcl::PointXYZ minPoint,maxPoint;
        pcl::getMinMax3D(*cluster,minPoint,maxPoint);
        BBox bbox;
        bbox.x_min = minPoint.x;
        bbox.y_min = minPoint.y;
        bbox.z_min = minPoint.z;
        bbox.x_max = maxPoint.x;
        bbox.y_max = maxPoint.y;
        bbox.z_max = maxPoint.z;

        bbox.x_ctr = (minPoint.x + maxPoint.x) / 2;
        bbox.y_ctr = (minPoint.y + maxPoint.y) / 2;
        bbox.z_ctr = (minPoint.z + maxPoint.z) / 2;

        bbox.dist2ctr = sqrt((bbox.x_ctr * bbox.x_ctr) + (bbox.y_ctr * bbox.y_ctr));

        // Draw BBox
        renderBox(viewer,bbox,clusterId);
        clusterId ++;
    // }
    }
}

void PclCluster::renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, BBox box, int id, Color color, float opacity)
{
	if(opacity > 1.0)
		opacity = 1.0;
	if(opacity < 0.0)
		opacity = 0.0;
	
	std::string cube = "box"+std::to_string(id);
    //viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube); 
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);
    
    std::string cubeFill = "boxFill"+std::to_string(id);
    //viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill); 
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}
