#include "pclProcess.h"


PclProcess::PclProcess(/* args */)
{
}

PclProcess::~PclProcess()
{
}

void PclProcess::initCamera(CameraAngle setAngle,pcl::visualization::PCLVisualizer::Ptr& viewer){
    // Set the BG color
    viewer -> setBackgroundColor (0,0,0);
    // Internal func of PCL
    viewer->initCameraParameters();
    // distance away in meters.
    int pnt_dist = 12;
    // 根据枚举量中不同的视图角度选择，设置不同的视图初始位置角度；
    switch(setAngle)
    {
        case XY : 
            viewer->setCameraPosition(-pnt_dist, -pnt_dist, pnt_dist, 1, 1, 0); 
            break;
        case TopDown : 
            viewer->setCameraPosition(0, 0, pnt_dist, 1, 0, 1); 
            break;
        case Side : 
            viewer->setCameraPosition(0, -pnt_dist, 0, 0, 0, 1); 
            break;
        case FPS : 
            viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }
    if(setAngle!=FPS){
        viewer->addCoordinateSystem (1.0);
    }
}

void PclProcess::VoxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &voxelResult,float leaf){
    pcl::VoxelGrid<PointT> grid; //创建滤波对象
    grid.setLeafSize(leaf, leaf, leaf); // 设置体素体积
    grid.setInputCloud(cloud); // 设置点云
    grid.filter(*voxelResult); // 执行滤波，输出结果
    std::cout << "voxel downsample size :" << voxelResult->size() << std::endl;
}

void PclProcess::uniformSample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &uniformResult,double radius){

    // 使用UniformSampling进行下采样
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud(cloud);
    uniform_sampling.setRadiusSearch(radius);
    uniform_sampling.filter(*uniformResult);
    // std::cout << "UniformSampling size :" << uniformResult->size() << std::endl;
}

void PclProcess::processROI(pcl::PointCloud<PointT>::Ptr cloudFiltered,pcl::PointCloud<PointT>::Ptr &cloudROI,Eigen::Vector4f minPoint,Eigen::Vector4f maxPoint){

    // 定义一个进行数据截取的对象
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudROI);

    // ROI相同的方法，移除打到本车顶部的雷达点
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6,1.7,-0.4,1));
    roof.setInputCloud(cloudROI);
    //  记录了滤波后点的序号；
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
        inliers->indices.push_back(point);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudROI);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudROI);
}

