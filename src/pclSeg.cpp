#include "pclSeg.h"

SegPointClouds::SegPointClouds() {}

SegPointClouds::~SegPointClouds() {}

std::pair< pcl::PointCloud<pcl::PointXYZ>::Ptr,  pcl::PointCloud<pcl::PointXYZ>::Ptr> SegPointClouds::SeparateClouds(
    pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstCloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud (new pcl::PointCloud<pcl::PointXYZ> ());
    for(int index : inliers -> indices)
        planeCloud -> points.push_back(cloud -> points[index]);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,  pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult(obstCloud, planeCloud);

    return segResult;
}

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> SegPointClouds::SegmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    //  TODO:: Fill in this function to find inliers for the cloud.
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    // pcl::SACSegmentation<PointT> seg;
    // seg.setOptimizeCoefficients (true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(maxIterations);
    // seg.setDistanceThreshold(distanceThreshold);
    // seg.setInputCloud(cloud);
    // seg.segment(*inliers, *coefficients);
    // if(inliers->indices.size()==0){
    //     std::cout<<"could not find a plane model for the given dataset."<<std::endl;
    // }

//ransac3d
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    while(maxIterations--){
            std::unordered_set<int> inliers;
            while(inliers.size()<3)
                inliers.insert(rand()%(cloud->points.size()));
            float x1, y1,z1, x2, y2,z2, x3, y3,z3;
            auto itr = inliers.begin();
            x1 = cloud->points[*itr].x;
            y1 = cloud->points[*itr].y;
            z1 = cloud->points[*itr].z;
            itr++;
            x2 = cloud->points[*itr].x;
            y2 = cloud->points[*itr].y;
            z2 = cloud->points[*itr].z;
            itr++;
            x3 = cloud->points[*itr].x;
            y3 = cloud->points[*itr].y;
            z3 = cloud->points[*itr].z;
            float i = (y2 - y1)*(z3 - z1) - (z2-z1)*(y3 - y1);
            float j = (z2 - z1)*(x3 - x1) - (x2-x1)*(z3 - z1);
            float k = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3-x1);
            float D = -(i*x1 + j*y1 + k*z1);
            for(int index=0;index< cloud->points.size();index++){
                if(inliers.count(index)>0)
                    continue;
                pcl::PointXYZ  point = cloud->points[index];
                float x4 = point.x;
                float y4 = point.y;
                float z4 = point.z;
                float d = fabs(i*x4 + j*y4 + k*z4 +D)/sqrt(i*i + j*j + k*k);
                if(d< distanceThreshold)
                    inliers.insert(index);
            }
            if(inliers.size() > inliersResult.size()){
                inliersResult = inliers;
            }
        }

        pcl::PointIndices::Ptr inliers2(new pcl::PointIndices());
        for(int index=0; index< cloud->points.size();index++){
            //  PointT  point = cloud->points[index];
            if(inliersResult.count(index)){
                inliers2-> indices.push_back(index);
            }

        }

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult = SeparateClouds(inliers2,cloud);
        return segResult;
}

std::pair< pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> SegPointClouds::SegGroundRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIter, float distanceThres){

    // 创建SACSegmentation对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 设置SAC模型类型为PLANE，即平面
    seg.setModelType(pcl::SACMODEL_PLANE);
    // 设置方法类型为RANSAC
    seg.setMethodType(pcl::SAC_RANSAC);
    // 设置最大迭代次数
    seg.setMaxIterations(maxIter);
    // 设置点到模型的距离阈值，即RANSAC算法中的阈值
    seg.setDistanceThreshold(distanceThres);
    // 输入点云数据
    seg.setInputCloud(cloud);
    // 创建点云索引对象，用于存储地面点和非地面点的索引
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // 地面分割
    seg.segment(*inliers, *coefficients);
    // 提取地面点
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false); // 提取地面点，而不是非地面点
    extract.filter(*ground);

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult = SeparateClouds(inliers,cloud);

    return segResult;
}


std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> SegPointClouds::SegGroundHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float heightThres){
    pcl::PointIndices::Ptr ground_idx (new pcl::PointIndices);
    pcl::PointIndices::Ptr non_ground_idx (new pcl::PointIndices);

    // The height threshold is conducted by analysing the pc data.
    for(auto i = 0; i < cloud->size(); ++i){
        if(cloud->points[i].z < heightThres){
            ground_idx -> indices.push_back(static_cast<int>(i));
        }
        else{
            non_ground_idx -> indices.push_back(static_cast<int>(i));
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract_ground;
    extract_ground.setInputCloud(cloud);
    extract_ground.setIndices(ground_idx);
    extract_ground.setNegative(false);
    extract_ground.filter(*ground);

    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract_non_ground;
    extract_non_ground.setInputCloud(cloud);
    extract_non_ground.setIndices(non_ground_idx);
    extract_non_ground.setNegative(false);
    extract_ground.filter(*non_ground);

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,  pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult(non_ground, ground);

    return segResult;
}
