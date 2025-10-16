     
            // The PointCloud to be processed
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudForSeg(new pcl::PointCloud<pcl::PointXYZ>);
            cloudForSeg = cloudROI;
            int maxIterations = 100;
            double distanceThreshold = 0.9;
            pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
            // TODO:: Fill in this function to find inliers for the cloud.
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
            // Create the segmentation object
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            // Optional
            seg.setOptimizeCoefficients (true);
            // Mandatory
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setMaxIterations (maxIterations);
            seg.setDistanceThreshold (distanceThreshold);

            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloudForSeg);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0){
                std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            }

            // Seperate the ground & obstacles.
            typename pcl::PointCloud<pcl::PointXYZ>::Ptr obstCloud(new pcl::PointCloud<pcl::PointXYZ>());
            typename pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZ>());

            for(int it : inliers->indices)
            {
                planeCloud->points.push_back(cloudForSeg->points[it]);
            }

            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (cloudForSeg);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*obstCloud);

            std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult(obstCloud, planeCloud);