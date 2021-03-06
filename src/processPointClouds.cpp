// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
//#include <pcl/io/pcd_io.h>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT> ());
    typename pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes,filterRes,filterRes); // unit in meter
    vg.filter(*cloudFiltered);

    // crop  the region of interest
    typename pcl::PointCloud<PointT>::Ptr  cloudRegion (new  pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    //remove the roof pixels
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f (2.6,1.7,-0.4,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    //cloud region is  striked
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
        inliers -> indices.push_back(point);

    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());
    for (int index: inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }
    
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    //extract.setNegative (false);
    //extract.filter (*cloud_p);
    extract.setNegative (true);
    extract.filter (*obstCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    bool flag = false; // true use default ransac, flase use the customized ransac.
    if(flag){
        // Time segmentation process
        auto startTime = std::chrono::steady_clock::now();
        //pcl::PointIndices::Ptr inliers;
        // TODO:: Fill in this function to find inliers for the cloud.

        //pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers {new pcl::PointIndices()};
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (maxIterations);
        seg.setDistanceThreshold (distanceThreshold);

        // segement the largest planar componet from the input cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        if(inliers->indices.size() == 0){
            std::cout<< "Could not estimate a planar mode for the given dataset." <<std::endl;
        }
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
        return segResult;
    }
    
    else{
        //initial the rancac class
        RansacSegmentation<PointT> RansacSeg(maxIterations, distanceThreshold);

	    std::unordered_set<int> inliersResult = RansacSeg.RansacPlane(cloud);
        typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
        typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
        for(int i = 0; i< cloud->points.size();++i){
            PointT pt = cloud->points[i];
            if(inliersResult.count(i)){
                planeCloud->points.push_back(pt); // points on the plane are ground points
            }else{
                obstCloud->points.push_back(pt); // points outside the plane are objects
            }
        }
        return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(obstCloud,planeCloud );

    }
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    bool flag = false; // true is the pcl buil in function, flase is customized funciton
    if(flag){
        // Time clustering process
        auto startTime = std::chrono::steady_clock::now();
        pcl::PCDWriter writer;

        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

        // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

        // Creating the KdTree object for the search method of the extraction
        typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud (cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (clusterTolerance); // 0.02 =  2cm
        ec.setMinClusterSize (minSize);
        ec.setMaxClusterSize (maxSize);
        ec.setSearchMethod (tree);
        ec.setInputCloud(cloud);
        ec.extract (cluster_indices);


        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {   
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
            for (const auto& idx : it->indices)
            cloud_cluster->push_back ((*cloud)[idx]); //*
            cloud_cluster->width = cloud_cluster->size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            clusters.push_back(cloud_cluster);
            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
            std::stringstream ss;
            ss << "clusters_" << j << ".pcd";
        // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
            j++;
        }

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

        return clusters;
    }else{
        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
        ClusterPtr<PointT> clusterPoints(clusterTolerance, minSize, maxSize);
        clusters = clusterPoints.euclideanCluster(cloud);
        return clusters;
    }

}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}