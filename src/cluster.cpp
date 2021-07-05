
#include <chrono>
#include <string>
#include "cluster.h"
#include <pcl/impl/point_types.hpp>

template<typename PointT>
ClusterPtr<PointT>::~ClusterPtr(){}

template<typename PointT>


void ClusterPtr<PointT>:: clusterHelper(int  indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster_ind, std::vector<bool>& processed_points, KdTree<PointT>* tree){
	processed_points[indice] = true;
	cluster_ind.push_back(indice);
	std::vector<int> nearest = tree->search(cloud->points[indice],distanceTol);
	for(int id: nearest){
		if(!processed_points[id]){
			clusterHelper(id, cloud, cluster_ind, processed_points,tree);
		}
	}
}

//define the eucldean Cluster
// S1: inser cloud points into KD Tree
// S2:  iterate all the points,
// S3:  if the distacne to the target poitn less than distanceTol then treate the current point as neightbor.
// S4 if all the neighbors points  size in (minSize, maxSize) will be pushed  into clusters;
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ClusterPtr<PointT>:: euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud)
{
	// TODO: Fill out this function to return list of indices for each cluster
    KdTree<PointT> *tree = new KdTree<PointT>;
    for(int i =0; i<cloud->points.size();i++ ){
        tree->insert(cloud->points[i],i);
    }
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::vector<bool> processed_points(cloud->points.size(),false);
	int i = 0;		

	while(i < cloud->points.size()){
		if(processed_points[i]){
			i++;
			continue;
		}
        std::vector<int> cluster_ind;
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>());

		clusterHelper(i, cloud, cluster_ind, processed_points, tree);
        if(cluster_ind.size()>minSize &&  cluster_ind.size()<maxSize ){
            for(int j=0; j<cluster_ind.size();j++){
                cloud_cluster->points.push_back(cloud->points[cluster_ind[j]]);
            }
            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            clusters.push_back(cloud_cluster);
        }

		i++;
	}
	return clusters;

}