#ifndef CLUSTER_H
#define CLUSTER_H
#include <pcl/common/common.h>
#include "kdTree.h"


template<typename PointT>
class ClusterPtr{
    private:
        float distanceTol;
        int minSize;
        int maxSize;
    public:
        ClusterPtr(float clt, int minS, int maxS): distanceTol(clt),minSize(minS), maxSize(maxS) {}
        ~ClusterPtr();
        std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud);
        void clusterHelper(int indice,typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster_ind, std::vector<bool>& processed_points, KdTree<PointT>* tree);
};



#endif