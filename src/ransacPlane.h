#ifndef RANSACPLANE_H_
#define RANSACPLANE_H_

#include <pcl/common/common.h>
#include <unordered_set>

#include <iostream>


template<typename PointT>
class RansacSegmentation{

    int maxIterations;
    float distanceTol;
    public:
    //int maxIterations;
    //float distanceTol;
        RansacSegmentation(int maxIter, float distThr): maxIterations(maxIter), distanceTol(distThr){}
        ~RansacSegmentation();
        std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud);      
};


#endif