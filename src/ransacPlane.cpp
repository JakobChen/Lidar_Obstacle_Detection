#include "ransacPlane.h"
template<typename PointT>
RansacSegmentation<PointT>::~RansacSegmentation() {}



template<typename PointT>
std::unordered_set<int> RansacSegmentation<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud)
{
	std::unordered_set<int> inliersResult;
	
	// TODO: Fill in this function

	// For max iterations 
	while(maxIterations--){
	// Randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while(inliers.size()  < 3){
			inliers.insert(rand()%(cloud->points.size()));
		}

	// Measure distance between every point and fitted line
	// 1. calculate the A, B C Dfor Ax + By + Cz + D=0; (y1-y2)x + (x2-x1)y +(x1*y2- x2*y1) = 0
		float x1, y1,z1, x2, y2,z2, x3,y3,z3;
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

		// vector v1 travels from point1 to point2
		std::vector<float> v1 {x2-x1, y2-y1, z2-z1};
		//vector v2 travels from point1 to point3
		std::vector<float> v2 {x3-x1, y3-y1, z3-z1};
		//vector v1_v2 cross product
		std::vector<float> v1_v2(3);
		//cross product;
		v1_v2[0] = v1[1] * v2[2] - v1[2] * v2[1];
		v1_v2[1] = v1[0] * v2[2] - v1[2] * v2[0];
		v1_v2[2] = v1[0] * v2[1] - v1[1] * v2[0];
		
	 	float a =  v1_v2[0];
		float b =  v1_v2[1];
		float c =  v1_v2[2];
		float d =  -(v1_v2[0]*x1 + v1_v2[1]*y1 +  v1_v2[2]*z1 );

	// If distance is smaller than threshold count it as inlier
		for(int index =0; index < cloud->points.size(); index++){
			if(inliers.count(index) >0)
				continue;
			PointT point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			float dist = fabs(a*x4 + b*y4 + c*z4 + d) / sqrt(a*a + b*b + c*c);
			if(dist< distanceTol)
				inliers.insert(index);

		}
 

	// Return indicies of inliers from fitted line with most inliers
		if(inliers.size() > inliersResult.size() )
			inliersResult = inliers;
	}
	return inliersResult;

}
