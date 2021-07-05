/* \author Aaron Brown */
// Quiz on implementing kd tree


#ifndef KDTREE_H
#define KDTREE_H
#include <pcl/impl/point_types.hpp>
#include <vector>

// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	//std::vector<float> point;
    PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId): point(arr), id(setId), left(NULL), right(NULL){}
	//decontractor
	~Node()
	{
		delete left;
		delete right;
	}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree(): root(NULL){}

	~KdTree()
	{
		delete root;
	}

	void insertHelper( Node<PointT>** node, uint depth, PointT point, int id){
		if(*node == NULL)
			*node = new Node<PointT>(point,id);
		else{
			//calculate the current dim;
			uint cd = depth % 3; // xyz
            if(cd == 0){
                if(point.x < ((*node)->point.x ))
                    insertHelper( &((*node)->left), depth+1, point, id);
                else
                    insertHelper( &((*node)->right), depth+1, point, id);
            }else if(cd == 1){
                if(point.y < ((*node)->point.y ))
                    insertHelper( &((*node)->left), depth+1, point, id);
                else
                    insertHelper( &((*node)->right), depth+1, point, id);
            }else if(cd == 2){
                if(point.z < ((*node)->point.z ))
                    insertHelper( &((*node)->left), depth+1, point, id);
                else
                    insertHelper( &((*node)->right), depth+1, point, id);
            }

		}
	}

	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		// reecursively insert all the cloud points into  KD tree;
		insertHelper(&root,0,point,id);
	}

	void searchHelper(PointT target, Node<PointT>* node, uint depth,float distanceTol, std::vector<int>& ids){
		// ensure that the points x, y, z with buffer  [-ditanceTol,ditanceTol]
		if(node !=NULL){
            float deltaX = node->point.x - target.x;
            float deltaY = node->point.y - target.y;
            float deltaZ = node->point.z - target.z;

			if( (deltaX>=-distanceTol && deltaX<= distanceTol)  && 
				(deltaY>=-distanceTol && deltaY<= distanceTol) 	&& 
				(deltaZ>=-distanceTol && deltaZ<= distanceTol) )
            {
				float dist = sqrt(deltaX*deltaX + deltaY*deltaY +deltaZ*deltaZ);
				if (dist  <= distanceTol)
					ids.push_back(node->id);
			}

			//check boundery 
			uint cd = depth % 3;
			// x as metric
            if(cd == 0){
                if(target.x -distanceTol <  node->point.x )
                    searchHelper(target, node->left, depth+1, distanceTol, ids);
                if(target.x +distanceTol >  node->point.x )
                    searchHelper(target, node->right, depth+1, distanceTol, ids);
			
            }else if(cd == 1){ // y as metric
                if(target.y -distanceTol <  node->point.y )
                    searchHelper(target, node->left, depth+1, distanceTol, ids);
                if(target.y +distanceTol >  node->point.y )
                    searchHelper(target, node->right, depth+1, distanceTol, ids);
            }else if(cd == 2){ // z as metric
                if(target.z -distanceTol <  node->point.z )
                    searchHelper(target, node->left, depth+1, distanceTol, ids);
                if(target.z +distanceTol >  node->point.z )
                    searchHelper(target, node->right, depth+1, distanceTol, ids);
            }

		}

	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};



#endif
