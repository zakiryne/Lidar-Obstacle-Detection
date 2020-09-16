/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void InsertHelper(Node **node, uint depth, std::vector<float> point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node(point, id); 	
		}
		else if(depth % 2 == 0)  //even depth means, check with 'x' value
		{
			if(point[0] > (*node)->point[0])
			{
				InsertHelper(&((*node)->right), depth + 1, point, id);
			}
			else
			{
				InsertHelper(&((*node)->left), depth + 1, point, id);
			}
		}
		else  //odd depth means , check with 'y' value
		{
			if(point[1] > (*node)->point[1])
			{
				InsertHelper(&((*node)->right), depth + 1, point, id);
			}
			else
			{
				InsertHelper(&((*node)->left), depth + 1, point, id);
			}	
		}
	}



	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		InsertHelper(&root, 0, point, id); // depth = 0 at the start
	}

	
	
	void SearchHelper(Node **node, uint depth, std::vector<float> target, float distanceTol, std::vector<int>& ids)
	{
		if(*node != NULL)
		{
			
			// check if the point is inside the Box:
			if( (*node)->point[0] >= (target[0] - distanceTol) && (*node)->point[0] <= (target[0] + distanceTol) && (*node)->point[1] >= (target[1] - distanceTol) && (*node)->point[1] <= (target[1] + distanceTol) )
			{
				float dist = sqrt( (target[0] - (*node)->point[0])*(target[0] - (*node)->point[0]) + (target[1] - (*node)->point[1])*(target[1] - (*node)->point[1]));
				if (dist <= distanceTol)
					ids.push_back((*node)->id);
			}

			if(depth % 2 == 0)  //even depth means, check with 'x' value
			{
				if( (target[0] + distanceTol) > (*node)->point[0] )
				{
					SearchHelper(&((*node)->right), depth + 1, target, distanceTol, ids);
				}
				if ( (target[0] - distanceTol) < (*node)->point[0] )
				{
					SearchHelper(&((*node)->left), depth + 1, target, distanceTol, ids);
				}
		
				
			}
			else
			{
				if( (target[1] + distanceTol) > (*node)->point[1] )
				{
					SearchHelper(&((*node)->right), depth + 1, target, distanceTol, ids);
				}
				if( (target[1] - distanceTol) < (*node)->point[1] )
				{
					SearchHelper(&((*node)->left), depth + 1, target, distanceTol, ids);
				}
		
				
			}
		}
		//return ids;
	}
	

	
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		SearchHelper(&root, 0, target, distanceTol, ids);

		return ids;
	}
	

};




