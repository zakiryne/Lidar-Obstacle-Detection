// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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

    // Create the filtering object (Voxel --> reduce resolution)............
    typename pcl::PointCloud<PointT>::Ptr VoxelCloud(new pcl::PointCloud<PointT>());

    typename pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*VoxelCloud);

    //Cropping (distant points).......................................
    typename pcl::PointCloud<PointT>::Ptr CropDistCloud(new pcl::PointCloud<PointT>); 
    typename pcl::CropBox<PointT> cropFilter;
    cropFilter.setInputCloud(VoxelCloud);
    cropFilter.setMin(minPoint);
    cropFilter.setMax(maxPoint);
    cropFilter.filter(*CropDistCloud);
 
    // Cropping (roof points).......................................
    std::vector<int> indices;
    typename pcl::CropBox<PointT> cropRoofFilter(true);
    cropRoofFilter.setInputCloud(CropDistCloud);
    cropRoofFilter.setMin(Eigen::Vector4f(-1.5, -1.7, -1.0, 1));
    cropRoofFilter.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    cropRoofFilter.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for(int point: indices)
        inliers->indices.push_back(point);

    typename pcl::PointCloud<PointT>::Ptr FinalCloud(new pcl::PointCloud<PointT>); 
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (CropDistCloud); // referenece cloud
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*FinalCloud);  // inlier points are removed from the reference cloud.


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

   
    return FinalCloud;

}

template<typename PointT>
typename std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	while(maxIterations--)
	{
		std::unordered_set<int> inliers;
		
		while(inliers.size() < 3) //collect 3 points to draw a plane
		{
			int index = rand()%(cloud->points.size());
			inliers.insert(index);
		}

		//extract x,y, z points------------------------------------------------------------------->
		//unordered_set<int> :: iterator itr;
		auto itr = (inliers.begin()); 

		float x1 = cloud->points[*itr].x;
		float y1 = cloud->points[*itr].y;
		float z1 = cloud->points[*itr].z;
		itr++;
		float x2 = cloud->points[*itr].x;
		float y2 = cloud->points[*itr].y;
		float z2 = cloud->points[*itr].z;
		itr++;
		float x3 = cloud->points[*itr].x;
		float y3 = cloud->points[*itr].y;
		float z3 = cloud->points[*itr].z;

		
		float A = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		float B = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		float C = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		float D = -(A*x1 + B*y1 + C*z1);

		// get a random point other than the 3 points of the plane selected above----------------->
		for(int ind = 0; ind < cloud->points.size(); ind++)
		{
			if(inliers.count(ind) != 1)  //the point is not present in the inliers 
			{
				float x = cloud->points[ind].x;
				float y = cloud->points[ind].y;
				float z = cloud->points[ind].z;		
				
				float d = fabs(A*x + B*y + C*z + D)/sqrt(A*A + B*B + C*C);  //distance to the plane using the formula: Distance d=∣Ax+By+C∣/sqrt(A^2+B^2)

				if(d <= distanceTol)
				{
					inliers.insert(ind);	
				}
			}
		}

		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;	
		}
	}
	
	return inliersResult;

}




template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane -----------------------------------

    typename pcl::PointCloud<PointT>::Ptr PlaneCloud {new pcl::PointCloud<PointT>()}; 
    typename pcl::PointCloud<PointT>::Ptr ObstCloud {new pcl::PointCloud<PointT>()};  

     for(int index : inliers->indices)
     {
         PlaneCloud->points.push_back(cloud->points[index]);
     }  
     // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers
    extract.setInputCloud (cloud); // referenece cloud
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*ObstCloud);  // inlier points are removed from the reference cloud.


  //-------------------------------------------------------------------------------------------------------------------------------

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(ObstCloud, PlaneCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.-----------------------------------------------------

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());    

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud); // apply segmentation in the cloud points
    seg.segment (*inliers, *coefficients); //get the inliers and coefficients 
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }


    //-----------------------------------------------------------------------------------------------------------------

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles------------------------------------------------------

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back((*cloud)[*pit]); //*

         cloud_cluster->width = cloud_cluster->points.size ();
         cloud_cluster->height = 1;
         cloud_cluster->is_dense = true;

         clusters.push_back(cloud_cluster);
    }


    //-----------------------------------------------------------------------------------------------------------------------------------------------

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
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

template<typename PointT>
void ProcessPointClouds<PointT>::ClusterHelper(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree *tree, float distanceTol)
{
	processed[indice] = true;
	cluster.push_back(indice);

	
	std::vector<int> nearest = tree->search(points[indice], distanceTol);

	for(int id : nearest)
	{
		if(!processed[id])
			ClusterHelper(id, points, cluster, processed, tree, distanceTol);
	}


}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed(points.size(), false);

	int i = 0;

	while(i < points.size())
	{
		if(processed[i])
		{
			i++;
			continue;  // skip this point and proceed to the next
		}

		// if the point is not processed, create a brand new cluster
		std::vector<int> cluster;
		ClusterHelper(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
		//std::cout << "ID: " << i <<endl;
		i++;


	}
 
	return clusters;

}