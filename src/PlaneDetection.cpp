#include "PlaneDetection.h"
#include <omp.h>

#include "CommonFunctions.h"
#include "Timer.h"

PlaneDetection::PlaneDetection() : init_resolution_(0.0)
                                 , lowest_resolution_(0.0)
				 , angle_threshold_(0.0)
				 , dist_threshold_(0.0)
				 , res_threshold_(0.0)
                                 , points_(new pcl::PointCloud<PointT>())
				 , octree_(new pcl::octree::OctreePointCloudAdjacency<PointT>(init_resolution_))
{
};

PlaneDetection::~PlaneDetection()
{
};

void PlaneDetection::createVoxelization( pcl::PointCloud<PointT>::Ptr inputPoints, const double& initRes
                                       , const double& lowestRes, const double& distThr )
{   
    // clear octree
    octree_->deleteTree();
    
    // create octree object
    octree_->setResolution(initRes); /// \note try to do this by a relative large value since the poor quality of slam result
//     octree_->setResolution(lowestRes);
    octree_->setInputCloud(inputPoints);
    octree_->addPointsFromInputCloud();
    
    // continuous octree subdevide based on lowest size of voxel and residual threshold
    /// \todo need to create a child class for OctreePointCloud
    
}

void PlaneDetection::runPlaneExtraction_OBR(pcl::PointCloud<PointT>::Ptr inputPoints, const double& angleThreshold, const double& resThreshold)
{
    std::vector<int> indices;
    pcl::PointCloud<PointT>::Ptr filteredPoints(new pcl::PointCloud<PointT>());
    pcl::removeNaNFromPointCloud(*inputPoints, *filteredPoints, indices);
    
    points_ = filteredPoints;
    angle_threshold_ = angleThreshold * M_PI / 180.0;
    res_threshold_ = resThreshold;
    
    double totalTime = 0.0;
    CTimer timer;
    char msg[1024];
    std::vector<double> ts;
    
    ///< adaptively adjust parameters
    timer.Start();
    printf("  Getting average points distance....");
    double avgDist = getAverageDistance();
    std::cout << "avgDist = " << avgDist << std::endl;
    
    // update parameters based on the average distances between points
    init_resolution_ = 10 * avgDist;
    lowest_resolution_ = 2 * avgDist;
    dist_threshold_ = avgDist;
    
    timer.Stop();
    totalTime += timer.GetElapsedSeconds();
    timer.PrintElapsedTimeMsg(msg);
    printf("  Get average points distance time: %s.\n\n", msg);
    ts.push_back(timer.GetElapsedSeconds());
    
    ///< creat octree
    timer.Start();
    printf("  Creating initial octree....");
    createVoxelization(points_, init_resolution_, lowest_resolution_, dist_threshold_);
    
    timer.Stop();
    totalTime += timer.GetElapsedSeconds();
    timer.PrintElapsedTimeMsg(msg);
    printf("  Creat initial octree time: %s.\n\n", msg);
    ts.push_back(timer.GetElapsedSeconds());
    
    ///< plane segmentation
    timer.Start();
    printf("  Segmenting octree....");
    getPlanarFeature_OBR();
    
    timer.Stop();
    totalTime += timer.GetElapsedSeconds();
    timer.PrintElapsedTimeMsg(msg);
    printf("  Plane segemntation time: %s.\n\n", msg);
    ts.push_back(timer.GetElapsedSeconds());
    
}

void PlaneDetection::getPlanarFeature_OBR()
{
    CTimer timer;
    char msg[1024];
    
    ///< A.1b
    timer.Start();
    printf("  A.1b saliency feature estimation for voxels....");
    std::vector<VoxelInfo> voxelInfos;
    saliencyFeatureEstimation(voxelInfos);
    
    timer.Stop();
    timer.PrintElapsedTimeMsg(msg);
    printf("  cost time: %s.\n\n", msg);
    
    ///< A.2 
    timer.Start();
    printf("  A.2 voxel based region growing....");
    std::vector<SegmentInfo> segmentInfos;
    voxelBasedRegionGrowing(voxelInfos, segmentInfos);
     
    timer.Stop();
    timer.PrintElapsedTimeMsg(msg);
    printf("  cost time: %s.\n\n", msg);
    
//     std::cout << " points size is: " << points_->size() << ", searched voxel points size is: " << indexVector.size() << std::endl;
//     std::cout << " leaf nodes size is: " << leafCount << ", searched voxel leaf count size is: " << leafNodeCounter << std::endl;

//     pcl::octree::OctreePointCloudSearch<PointT>::Iterator it2 = octree_->begin();
// 
//     unsigned int traversCounter = 0;
//     for (; it2 != octree_->end(); ++it2)
//     {
// 	traversCounter++;
//     }
// 
//     std::cout << "all voxels size is: " << octree_->getLeafCount () + octree_->getBranchCount () << ", searched all voxels count size is: " << traversCounter << std::endl;
}

void PlaneDetection::voxelBasedRegionGrowing(const vector< VoxelInfo >& cellInfos, vector< SegmentInfo >& segmentInfos)
{
    int noBlockedSize = cellInfos.size();
    
    // sort based on the residual of each voxel
    std::vector<pair<int, double>> resIdxSorted(noBlockedSize);
    for (int i=0; i<noBlockedSize; ++i)
    {
	resIdxSorted[i] = std::pair<int, double>(i, cellInfos[i].residual);
    }
    std::sort( resIdxSorted.begin(), resIdxSorted.end(), [](const std::pair<int,double>& lhs, const std::pair<int,double>& rhs) { return lhs.second < rhs.second; } );
	
    int N = resIdxSorted.size();
    for (int i=0; i<N; ++i)
    {
	int voxelSeedIdx = resIdxSorted[i].first; /// begin from the point with the smallest residual
	double voxelRes = cellInfos[voxelSeedIdx].residual;
	if (voxelRes < res_threshold_)
	    break;
	
	if ( cellInfos[voxelSeedIdx].blocked ) { continue; }
	
	std::vector<int> segmentIdxTemp;
	segmentIdxTemp.reserve(10000);
	segmentIdxTemp.push_back( voxelSeedIdx );
	
	int count = 0;
	while( count < segmentIdxTemp.size() )
	{
	    voxelSeedIdx = segmentIdxTemp[count];
	    cv::Matx31d voxelSeedNormal = cellInfos[voxelSeedIdx].normal;
	    double thOrtho = pcaInfos[idxSeed].scale;

	    // search neighbouring voxels
	    
	    int num = pcaInfos[idxSeed].idxAll.size(); /// num of neighbors
	    for( int j = 0; j < num; ++j )
	    {
		int idxCur = pcaInfos[idxSeed].idxAll[j];
		if (isUsed[idxCur]) { continue; }

		// judgement1: normal deviation
		cv::Matx31d normalCur = pcaInfos[idxCur].normal;
		double normalDev = abs(normalCur.val[0] * normalStarter.val[0] + normalCur.val[1] * normalStarter.val[1] + normalCur.val[2] * normalStarter.val[2]);
		if (normalDev < thNormal) { continue;} // the angle between two vectors is more than the threshold -> drop

		// judgement2: orthogonal distance
		double dx = pointData->points[idxCur].x - xStrater;
		double dy = pointData->points[idxCur].y - yStrater;
		double dz = pointData->points[idxCur].z - zStrater;
		double dOrtho = abs(dx * normalCur.val[0] + dy * normalCur.val[1] + dz * normalCur.val[2]);
		if (dOrtho > thOrtho) { continue;} // the distance from the neighbouring point to the plane is more than the threshold -> drop

		// judgement3: parallel distance
		double dPara = dx*dx + dy*dy + dz*dz;
		if (dPara > thRadius2) { continue; } // distance from the two points is more than the threshold -> drop

		clusterTemp.push_back( idxCur );
		isUsed[idxCur] = 1;
	    }
	    count ++;
	}
	
    }
    
}

void PlaneDetection::saliencyFeatureEstimation(std::vector<VoxelInfo> &cellInfos)
{
    double MINVALUE = 1e-7;
    int leafCount = octree_->getLeafCount();
    std::vector<VoxelInfo> voxelInfos(leafCount);
    
    ///< A.1b feature estimation
    pcl::octree::OctreePointCloudSearch<PointT>::LeafNodeIterator it = octree_->leaf_begin();
    unsigned int traversCounter = 0;
    for (; it != octree_->leaf_end(); ++it)
    {
	// construct voxel points index vector
	std::vector<int> voxelPointsIndexVector;
	it.getLeafContainer().getPointIndices(voxelPointsIndexVector);
	
	// mean value
	size_t voxelPointsNum = voxelPointsIndexVector.size();

	double h_mean_x = 0.0, h_mean_y = 0.0, h_mean_z = 0.0;
	for( int j = 0; j < voxelPointsNum; ++j )
	{
	    int idx = voxelPointsIndexVector[j];
	    h_mean_x += points_->points[idx].x;
	    h_mean_y += points_->points[idx].y;
	    h_mean_z += points_->points[idx].z;
	}
	h_mean_x *= 1.0/voxelPointsNum;  h_mean_y *= 1.0/voxelPointsNum; h_mean_z *= 1.0/voxelPointsNum;
	cv::Matx31d h_mean( h_mean_x, h_mean_y, h_mean_z );

	// covariance
	double h_cov_1 = 0.0, h_cov_2 = 0.0, h_cov_3 = 0.0;
	double h_cov_5 = 0.0, h_cov_6 = 0.0;
	double h_cov_9 = 0.0;
	double dx = 0.0, dy = 0.0, dz = 0.0;
	for( int j = 0; j < voxelPointsNum; ++j )
	{
	    int idx = voxelPointsIndexVector[j];
	    dx = points_->points[idx].x - h_mean_x;
	    dy = points_->points[idx].y - h_mean_y;
	    dz = points_->points[idx].z - h_mean_z;

	    h_cov_1 += dx*dx; h_cov_2 += dx*dy; h_cov_3 += dx*dz;
	    h_cov_5 += dy*dy; h_cov_6 += dy*dz;
	    h_cov_9 += dz*dz;
	}
	cv::Matx33d h_cov(
		h_cov_1, h_cov_2, h_cov_3, 
		h_cov_2, h_cov_5, h_cov_6, 
		h_cov_3, h_cov_6, h_cov_9);
	h_cov *= 1.0/voxelPointsNum;

	// norm and curvature estimation
	cv::Matx33d h_cov_evectors;
	cv::Matx31d h_cov_evals;
	cv::eigen( h_cov, h_cov_evals, h_cov_evectors );
	cv::Matx31d norm = h_cov_evectors.row(2).t();
	double t = h_cov_evals.row(0).val[0] + h_cov_evals.row(1).val[0] + h_cov_evals.row(2).val[0] + ( rand()%10 + 1 ) * MINVALUE;
	double curvature = h_cov_evals.row(2).val[0] / t;
 
	// all points idx and residual 
	voxelInfos[traversCounter].idxAll.resize( voxelPointsNum );
	double resSquareSum = 0.0;
	for ( int j =0; j<voxelPointsNum; ++j )
	{
	    int idx = voxelPointsIndexVector[j];
	    voxelInfos[traversCounter].idxAll[j] = idx;
	    
	    cv::Matx31d pt( points_->points[idx].x, points_->points[idx].y, points_->points[idx].z );
	    cv::Matx<double, 1, 1> resMat = ( pt - h_mean ).t() * norm;
	    double resSquare = fabs( resMat.val[0] ) * fabs( resMat.val[0] );
	    resSquareSum += resSquare;
	}
	double residual = sqrt(resSquareSum / voxelPointsNum);

	// alpha and Ef
	double alpha_1d = (h_cov_evals.row(0).val[0]-h_cov_evals.row(1).val[0]) / h_cov_evals.row(0).val[0];
	double alpha_2d = (h_cov_evals.row(0).val[1]-h_cov_evals.row(2).val[0]) / h_cov_evals.row(0).val[0];
	double alpha_3d = h_cov_evals.row(2).val[0] / h_cov_evals.row(0).val[0];
	
	double sEntr = -alpha_1d * log(alpha_1d) - alpha_2d * log(alpha_2d) - alpha_3d * log(alpha_3d);

	// update voxel info
	voxelInfos[traversCounter].curvature = curvature;
	voxelInfos[traversCounter].residual = residual;
	voxelInfos[traversCounter].alpha1d = alpha_1d;
	voxelInfos[traversCounter].alpha2d = alpha_2d;
	voxelInfos[traversCounter].alpha3d = alpha_3d;
	voxelInfos[traversCounter].sEntr = sEntr;
	voxelInfos[traversCounter].normal = norm;
	voxelInfos[traversCounter].mean = h_mean;
	voxelInfos[traversCounter].cov = h_cov;
	voxelInfos[traversCounter].idxIn = voxelInfos[traversCounter].idxAll;
	voxelInfos[traversCounter].blocked = false;

	traversCounter++;
    }
    
    cellInfos = voxelInfos;
}

double PlaneDetection::getAverageDistance()
{   
//     struct ParallelSearch
//     {
//     public:
// 	pcl::PointCloud<PointT>::Ptr points_;
// 	pcl::KdTreeFLANN<PointT>::Ptr kdtree_;
// 	mutable double res_;
// 	mutable int n_points_;
// 	
// 	ParallelSearch(pcl::PointCloud<PointT>::Ptr points,
// 	               pcl::KdTreeFLANN<PointT>::Ptr kdtree, 
// 		       const double &res, const int &n_points) 
// 	             : points_(points)
// 		     , kdtree_(kdtree)
// 		     , res_(res)
// 		     , n_points_(n_points)
// 	                                                            
// 	{}
// 	
// 	~ParallelSearch() {}
// 	
// 	void operator()(const tbb::blocked_range<size_t>& r) const
// 	{
// 	    for (int i = r.begin(); i != r.end(); ++i)
// 	    {
// 		(*this)(points_->points[i]);
// 	    }
// 	}
// 	
// 	void operator()(PointT point) const
// 	{
// 	    PointT queryPoint = point;
// 	    std::vector<int> indices(2);
//             std::vector<float> sqr_distances(2);
// 	    int np = kdtree_->nearestKSearch(queryPoint, 2, indices, sqr_distances);
// 	    if (np == 2)
// 	    {
// 		res_ += sqrt(sqr_distances[1]);
// 		++n_points_;
// 	    }
// 	}
//     };
//     
    pcl::PointCloud<PointT>::Ptr cloud = points_;
    pcl::KdTreeFLANN<PointT>::Ptr tree(new pcl::KdTreeFLANN<PointT>());
    tree->setInputCloud(cloud);
//     
//     ParallelSearch searcher(cloud, tree, 0.0, 0.0);
//     tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud->size(), 1000), searcher);
//     
//     double res = searcher.res_;
//     int n_points = searcher.n_points_;
    
    
    double res = 0.0;
    int n_points = 0;
    int nres = 0;
    
    for (size_t i = 0; i < cloud->size(); ++i)
    {
	PointT queryPoint = cloud->points[i];
	std::vector<int> indices(2);
        std::vector<float> sqr_distances(2);
	nres = tree->nearestKSearch(queryPoint, 2, indices, sqr_distances);
	if (nres == 2)
	{
	    res += sqrt(sqr_distances[1]);
	    ++n_points;
	}
    }
    
    if (n_points != 0)
    {
	res /= n_points;
    }
    return res;
}
