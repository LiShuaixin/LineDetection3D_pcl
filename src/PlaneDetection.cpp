#include "PlaneDetection.h"
#include <omp.h>

#include "CommonFunctions.h"
#include "Timer.h"

PlaneDetection::PlaneDetection() : init_resolution_(0.0)
                                 , lowest_resolution_(0.0)
				 , angle_threshold_(0.0)
				 , dist_threshold_(0.0)
				 , res_threshold_(0.0)
				 , min_valid_points_(0)
                                 , points_(new pcl::PointCloud<PointT>())
				 , octree_(new pcl::octree::OctreePointCloudVoxel<PointT>(init_resolution_))
{
};

PlaneDetection::~PlaneDetection()
{
};

void PlaneDetection::runPlaneExtraction_OBR(pcl::PointCloud<PointT>::Ptr inputPoints, const double& angleThreshold, 
					    const double& resThreshold, const int& minValidPoints)
{
    std::vector<int> indices;
    pcl::PointCloud<PointT>::Ptr filteredPoints(new pcl::PointCloud<PointT>());
    pcl::removeNaNFromPointCloud(*inputPoints, *filteredPoints, indices);
    
    points_ = filteredPoints;
    angle_threshold_ = angleThreshold * M_PI / 180.0;
    res_threshold_ = resThreshold;
    min_valid_points_ = minValidPoints;
    
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
    init_resolution_ = 20 * avgDist;
    lowest_resolution_ = 5 * avgDist;
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

void PlaneDetection::getPlanarFeature_OBR()
{
    CTimer timer;
    char msg[1024];
    
    ///< A.1b
    timer.Start();
    printf("  A.1b saliency feature estimation for voxels....");
    std::map<pcl::octree::LeafNodeKey, VoxelInfo> voxelInfos;
    saliencyFeatureEstimation(voxelInfos);
    
    timer.Stop();
    timer.PrintElapsedTimeMsg(msg);
    printf("  cost time: %s.\n\n", msg);
    
    ///< A.2 
    timer.Start();
    printf("  A.2 voxel based region growing....");
    std::vector<std::vector<pcl::octree::LeafNodeKey>> segmentClusters;
    voxelBasedRegionGrowing(voxelInfos, segmentClusters);
     
    timer.Stop();
    timer.PrintElapsedTimeMsg(msg);
    printf("  cost time: %s.\n\n", msg);
    
    ///< B.1
    timer.Start();
    printf("  B.1 boundry voxel extraction and search for points in the vicinity....");
    
    
    timer.Stop();
    timer.PrintElapsedTimeMsg(msg);
    printf("  cost time: %s.\n\n", msg);
    
//     std::vector<SegmentInfo> segmentInfos;
}

void PlaneDetection::saliencyFeatureEstimation(std::map<pcl::octree::LeafNodeKey, VoxelInfo> &cellInfos)
{
    double MINVALUE = 1e-7;
    int leafCount = octree_->getLeafCount();
//     std::cout << "leafCount = " << leafCount << std::endl;
    
    ///< A.1b feature estimation
    pcl::octree::OctreePointCloudVoxel<PointT>::LeafNodeIterator it = octree_->leaf_begin();
    unsigned int traversCounter = 0;
    for (; it != octree_->leaf_end(); ++it)
    {
	VoxelInfo voxelInfo;
	
	// construct voxel points index vector
	std::vector<int> voxelPointsIndexVector;
	it.getLeafContainer().getPointIndices(voxelPointsIndexVector);

	// mean value
	int voxelPointsNum = voxelPointsIndexVector.size();
	if (voxelPointsNum == 0)
	    continue;
	// std::cout << "size of current voxel points = " << voxelPointsNum << std::endl;

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
	voxelInfo.idxAll.resize( voxelPointsNum );
	double resSquareSum = 0.0;
	for ( int j =0; j<voxelPointsNum; ++j )
	{
	    int idx = voxelPointsIndexVector[j];
	    voxelInfo.idxAll[j] = idx;
	    
	    cv::Matx31d pt( points_->points[idx].x, points_->points[idx].y, points_->points[idx].z );
	    cv::Matx<double, 1, 1> resMat = ( pt - h_mean ).t() * norm;
	    double resSquare = fabs( resMat.val[0] ) * fabs( resMat.val[0] );
	    resSquareSum += resSquare;
	}
	double residual = sqrt(resSquareSum / (voxelPointsNum + MINVALUE));

	// alpha and Ef
	double alpha_1d = (sqrt(h_cov_evals.row(0).val[0])-sqrt(h_cov_evals.row(1).val[0])) / (sqrt(h_cov_evals.row(0).val[0]) + MINVALUE);
	double alpha_2d = (sqrt(h_cov_evals.row(1).val[0])-sqrt(h_cov_evals.row(2).val[0])) / (sqrt(h_cov_evals.row(0).val[0]) + MINVALUE);
	double alpha_3d = sqrt(h_cov_evals.row(2).val[0]) / (sqrt(h_cov_evals.row(0).val[0]) + MINVALUE);
	
	double sEntr = 0.0;
	if (alpha_1d != 0 && alpha_2d != 0 && alpha_3d != 0)
	    sEntr = -alpha_1d * log(alpha_1d) - alpha_2d * log(alpha_2d) - alpha_3d * log(alpha_3d);
	
	// leaf key
	pcl::octree::LeafNodeKey keyArg;
	if (voxelPointsNum > 1)
	{
	    PointT voxelCentroid;
	    voxelCentroid.x = h_mean.row(0).val[0];
	    voxelCentroid.y = h_mean.row(1).val[0];
	    voxelCentroid.z = h_mean.row(2).val[0];
	    
	    octree_->getOctreeKeyforPoint(voxelCentroid, keyArg);
	    // std::cout << "keyArg = " << keyArg.index_ << "\n" << std::endl;
	}

	// update voxel info	
	voxelInfo.curvature = curvature;
	voxelInfo.residual = residual;
// 	voxelInfo.alpha1d = alpha_1d;
// 	voxelInfo.alpha2d = alpha_2d;
// 	voxelInfo.alpha3d = alpha_3d;
// 	voxelInfo.sEntr = sEntr;
	voxelInfo.normal = norm;
	voxelInfo.mean = h_mean;
	voxelInfo.cov = h_cov;
	voxelInfo.idxIn = voxelInfo.idxAll;
	voxelInfo.keyArg = keyArg;
	voxelInfo.blocked = false;
	voxelInfo.allocated = false;

// 	std::cout << "voxel info: \n" << "\t curvature: " << voxelInfo.curvature << "\n\t residual: " << voxelInfo.residual
// 	          << "\n\t alpha_1d: " << voxelInfo.alpha1d << "\n\t alpha_2d: " << voxelInfo.alpha2d
// 	          << "\n\t alpha_3d: " << voxelInfo.alpha3d << "\n\t sEntr: " << voxelInfo.sEntr
// 	          << "\n\t blocked: " << voxelInfo.blocked << "\n\t allocated: " << voxelInfo.allocated 
// 	          << "\n\t eigen value: " << h_cov_evals.row(0).val[0] << ", " << h_cov_evals.row(1).val[0] 
// 	          << ", " << h_cov_evals.row(2).val[0] 
// 	          << "\n\t test eigen value: " << eigen_value[0] << ", " << eigen_value[1] 
// 	          << ", " << eigen_value[2] 
// 	          << "\n\t norm: " << norm.row(0).val[0] << ", " << norm.row(1).val[0] 
// 	          << ", " << norm.row(2).val[0] 
// 	          << "\n\t test norm: " << unit_norm[0] << ", " << unit_norm[1] 
// 	          << ", " << unit_norm[2]
// 	          << std::endl;
	
	cellInfos.insert(std::pair<pcl::octree::LeafNodeKey, VoxelInfo>(keyArg, voxelInfo));
	
	traversCounter++;
    }
    std::cout << "traversCounter is: " << traversCounter << ", while cellInfos size is: " << cellInfos.size() << std::endl;
}

void PlaneDetection::voxelBasedRegionGrowing(std::map<pcl::octree::LeafNodeKey, VoxelInfo> &cellInfos, std::vector<std::vector<pcl::octree::LeafNodeKey>>& segmentClusters)
{
    int sizeVoxel = cellInfos.size();
    std::cout << "\n----- size of all voxel is: " << sizeVoxel << std::endl;
    // sort based on the residual of each voxel
    std::vector<pair<pcl::octree::LeafNodeKey, double>> resIdxSorted(sizeVoxel);
    int countTmp = 0;
    for (std::map<pcl::octree::LeafNodeKey, VoxelInfo>::const_iterator it = cellInfos.begin(); it != cellInfos.end(); ++it, ++countTmp)
    {
	resIdxSorted[countTmp] = std::pair<pcl::octree::LeafNodeKey, double>(it->first, it->second.residual);
    }
    std::sort( resIdxSorted.begin(), resIdxSorted.end(), [](const std::pair<pcl::octree::LeafNodeKey,double>& lhs, const std::pair<pcl::octree::LeafNodeKey,double>& rhs) { return lhs.second < rhs.second; } );
    
    std::vector<std::vector<pcl::octree::LeafNodeKey>> R; // each plane seed with smoothing surface -> will be expended further
    int count = 0, sizeBlocked = 0;
    while (sizeBlocked < sizeVoxel)
    {
	std::vector<pcl::octree::LeafNodeKey> Rc;
	std::list<pcl::octree::LeafNodeKey> Sc;	
	int numSegmentPoints = 0;
	
	pcl::octree::LeafNodeKey voxelSeedKey = resIdxSorted[count].first; /// begin from the point with the smallest residual
	count ++;
	VoxelInfo voxelSeed = cellInfos.find(voxelSeedKey)->second;
	
	if ( voxelSeed.blocked ) 
	{ 
	    continue; 
	}
	else
	{
	    cellInfos.find(voxelSeedKey)->second.blocked = true;
	    sizeBlocked++;
	}	
	
	double voxelRes = voxelSeed.residual;
	if (voxelRes > res_threshold_)
	    break;
	
	Rc.push_back(voxelSeedKey);
	Sc.push_back(voxelSeedKey);
	numSegmentPoints += voxelSeed.idxAll.size();
	
	while(!Sc.empty())
	{
	    pcl::octree::LeafNodeKey keyVi = Sc.front(); // idx of current voxel seed
	    Sc.pop_front();
	    VoxelInfo voxelVi;
	    if(cellInfos.find(keyVi) != cellInfos.end())
		voxelVi = cellInfos.find(keyVi)->second;
	    else
		continue;
	    
	    // finding adjoining voxels of the current seed Vi
	    std::vector<pcl::octree::LeafNodeKey> neighborLeafKeys;
	    neighborLeafKeys = octree_->getNeighborVoxelKeysAtVoxel(keyVi);
	    // std::cout << "----- size of neighbour leaf is: " << neighborLeafKeys.size() << std::endl;
	    
	    for (int idxVj = 0; idxVj < neighborLeafKeys.size(); ++idxVj)
	    {
		pcl::octree::LeafNodeKey keyVj = neighborLeafKeys[idxVj]; // idx of neighbor voxel
		
		VoxelInfo voxelVj;
		if( cellInfos.find(keyVj) != cellInfos.end())
		    voxelVj = cellInfos.find(keyVj)->second;
		else
		    continue;
		
		// judgement1: the voxel has been blocked
		if (voxelVj.blocked) { continue; }
		
		// judgement2: normal deviation
		cv::Matx31d normalVi = voxelVi.normal;
		cv::Matx31d normalVj = voxelVj.normal;
		double normalDev = abs(normalVj.val[0] * normalVi.val[0] + normalVj.val[1] * normalVi.val[1] + normalVj.val[2] * normalVi.val[2]);
		if (normalDev < cos(angle_threshold_)) { continue;}
		
		Rc.push_back(keyVj);
		numSegmentPoints += voxelVj.idxAll.size();
		cellInfos.find(keyVj)->second.blocked = true;
		sizeBlocked++;
		
		if (voxelVj.residual < res_threshold_) {
		    Sc.push_back(keyVj);
		}
	    }
	    // std::cout << "----- size of blocked voxel is: " << sizeBlocked << "\n" << std::endl;
	}
	
	if (numSegmentPoints > min_valid_points_)
	{
	    R.push_back(Rc);
	    for (int k=0; k<Rc.size(); ++k)
	    {
		pcl::octree::LeafNodeKey key = Rc[k];
		cellInfos.find(key)->second.allocated = true;
	    }
	}
    } 
    
    segmentClusters = R;
    std::cout << "size of segment clusters is: " << segmentClusters.size() << std::endl;
}
