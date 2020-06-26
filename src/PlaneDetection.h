#ifndef _PLANE_DETECTION_H_
#define _PLANE_DETECTION_H_

#include "CommonFunctions.h"
#include <OctreePointcloudVoxel.hpp>

class PlaneDetection
{
public:
    
    PlaneDetection();
    ~PlaneDetection();
    
    void createVoxelization( pcl::PointCloud<PointT>::Ptr inputPoints, const double& initRes
                             , const double& lowestRes, const double& distThr );

    /**
     * \brief proposed by Minglei Li -> Adaptive Multi-level Octree Based Region Growing
     */
//     void runPlaneExtraction_AMG();
//     void getPlanarFeature_AMG();
    
    
    
    /**
     * \brief proposed by VO -> Octree Based Region Growing
     */
    void runPlaneExtraction_OBR(pcl::PointCloud<PointT>::Ptr inputPoints, const double& angleThreshold /*degree*/, 
				const double& resThreshold /*m*/, const int& minValidPoints);
    void getPlanarFeature_OBR();
    double getAverageDistance();
    void saliencyFeatureEstimation(std::map<pcl::octree::LeafNodeKey, VoxelInfo> &cellInfos);
    void voxelBasedRegionGrowing(std::map<pcl::octree::LeafNodeKey, VoxelInfo> &cellInfos, std::vector<std::vector<pcl::octree::LeafNodeKey>>& segmentClusters);
			
public:
    
    ///< octree based region growing parameters
    pcl::octree::OctreePointCloudVoxel<PointT>::Ptr octree_;
    pcl::PointCloud<PointT>::Ptr points_;
    double init_resolution_;
    double lowest_resolution_;
    double dist_threshold_;
    
    double res_threshold_;
    double angle_threshold_;
    int min_valid_points_;
    
};
#endif //_PLANE_DETECTION_H_