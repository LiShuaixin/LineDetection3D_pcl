#ifndef _COMMON_FUNCTIONS_
#define _COMMON_FUNCTIONS_
#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/random_sample.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h> // this must before opencv

#include <opencv2/opencv.hpp>
#include <vector>

#include "nanoflann.hpp"
#include "utils.h"

using namespace cv;
using namespace std;
using namespace nanoflann;

typedef pcl::PointXYZI PointT;

class PointCloudFunctions
{
public:
    PointCloudFunctions(void){};
    ~PointCloudFunctions(void){};
    
    struct CloudPointIndexIdx 
    {
	unsigned int idx;
	unsigned int cloud_point_index;

	CloudPointIndexIdx(unsigned int idx_, unsigned int cloud_point_index_) : idx (idx_), cloud_point_index (cloud_point_index_) {}
	bool operator < (const CloudPointIndexIdx &p) const { return (idx < p.idx); }
    };
    
    static void getMinMax3D(const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
                       Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt);
    
    static void approximateVoxelGridFilter(pcl::PointCloud<PointT>::Ptr input_,
					   const double& leaf_size_,
					   pcl::PointCloud<PointT> &output);
};

class LineFunctions
{
public:
    LineFunctions(void){};
    ~LineFunctions(void){};

public:
    static void lineFitting( int rows, int cols, std::vector<cv::Point> &contour, double thMinimalLineLength, std::vector<std::vector<cv::Point2d> > &lines );

    static void subDivision( std::vector<std::vector<cv::Point> > &straightString, std::vector<cv::Point> &contour, int first_index, int last_index
	    , double min_deviation, int min_size  );

    static void lineFittingSVD( cv::Point *points, int length, std::vector<double> &parameters, double &maxDev );
};

struct PCAInfo
{
    double lambda0, scale;
    cv::Matx31d normal, planePt;
    std::vector<int> idxAll, idxIn;

    PCAInfo &operator =(const PCAInfo &info)
    {
	this->lambda0 = info.lambda0;
	this->normal = info.normal;
	this->idxIn = info.idxIn;
	this->idxAll = info.idxAll;
	this->scale = scale;
	return *this;
    }
};

class PCAFunctions 
{
public:
    PCAFunctions(void){};
    ~PCAFunctions(void){};

    void Ori_PCA( pcl::PointCloud<PointT>::Ptr cloud_ptr, int k, std::vector<PCAInfo> &pcaInfos, double &scale, double &magnitd );

    void PCASingle( pcl::PointCloud<PointT>::Ptr cloud_ptr, PCAInfo &pcaInfo );

    void MCMD_OutlierRemoval( pcl::PointCloud<PointT>::Ptr cloud_ptr, PCAInfo &pcaInfo );

    double meadian( std::vector<double> dataset );
};

#endif //_COMMON_FUNCTIONS_
