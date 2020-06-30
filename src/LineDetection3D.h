#ifndef _LINE_DETECTION_H_
#define _LINE_DETECTION_H_
#pragma once

#include "CommonFunctions.h"

struct PLANE
{
    double scale;
    std::vector<std::vector<std::vector<cv::Point3d> > > lines3d;
    
    // for visualization and motion estimation
    pcl::PointCloud<PointT> points;
    double curvature;
    Eigen::Matrix3d covariance;
    Eigen::Vector3d mean;
    Eigen::Vector3d norm;
    Eigen::Vector3d norm_test;
    double negative_OA_dot_norm;
    double residual;
    bool valid;
    

    PLANE &operator =(const PLANE &info)
    {
	this->mean                 = info.mean;
	this->norm                 = info.norm;
	this->scale                = info.scale;
	this->points               = info.points;
	this->lines3d              = info.lines3d;
	this->residual             = info.residual;
	this->covariance           = info.covariance;	
	this->negative_OA_dot_norm = info.negative_OA_dot_norm;
		
	return *this;
    }
};

class LineDetection3D 
{
public:
	LineDetection3D();
	~LineDetection3D();

	void run( pcl::PointCloud<PointT>::Ptr data, int k, std::vector<PLANE> &planes, std::vector<std::vector<cv::Point3d> > &lines, std::vector<double> &ts );

	void pointCloudSegmentation( std::vector<std::vector<int> > &regions );

	void planeBased3DLineDetection( std::vector<std::vector<int> > &regions, std::vector<PLANE> &planes );

	void postProcessing( std::vector<PLANE> &planes, std::vector<std::vector<cv::Point3d> > &lines );

	// 
	void regionGrow( double thAngle, std::vector<std::vector<int> > &regions );

	void regionMerging( double thAngle, std::vector<std::vector<int> > &regions );

	bool maskFromPoint( std::vector<cv::Point2d> &pts2d, double radius, double &xmin, double &ymin, double &xmax, double &ymax, int &margin, cv::Mat &mask );

	void lineFromMask( cv::Mat &mask, int thLineLengthPixel, std::vector<std::vector<std::vector<cv::Point2d> > > &lines );

	void outliersRemoval( std::vector<PLANE> &planes );

	void lineMerging( std::vector<PLANE> &planes, std::vector<std::vector<cv::Point3d> > &lines );

public:
        /// point based parameters
	int k;
	int pointNum;
	double scale, magnitd;
	std::vector<PCAInfo> pcaInfos;
	pcl::PointCloud<PointT>::Ptr pointData;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr planePoints;
	pcl::PointCloud<pcl::Normal>::Ptr planeNormals;
	pcl::PointCloud<pcl::PointXYZ>::Ptr planeCentroid;
	
	/// voxel based parameters
	std::vector<VoxelInfo> voxelInfos;
};

#endif //_LINE_DETECTION_H_
