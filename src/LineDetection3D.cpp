﻿#include "LineDetection3D.h"
#include <omp.h>

#include "CommonFunctions.h"
#include "Timer.h"

using namespace std;
using namespace cv;

LineDetection3D::LineDetection3D() : pointData(new pcl::PointCloud<PointT>())
{
}

LineDetection3D::~LineDetection3D()
{
}

void LineDetection3D::run( pcl::PointCloud<PointT>::Ptr data, int k, std::vector<PLANE> &planes, 
			   std::vector<std::vector<cv::Point3d> > &lines, std::vector<double> &ts  )
{
	this->pointData = data;
	this->pointNum = data->size();
	this->k = k;

	// step1: point cloud segmentation
	double totalTime = 0.0;
	CTimer timer;
	char msg[1024];

	timer.Start();
	cout<<endl<<endl;
	cout<<"Step1: Point Cloud Segmentation ..."<<endl;
	std::vector<std::vector<int> > regions;
	pointCloudSegmentation( regions );
	timer.Stop();
	totalTime += timer.GetElapsedSeconds();
	timer.PrintElapsedTimeMsg(msg);
	printf("  Point Cloud Segmentation Time: %s.\n\n", msg);
	ts.push_back(timer.GetElapsedSeconds());

	// step2: plane based 3D line detection
	timer.Start();
	cout<<"Step2: Plane Based 3D LineDetection ..."<<endl;
	planeBased3DLineDetection( regions, planes );
	timer.Stop();
	totalTime += timer.GetElapsedSeconds();
	timer.PrintElapsedTimeMsg(msg);
	printf("  Plane Based 3D LineDetection Time: %s.\n\n", msg);
	ts.push_back(timer.GetElapsedSeconds());

	// step3: post processing
	timer.Start();
	cout<<"Step3: Post Processing ..."<<endl;
	postProcessing( planes, lines );
	timer.Stop();
	totalTime += timer.GetElapsedSeconds();
	timer.PrintElapsedTimeMsg(msg);
	printf("  Post Processing Time: %s.\n\n", msg);
	ts.push_back(timer.GetElapsedSeconds());

	printf("Total Time: %lf.\n\n", totalTime);
}


void LineDetection3D::pointCloudSegmentation( std::vector<std::vector<int> > &regions )
{
    cout<<"----- Normal Calculation ..."<<endl;
    PCAFunctions pcaer;
    pcaer.Ori_PCA( this->pointData, this->k, this->pcaInfos, this->scale, this->magnitd );
    
    cout<<"----- Region Growing ..."<<endl;
    double thAngle = 15.0/180.0*CV_PI;
    regionGrow( thAngle, regions );

    // step3: region merging
    cout<<"----- Region Merging ..."<<endl;
    double thAnglePatch = thAngle;
    regionMerging( thAnglePatch, regions );
}


void LineDetection3D::regionGrow( double thAngle, std::vector<std::vector<int> > &regions )
{
    double thNormal = cos(thAngle);

    // sort according to the curvature of points
    std::vector<std::pair<int,double> > idxSorted( this->pointNum );
    for ( int i=0; i<this->pointNum; ++i )
    {
	idxSorted[i].first = i; // point idx
	idxSorted[i].second = pcaInfos[i].lambda0; // curvature
    }
    std::sort( idxSorted.begin(), idxSorted.end(), [](const std::pair<int,double>& lhs, const std::pair<int,double>& rhs) { return lhs.second < rhs.second; } );

    // get the initial clusters
    double percent = 0.9;
    int N = int(this->pointNum*percent);
    std::vector<int> isUsed( this->pointNum, 0 );
    for ( int i=0; i<N; ++i )
    {
	int idxStrater = idxSorted[i].first; /// begin from the point with the smallest curvature
	if ( isUsed[idxStrater] ) { continue; }
	
	cv::Matx31d normalStarter = pcaInfos[idxStrater].normal;
	double xStrater = pointData->points[idxStrater].x, yStrater = pointData->points[idxStrater].y, zStrater = pointData->points[idxStrater].z;
	double thRadius2 = pow(50*pcaInfos[idxStrater].scale, 2);

	std::vector<int> clusterTemp;
	clusterTemp.reserve(10000);
	clusterTemp.push_back( idxStrater );
	int count = 0;
	while( count < clusterTemp.size() ) /// traverse all candidates in the cluster
	{
	    int idxSeed = clusterTemp[count];
	    cv::Matx31d normalSeed = pcaInfos[idxSeed].normal;
	    double thOrtho = pcaInfos[idxSeed].scale;

	    // point cloud collection
	    int num = pcaInfos[idxSeed].idxAll.size(); /// num of neighbors
	    for( int j = 0; j < num; ++j )
	    {
		int idxCur = pcaInfos[idxSeed].idxAll[j];
		if (isUsed[idxCur]) { continue; }

		// judgement1: normal deviation
		cv::Matx31d normalCur = pcaInfos[idxCur].normal;
		double normalDev = abs(normalCur.val[0] * normalStarter.val[0] + normalCur.val[1] * normalStarter.val[1] + normalCur.val[2] * normalStarter.val[2]);
		//double normalDev = abs(normalCur.val[0] * normalSeed.val[0] + normalCur.val[1] * normalSeed.val[1] + normalCur.val[2] * normalSeed.val[2]);
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

	if ( clusterTemp.size() > 30 ) /// only cluster with enough points could be stored
	{
	    regions.push_back( clusterTemp );
	}
	else
	{
	    for (int j=0; j<clusterTemp.size(); ++j)
	    {
		isUsed[clusterTemp[j]] = 0;
	    }
	}
    }
}

void LineDetection3D::regionMerging( double thAngle, std::vector<std::vector<int> > &regions )
{
    double thRegionSize = 600000;

    // step1: plane fitting via PCA for each region
    std::vector<PCAInfo> patches;
    patches.resize( regions.size() );

#pragma omp parallel for
    for ( int i=0; i<regions.size(); ++i )
    {
	int pointNumCur = regions[i].size();
	pcl::PointCloud<PointT>::Ptr pointDataCur(new pcl::PointCloud<PointT>());
	for ( int j=0; j<pointNumCur; ++j )
	{
	    PointT pt;
	    pt.x = pointData->points[regions[i][j]].x;
	    pt.y = pointData->points[regions[i][j]].y;
	    pt.z = pointData->points[regions[i][j]].z;
	    
	    pointDataCur->push_back(pt);
	}

	/// select the inlier points in the current cluster
	PCAFunctions pcaer;
	pcaer.PCASingle( pointDataCur, patches[i] );

	patches[i].idxAll = regions[i];
	double scaleAvg = 0.0;
	for ( int j=0; j<patches[i].idxIn.size(); ++j )
	{
	    int idx = regions[i][patches[i].idxIn[j]];
	    patches[i].idxIn[j] = idx; /// transform the patch's inlier idx as the idx of cloud_ptr
	    scaleAvg += pcaInfos[idx].scale;
	}
	scaleAvg /= patches[i].idxIn.size();
	patches[i].scale = 5.0 * scaleAvg;
    }

    // get the patch label of each point
    std::vector<int> label( this->pointNum, -1 );
#pragma omp parallel for
    for ( int i=0; i<regions.size(); ++i )
    {
	for ( int j=0; j<regions[i].size(); ++j )
	{
	    int id = regions[i][j];
	    label[id] = i;
	}
    }

    // step2: find adjacent patches
    std::vector<std::vector<int> > patchAdjacent( patches.size() );
#pragma omp parallel for
    for ( int i=0; i<patches.size(); ++i )
    {
	std::vector<int> patchAdjacentTemp;
	std::vector<std::vector<int> > pointAdjacentTemp;
	for ( int j=0; j<patches[i].idxIn.size(); ++j )
	{
	    int id = patches[i].idxIn[j];
	    for ( int m=0; m<pcaInfos[id].idxIn.size(); ++m )
	    {
		int idPoint = pcaInfos[id].idxIn[m];
		int labelPatch = label[idPoint];
		if ( labelPatch == i || labelPatch < 0 ) { continue; }

		bool isNeighbor = false;
		for ( int n=0; n<pcaInfos[idPoint].idxIn.size(); ++n )
		{
		    if ( pcaInfos[idPoint].idxIn[n] == id )
		    {
			isNeighbor = true;
		    }
		}
		if ( ! isNeighbor ) { continue; }

		// accept the patch as a neighbor
		bool isIn = false;
		int n = 0;
		for ( n=0; n<patchAdjacentTemp.size(); ++n )
		{
		    if ( patchAdjacentTemp[n] == labelPatch )
		    {
			isIn = true;
			break;
		    }
		}

		if ( isIn )
		{
		    pointAdjacentTemp[n].push_back( idPoint );
		}
		else
		{
		    patchAdjacentTemp.push_back( labelPatch );

		    std::vector<int> temp;
		    temp.push_back( idPoint );
		    pointAdjacentTemp.push_back( temp );
		}
	    }
	}

	// repetition removal
	for ( int j=0; j<pointAdjacentTemp.size(); ++j )
	{
	    std::sort(pointAdjacentTemp[j].begin(), pointAdjacentTemp[j].end());  
	    vector<int>::iterator new_end = unique(pointAdjacentTemp[j].begin(), pointAdjacentTemp[j].end());
	    pointAdjacentTemp[j].erase(new_end, pointAdjacentTemp[j].end());

	    if ( pointAdjacentTemp[j].size() >= 3 )
	    {
		patchAdjacent[i].push_back( patchAdjacentTemp[j] );
	    }
	}
    }

    // try to merge adjacent patch
    regions.clear();
    std::vector<int> mergedIndex( patches.size(), 0 );
    for ( int i=0; i<patches.size(); ++i )
    {
	if ( !mergedIndex[i] )
	{
	    int idxStarter = i;
	    cv::Matx31d normalStarter = patches[idxStarter].normal;
	    cv::Matx31d ptStarter = patches[idxStarter].planePt;

	    std::vector<int> patchIdx;
	    patchIdx.push_back( idxStarter );

	    int count = 0;
	    int totalPoints = 0;
	    bool isEnough = false;
	    while ( count < patchIdx.size() )
	    {
		int idxSeed = patchIdx[count];
		cv::Matx31d normalSeed = patches[idxSeed].normal;
		cv::Matx31d ptSeed = patches[idxSeed].planePt;
		double thOrtho = patches[idxSeed].scale;

		for ( int j=0; j<patchAdjacent[idxSeed].size(); ++j )
		{
		    int idxCur = patchAdjacent[idxSeed][j];

		    if ( mergedIndex[idxCur] )
		    {
			continue;
		    }

		    cv::Matx31d normalCur = patches[idxCur].normal;
		    cv::Matx31d ptCur = patches[idxCur].planePt;

		    // plane angle deviation and distance
		    double devAngle = 0.0;
		    double devDis = 0.0;
		    double thDev = 0.0;

		    cv::Matx31d ptVector1 = ptCur - ptStarter;
		    cv::Matx31d ptVector2 = ptCur - ptSeed;
		    devAngle = acos( normalStarter.val[0] * normalCur.val[0] + normalStarter.val[1] * normalCur.val[1] + normalStarter.val[2] * normalCur.val[2] );
		    devDis = abs( normalStarter.val[0] * ptVector1.val[0] + normalStarter.val[1] * ptVector1.val[1] + normalStarter.val[2] * ptVector1.val[2] );

		    if ( min( devAngle, fabs( CV_PI - devAngle ) ) < thAngle && devDis < thOrtho )
		    {
			patchIdx.push_back( idxCur );
			mergedIndex[idxCur] = 1;

			totalPoints += patches[idxCur].idxAll.size();
			if ( totalPoints > thRegionSize )
			{
			    isEnough = true;
			    break;
			}
		    }
		}

		if ( isEnough )
		{
		    break;
		}
		count ++;
	    }

	    // create a new cluster
	    std::vector<int> patchNewCur;
	    for ( int j=0; j<patchIdx.size(); ++j )
	    {
		int idx = patchIdx[j];

		for ( int m=0; m<patches[idx].idxAll.size(); ++m )
		{
		    patchNewCur.push_back( patches[idx].idxAll[m] );
		}
	    }

	    // 
	    if (patchNewCur.size() > 100)
	    {
		regions.push_back( patchNewCur );
	    }
	}
    }
}

void LineDetection3D::planeBased3DLineDetection( std::vector<std::vector<int> > &regions, std::vector<PLANE> &planes )
{
    double thAngle = 10.0/180.0*CV_PI;
    double thLineLength = 8*this->scale;
    int numPatches = regions.size();

    // step1: fitting 3D plane via PCA
    std::vector<PCAInfo> patches(numPatches);
#pragma omp parallel for
    for ( int i=0; i<numPatches; ++i )
    {
	int pointNumCur = regions[i].size();
	pcl::PointCloud<PointT>::Ptr pointDataCur(new pcl::PointCloud<PointT>());
	for ( int j=0; j<pointNumCur; ++j )
	{
	    PointT pt;
	    pt.x = pointData->points[regions[i][j]].x;
	    pt.y = pointData->points[regions[i][j]].y;
	    pt.z = pointData->points[regions[i][j]].z;
	    
	    pointDataCur->push_back(pt);
	}

	/// select the inlier points in the current cluster
	PCAFunctions pcaer;
	pcaer.PCASingle( pointDataCur, patches[i] );

	patches[i].idxAll = regions[i];
	for ( int j=0; j<patches[i].idxIn.size(); ++j )
	{
	    int idx = patches[i].idxIn[j];
	    patches[i].idxIn[j] = regions[i][idx];
	}
    }

    // step2: 3D line detection
    planes.resize(patches.size());
#pragma omp parallel for
    for(int i=0; i<patches.size(); ++i)
    {
	// A. 3D-2D Projection: project the 3d point onto the plane coordinate
	std::vector<cv::Point2d> pts2d;
	std::vector<double> ptScales;
	    
	bool initialized = false;
	cv::Mat_<double> vX, vY;
	cv::Mat_<double> planePt = (cv::Mat_<double>(3,1) << patches[i].planePt.val[0], patches[i].planePt.val[1], patches[i].planePt.val[2]);
	cv::Mat_<double> normal  = (cv::Mat_<double>(3,1) << patches[i].normal.val[0], patches[i].normal.val[1], patches[i].normal.val[2]);

	for(int j=0; j<patches[i].idxAll.size(); ++j)
	{
	    int id = patches[i].idxAll[j];
	    cv::Mat_<double> pt3d = (cv::Mat_<double>(3,1) << pointData->points[id].x, pointData->points[id].y, pointData->points[id].z );

	    cv::Mat_<double> v3d = pt3d - planePt;
	    cv::Mat_<double> vOrtho = v3d.dot(normal) * normal;
	    cv::Mat_<double> vPlane = v3d - vOrtho;
	    cv::Mat_<double> ptPlane = planePt + vPlane;

	    if(!initialized)
	    {
		vX = vPlane * 1.0/(cv::norm(vPlane));
		vY = vX.cross(normal);
		vY = vY * 1.0/cv::norm(vY);
		initialized = true;
	    }
	    if( initialized )
	    {
		double x = vPlane.dot(vX);
		double y = vPlane.dot(vY);
		pts2d.push_back(cv::Point2d(x,y));
		ptScales.push_back(pcaInfos[id].scale);
	    }
	}

	// A. 3D-2D Projection: get the side length of the grid cell
	double gridSideLength = 0;
	std::sort( ptScales.begin(), ptScales.end(), [](const double& lhs, const double& rhs) { return lhs < rhs; } );
	int idxNinety = min( int(double(ptScales.size()) * 0.9), int(ptScales.size()-1) );
	gridSideLength = ptScales[idxNinety] * 0.75;

	// A. 3D-2D Projection: get the binary image of the plane
	double xmin, ymin, xmax, ymax;
	int margin = 0;
	cv::Mat mask;
	bool isok = maskFromPoint( pts2d, gridSideLength, xmin, ymin, xmax, ymax, margin, mask );
	if ( !isok )
	{
	    continue;
	}

	// B. 2D Line Detection
	int thLineLengthPixel = max(thLineLength/gridSideLength,10.0);
	std::vector<std::vector<std::vector<cv::Point2d> > > lines2d;
	lineFromMask( mask, thLineLengthPixel, lines2d );
	if (!lines2d.size())
	{
	    continue;
	}

	// C. 2D-3D Projection
	planes[i].scale = gridSideLength;
	for ( int m=0; m<lines2d.size(); ++m ) 
	{
	    std::vector<std::vector<cv::Point3d> > temp;
	    for (int n=0; n<lines2d[m].size(); ++n)
	    {
		double length = abs(lines2d[m][n][1].x-lines2d[m][n][0].x) + abs(lines2d[m][n][1].y-lines2d[m][n][0].y);
		if ( length < thLineLengthPixel )
		{
		    continue;
		}

		lines2d[m][n][0].x = (lines2d[m][n][0].x - margin) * gridSideLength + xmin;
		lines2d[m][n][0].y = (lines2d[m][n][0].y - margin) * gridSideLength + ymin;

		lines2d[m][n][1].x = (lines2d[m][n][1].x - margin) * gridSideLength + xmin;
		lines2d[m][n][1].y = (lines2d[m][n][1].y - margin) * gridSideLength + ymin;

		cv::Mat_<double> xs = lines2d[m][n][0].x * vX;
		cv::Mat_<double> ys = lines2d[m][n][0].y * vY;
		cv::Mat_<double> pts = planePt + xs + ys;

		cv::Mat_<double> xe = lines2d[m][n][1].x * vX;
		cv::Mat_<double> ye = lines2d[m][n][1].y * vY;
		cv::Mat_<double> pte = planePt + xe + ye;

		std::vector<cv::Point3d> line3dTemp(2);
		line3dTemp[0] = cv::Point3d(pts(0), pts(1), pts(2));
		line3dTemp[1] = cv::Point3d(pte(0), pte(1), pte(2));

		temp.push_back( line3dTemp );
	    }
	    if (temp.size())
	    {
		planes[i].lines3d.push_back(temp);
	    }
	}
    }
}

bool LineDetection3D::maskFromPoint( std::vector<cv::Point2d> &pts2d, double radius, double &xmin, double &ymin, double &xmax, double &ymax, int &margin, cv::Mat &mask )
{
    xmin=10000000, ymin = 10000000;
    xmax=-xmin;
    ymax=-ymin;
    for (int i=0; i<pts2d.size(); ++i)
    {
	if(pts2d[i].x < xmin) { xmin = pts2d[i].x; }
	if(pts2d[i].x > xmax) { xmax = pts2d[i].x; }

	if(pts2d[i].y < ymin) { ymin = pts2d[i].y; }
	if(pts2d[i].y > ymax) { ymax = pts2d[i].y; }
    }

    margin = 4;
    int cols = (xmax-xmin) / radius + 2*margin;
    int rows = (ymax-ymin) / radius + 2*margin;
    if ( cols < 10 || rows < 10 )
    {
	return false;
    }

    mask = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(0));
    for (int i=0; i<pts2d.size(); ++i)
    {
	int xInt = int((pts2d[i].x-xmin)/radius+0.5+margin);
	int yInt = int((pts2d[i].y-ymin)/radius+0.5+margin);
	mask.at<uchar>(yInt,xInt) = 255;
    }
    return true;
}

void LineDetection3D::lineFromMask( cv::Mat &mask, int thLineLengthPixel, std::vector<std::vector<std::vector<cv::Point2d> > > &lines )
{
    lines.clear();

    // get mask image via dilate and erode
    cv::Mat mask2;
    cv::dilate(mask, mask2, cv::Mat());
    cv::erode(mask2, mask2, cv::Mat());

    // A. contours
    double thLength = thLineLengthPixel;
    
    std::vector<std::vector<cv::Point> > contours;  
    std::vector<cv::Vec4i> hierarchy;  
    cv::findContours(mask2, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    // B. line fitting from the contours
    for ( int i=0; i<contours.size(); ++i )
    {
	if ( contours[i].size() < 4*thLength  )
	{
	    continue;
	}

	std::vector<std::vector<cv::Point2d> > lineTemp;
	LineFunctions::lineFitting( mask2.rows, mask2.cols, contours[i], thLength, lineTemp );
	lines.push_back(lineTemp);
    }
}


void LineDetection3D::postProcessing( std::vector<PLANE> &planes, std::vector<std::vector<cv::Point3d> > &lines )
{
    // step1: plane line regularization
    outliersRemoval( planes );

    // step2: line merging
    lineMerging( planes, lines );
}

void LineDetection3D::outliersRemoval( std::vector<PLANE> &planes )
{
    double thCosAngleIN = cos(12.5/180.0*CV_PI);
    double thCosAngleNEW = cos(30.0/180.0*CV_PI);
    double thNonStructPlaneRatio = 0.3;
    double thAngle = 12.5;
    double thCosAngleParal = cos(thAngle/180.0*CV_PI);
    double thCosAngleOrtho = cos((90.0-thAngle)/180.0*CV_PI);
    double thNonStructLineRatio = 10;
    double thStructPlane = 60*this->scale;

    std::vector<int> isPlaneGood(planes.size(), 0);
#pragma omp parallel for
    for (int i=0; i<planes.size(); ++i)
    {
	if (!planes[i].lines3d.size())
	{
	    continue;
	}

	// step1: remove non-structural planes
	std::vector<double> lengthsAll;
	std::vector<cv::Mat> orientsAll;
	std::vector<std::pair<int, double> > lineInfos;
	std::vector<std::vector<double> > lengths(planes[i].lines3d.size());
	std::vector<std::vector<cv::Mat> > orients(planes[i].lines3d.size());

	double totalLength = 0.0;
	int count = 0;
	for (int m=0; m<planes[i].lines3d.size(); ++m)
	{
	    lengths[m].resize(planes[i].lines3d[m].size());
	    orients[m].resize(planes[i].lines3d[m].size());
	    for (int n=0; n<planes[i].lines3d[m].size(); ++n)
	    {
		cv::Mat orientTemp = cv::Mat(planes[i].lines3d[m][n][1] - planes[i].lines3d[m][n][0]);
		double lengthTemp = cv::norm(orientTemp);
		lengthsAll.push_back(lengthTemp);
		lengths[m][n] = lengthTemp;

		orientTemp *= 1.0/lengthTemp;
		orientsAll.push_back(orientTemp);
		orients[m][n] = orientTemp;

		std::pair<int, double> lineInfoTemp(count, lengthTemp);
		lineInfos.push_back(lineInfoTemp);

		totalLength += lengthTemp;
		count ++;
	    }
	}
	std::sort( lineInfos.begin(), lineInfos.end(), [](const std::pair<int, double>& lhs, const std::pair<int, double>& rhs) { return lhs.second > rhs.second; } );

	std::vector<cv::Mat> clusterOrient;
	std::vector<std::pair<int, double> > clusterInfos;
	for (int j=0; j<lineInfos.size(); ++j)
	{
	    int id = lineInfos[j].first;
	    double length = lineInfos[j].second;

	    if (!clusterInfos.size())
	    {
		clusterInfos.push_back(std::pair<int, double>(clusterInfos.size(), length));
		clusterOrient.push_back(orientsAll[id]);
		continue;
	    }

	    bool isIn = false;
	    double cosValueMin = 100;
	    for (int m=0; m<clusterInfos.size(); ++m)
	    {
		double cosValue = abs(orientsAll[id].dot(clusterOrient[m]));
		if ( cosValue < cosValueMin )
		{
		    cosValueMin =  cosValue;
		}
		if (cosValue > thCosAngleIN)
		{
		    clusterInfos[m].second += length;
		    isIn = true;
		    break;
		}
	    }

	    if (!isIn && cosValueMin < thCosAngleNEW)
	    {
		clusterInfos.push_back(std::pair<int, double>(clusterInfos.size(), length));
		clusterOrient.push_back(orientsAll[id]);
		continue;
	    }
	}

	double scaleCur = max(this->scale,planes[i].scale);
	if ( clusterInfos.size() > 1)
	{
	    double LStruct =  clusterInfos[0].second + clusterInfos[1].second;
	    if( LStruct < thNonStructPlaneRatio*totalLength || LStruct < thStructPlane ) 
	    {
		continue;
	    }
	}

	// step2: remove non-structural lines
	PLANE planeNew;
	planeNew.scale = planes[i].scale;
	//double scaleCur = planes[i].scale;
	double thNonStructLineLength = scaleCur*thNonStructLineRatio;
	for (int m=0; m<planes[i].lines3d.size(); ++m)
	{
	    int numLines = planes[i].lines3d[m].size();

	    double lengthTotal = 0.0;
	    for (int n=0; n<numLines; ++n)
	    {
		lengthTotal += lengths[m][n];
	    }

	    double ratioStruct = 0.0;
	    double lengthStruct = 0.0;
	    std::vector<int> isStruct(numLines, 0);
	    if (numLines > 1)
	    {
		// judge if the contour is structural
		std::vector<int> idxOrthoPara;
		for (int n=0; n<numLines-1; ++n)
		{
		    int id1 = n;
		    int id2 = (n+1)%numLines;

		    double cosAngle = abs(orients[m][id1].dot(orients[m][id2]));
		    if (cosAngle > thCosAngleParal || cosAngle < thCosAngleOrtho)
		    {
			idxOrthoPara.push_back(id1);
			idxOrthoPara.push_back(id2);
		    }
		}

		if (idxOrthoPara.size())
		{
		    // structural ratio
		    std::sort( idxOrthoPara.begin(), idxOrthoPara.end(), [](const int& lhs, const int& rhs) { return lhs > rhs; } );

		    int idTemp = idxOrthoPara[0];
		    isStruct[idTemp] = 1;
		    lengthStruct = lengths[m][idTemp];
		    for (int n=0; n<idxOrthoPara.size(); ++n)
		    {
			if (idxOrthoPara[n] != idTemp)
			{
			    lengthStruct += lengths[m][idxOrthoPara[n]];
			    idTemp = idxOrthoPara[n];
			    isStruct[idTemp] = 1;
			}
		    }

		    ratioStruct = lengthStruct/lengthTotal;
		}
	    }

	    std::vector<std::vector<cv::Point3d> > contourTemp;
	    for (int n=0; n<numLines; ++n)
	    {
		double thLengthTemp = 0.0;
		if (isStruct[n])
		{
		    if(ratioStruct>=0.75) 
		    {
			thLengthTemp = thNonStructLineLength;
		    }
		    else if (ratioStruct>=0.5) 
		    {
			thLengthTemp = 2*thNonStructLineLength;
		    }
		    else 
		    {
			thLengthTemp = 4*thNonStructLineLength;
		    }
		}
		else
		{
		    thLengthTemp = 4*thNonStructLineLength;
		}

		if (lengths[m][n] > thLengthTemp)
		{
		    contourTemp.push_back(planes[i].lines3d[m][n]);
		}
	    }
	    if (contourTemp.size())
	    {
		planeNew.lines3d.push_back(contourTemp);
	    }
	}

	if (planeNew.lines3d.size())
	{
	    planes[i] = planeNew;
	    isPlaneGood[i] = 1;
	}
    }

    //
    std::vector<PLANE> planesNew;
    for (int i=0; i<isPlaneGood.size(); ++i)
    {
	if (isPlaneGood[i])
	{
	    planesNew.push_back(planes[i]);
	}
    }
    planes = planesNew;
}

void LineDetection3D::lineMerging( std::vector<PLANE> &planes, std::vector<std::vector<cv::Point3d> > &lines )
{
    double thGapRatio = 20;
    double thMergeRatio = 6;
    double thDisHyps = 0.1;

    // get all the lines
    std::vector<double> lineScales;
    for (int i=0; i<planes.size(); ++i)
    {
	for (int m=0; m<planes[i].lines3d.size(); ++m)
	{
	    for (int n=0; n<planes[i].lines3d[m].size(); ++n)
	    {
		lines.push_back(planes[i].lines3d[m][n]);
		lineScales.push_back(planes[i].scale);
	    }
	}
    }

    // get the parameters of each 3d line
    std::vector<std::vector<double> > lineParas(lines.size()) ;
    std::vector<std::pair<int, double> > lineInfos(lines.size());
    for ( int i=0; i<lines.size(); ++i )
    {
	cv::Mat v(lines[i][1]-lines[i][0]);
	double length = cv::norm(v);
	v *= 1.0/length;

	cv::Mat ptmid((lines[i][1]+lines[i][0])*0.5);
	cv::Mat d = v.cross(ptmid)*(1.0/this->magnitd);

	// get the latitude of the line, longitude is not stable
	double latitude = asin(abs(v.at<double>(2)));

	// the length of the line
	lineParas[i].resize(6);
	lineParas[i][0] = v.at<double>(0);       lineParas[i][1] = v.at<double>(1);       lineParas[i][2] = v.at<double>(2);
	lineParas[i][3] = latitude;   
	lineParas[i][4] = cv::norm(d); 
	lineParas[i][5] = length; 

	lineInfos[i] = std::pair<int,double>(i, length);
    }
    std::sort( lineInfos.begin(), lineInfos.end(), [](const std::pair<int,double>& lhs, const std::pair<int,double>& rhs) { return lhs.second > rhs.second; } );

    // build grid with latitude
    double precision = 6.0/180.0*CV_PI;
    int laSize = CV_PI/2.0/precision;
    std::vector<std::vector<int > > grid(laSize);
    std::vector<int> gridIndex(lineParas.size());
    for ( int i=0; i<lineParas.size(); ++i )
    {
	int la = lineParas[i][3]/precision;
	grid[la].push_back(i);
	gridIndex[i] = la;
    }

    // line merging
    std::vector<bool> isUsed(lines.size(), 0);
    std::vector<std::vector<cv::Point3d> > linesNew;
    for ( int i=0; i<lineInfos.size(); ++i )
    {
	int id0 = lineInfos[i].first;
	if ( isUsed[id0] )
	{
	    continue;
	}
	isUsed[id0] = 1;

	double lineScale = max(lineScales[id0], this->scale);
	double vx0 = lineParas[id0][0], vy0 = lineParas[id0][1], vz0 = lineParas[id0][2];
	double d0 = lineParas[id0][4], length0 = lineParas[id0][5];
	cv::Point3d pts0 = lines[id0][0], pte0 = lines[id0][1];

	// get the merging hypotheses
	std::vector<int> idHyps;
	for (int j=-1; j<=1; ++j)
	{
	    int latemp = gridIndex[id0]+j;
	    int la = (latemp+laSize)%laSize;
	    for ( int m=0; m<grid[la].size(); ++m )
	    {
		int idTemp = grid[la][m];
		if (abs(lineParas[idTemp][4]-d0) < thDisHyps)
		{
		    idHyps.push_back(idTemp);
		}
	    }
	}

	// try merging
	for (int j=0; j<idHyps.size(); ++j)
	{
	    int id1 = idHyps[j];
	    if ( isUsed[id1] )
	    {
		continue;
	    }

	    cv::Point3d pts1 = lines[id1][0], pte1 = lines[id1][1];
	    double length1 = lineParas[id1][5];

	    // judge the distance between two line
	    cv::Point3d v1 = pts0 - pts1;
	    double disNormal1 = v1.x*vx0 + v1.y*vy0 + v1.z*vz0;
	    cv::Point3d vOrtho1 = v1 - disNormal1*cv::Point3d(vx0, vy0, vz0);
	    double disOrtho1 = sqrt(vOrtho1.x*vOrtho1.x + vOrtho1.y*vOrtho1.y + vOrtho1.z*vOrtho1.z);

	    cv::Point3d v2 = pts0 - pte1;
	    double disNormal2 = v2.x*vx0 + v2.y*vy0 + v2.z*vz0;
	    cv::Point3d vOrtho2 = v2 - disNormal2*cv::Point3d(vx0, vy0, vz0);
	    double disOrtho2 = sqrt(vOrtho2.x*vOrtho2.x + vOrtho2.y*vOrtho2.y + vOrtho2.z*vOrtho2.z);

	    if ( disOrtho1 > thMergeRatio*lineScale || disOrtho2 > thMergeRatio*lineScale )
	    {
		continue;
	    }

	    // judge the overlapping ratio of two line
	    cv::Point3d d1 = pts0 - pts1, d2 = pts0 - pte1, d3 = pte0 - pts1, d4 = pte0 - pte1;
	    double dis1 = sqrt(d1.x*d1.x + d1.y*d1.y + d1.z*d1.z);
	    double dis2 = sqrt(d2.x*d2.x + d2.y*d2.y + d2.z*d2.z);
	    double dis3 = sqrt(d3.x*d3.x + d3.y*d3.y + d3.z*d3.z);
	    double dis4 = sqrt(d4.x*d4.x + d4.y*d4.y + d4.z*d4.z);
	    double disMerge = max( max(dis1, dis2), max(dis3, dis4) );

	    double gapLength = disMerge - length0 - length1;
	    double gapRatio = gapLength / length0;
	    if ( gapRatio < 0.1 && gapLength < thGapRatio*lineScale )
	    {
		// update line id0
		if (gapRatio > 0)
		{
		    if (dis1 == disMerge)
		    {
			double disNormal = d1.x*vx0 + d1.y*vy0 + d1.z*vz0;
			lines[id0][1] = pts0 - disNormal*cv::Point3d(vx0, vy0, vz0);
		    }
		    else if (dis2 == disMerge)
		    {
			double disNormal = d2.x*vx0 + d2.y*vy0 + d2.z*vz0;
			lines[id0][1] = pts0 - disNormal*cv::Point3d(vx0, vy0, vz0);
		    }
		    else if (dis3 == disMerge)
		    {
			double disNormal = d3.x*vx0 + d3.y*vy0 + d3.z*vz0;
			lines[id0][0] = pte0 - disNormal*cv::Point3d(vx0, vy0, vz0);
		    }
		    else
		    {
			double disNormal = d4.x*vx0 + d4.y*vy0 + d4.z*vz0;
			lines[id0][0] = pte0 - disNormal*cv::Point3d(vx0, vy0, vz0);
		    }
		}

		isUsed[id1] = 1;
	    }
	}

	linesNew.push_back(lines[id0]);
    }

    lines = linesNew;
}
