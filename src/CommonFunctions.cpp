#include "CommonFunctions.h"
  
using namespace cv;
using namespace std;

/************************************************************************/
/*                           Line Functions                             */
/************************************************************************/

void LineFunctions::lineFitting( int rows, int cols, std::vector<cv::Point> &contour, double thMinimalLineLength, std::vector<std::vector<cv::Point2d> > &lines )
{
    // get straight strings from the contour
    double minDeviation = 6.0;
    std::vector<std::vector<cv::Point> > straightString;
    subDivision(straightString, contour, 0, contour.size()-1, minDeviation, int(thMinimalLineLength));
    if ( !straightString.size() )
    {
	return;
    }
    for ( int i=0; i<straightString.size(); ++i )
    {
	if ( straightString[i].size() < thMinimalLineLength )
	{
	    continue;
	}

	std::vector<double> parameters( 4 );
	double maxDev = 0.0;
	//bool isOK = lineFittingLS( straightString[i], parameters, maxDev );
	lineFittingSVD(&straightString[i][0], straightString[i].size(), parameters, maxDev);
	//if ( isOK )
	{
	    double k = parameters[1];
	    double b = parameters[2];
	    int lineLen = straightString[i].size();

	    double xs = 0, ys = 0, xe = 0, ye = 0;
	    if ( ! parameters[0] )  // horizontal
	    {
		xs = straightString[i][0].x;
		ys = k * xs + b;
		xe = straightString[i][lineLen-1].x;
		ye = k * xe + b;
	    }
	    else   // vertical
	    {
		ys = straightString[i][0].y;
		xs = k * ys + b;
		ye = straightString[i][lineLen-1].y;
		xe = k * ye + b;
	    }

	    if ( !( xs==xe && ys==ye ) )
	    {
		std::vector<cv::Point2d> lineCur(2);
		lineCur[0] = cv::Point2d(xs, ys);
		lineCur[1] = cv::Point2d(xe, ye);

		lines.push_back( lineCur );
	    }
	}
    }
}

void LineFunctions::subDivision( std::vector<std::vector<cv::Point> > &straightString, std::vector<cv::Point> &contour, int first_index, int last_index
	, double min_deviation, int min_size )
{
    int clusters_count = straightString.size();

    cv::Point first = contour[first_index];
    cv::Point last = contour[last_index];

    // Compute the length of the straight line segment defined by the endpoints of the cluster.
    int x = first.x - last.x;
    int y = first.y - last.y;
    double length = sqrt( static_cast<double>( (x * x) + (y * y) ) );

    // Find the pixels with maximum deviation from the line segment in order to subdivide the cluster.
    int max_pixel_index = 0;
    double max_deviation = -1.0;

    for (int i=first_index, count=contour.size(); i!=last_index; i=(i+1)%count)
    {
	    cv::Point current = contour[i];

	    double deviation = static_cast<double>( abs( ((current.x - first.x) * (first.y - last.y)) + ((current.y - first.y) * (last.x - first.x)) ) );

	    if (deviation > max_deviation)
	    {
		    max_pixel_index = i;
		    max_deviation = deviation;
	    }
    }
    max_deviation /= length;

    // 
    // 	// Compute the ratio between the length of the segment and the maximum deviation.
    // 	float ratio = length / std::max( max_deviation, min_deviation );

    // Test the number of pixels of the sub-clusters.
    int half_min_size=min_size/2;
    if ((max_deviation>=min_deviation) && ((max_pixel_index - first_index + 1) >= half_min_size) && ((last_index - max_pixel_index + 1) >= half_min_size))
    {
	    subDivision( straightString, contour, first_index, max_pixel_index, min_deviation, min_size );
	    subDivision( straightString, contour, max_pixel_index, last_index, min_deviation, min_size );
    }
    else
    {
	    // 
	    if ( last_index - first_index > min_size )
	    {
		    std::vector<cv::Point> straightStringCur;
		    for ( int i=first_index; i<last_index; ++i )
		    {
			    straightStringCur.push_back(contour[i]);
		    }
		    straightString.push_back(straightStringCur);
		    //terminalIds.push_back(std::pair<int,int>(first_index, last_index));
	    }
    }
}

void LineFunctions::lineFittingSVD(cv::Point *points, int length, std::vector<double> &parameters, double &maxDev)
{
    // 
    cv::Matx21d h_mean( 0, 0 );
    for( int i = 0; i < length; ++i )
    {
	    h_mean += cv::Matx21d( points[i].x, points[i].y );
    }
    h_mean *= ( 1.0 / length );

    cv::Matx22d h_cov( 0, 0, 0, 0 );
    for( int i = 0; i < length; ++i )
    {
	    cv::Matx21d hi = cv::Matx21d( points[i].x, points[i].y );
	    h_cov += ( hi - h_mean ) * ( hi - h_mean ).t();
    }
    h_cov *=( 1.0 / length );

    // eigenvector
    cv::Matx22d h_cov_evectors;
    cv::Matx21d h_cov_evals;
    cv::eigen( h_cov, h_cov_evals, h_cov_evectors );

    cv::Matx21d normal = h_cov_evectors.row(1).t();

    // 
    if ( abs(normal.val[0]) < abs(normal.val[1]) )  // horizontal
    {
	    parameters[0] = 0;
	    parameters[1] = - normal.val[0] / normal.val[1];
	    parameters[2] = h_mean.val[1] - parameters[1] * h_mean.val[0];
    }
    else  // vertical
    {
	    parameters[0] = 1;
	    parameters[1] = - normal.val[1] / normal.val[0];
	    parameters[2] = h_mean.val[0] - parameters[1] * h_mean.val[1];
    }

    // maximal deviation
    maxDev = 0;
    for( int i = 0; i < length; ++i )
    {
	    cv::Matx21d hi = cv::Matx21d( points[i].x, points[i].y );
	    cv::Matx21d v = hi - h_mean;
	    double dis2 = v.dot(v);
	    double disNormal = v.dot(normal);
	    double disOrtho = sqrt(dis2 - disNormal*disNormal);
	    if ( disOrtho > maxDev )
	    {
		    maxDev = disOrtho;
	    }
    }
}

/************************************************************************/
/*                            PCA Functions                             */
/************************************************************************/

void PCAFunctions::Ori_PCA( pcl::PointCloud<PointT>::Ptr cloud_ptr, int k, std::vector<PCAInfo> &pcaInfos, double &scale, double &magnitd )
{
    double MINVALUE = 1e-7;
    int pointNum = cloud_ptr->size();

    // 1. build kd-tree
    pcl::KdTreeFLANN<PointT>::Ptr kdtree(new pcl::KdTreeFLANN<PointT>());
    kdtree->setInputCloud(cloud_ptr);

    // 2. knn search
    std::vector<std::vector<int>> point_search_ind(pointNum);
    std::vector<std::vector<float>> point_search_sqdis(pointNum);
#pragma omp parallel for
    for (int i=0; i<pointNum; ++i)
    {
	PointT query_pt = cloud_ptr->points[i];
	std::vector<int> point_ind_tmp;
	std::vector<float> point_sqdis_tmp;
	kdtree->nearestKSearch(query_pt, k, point_ind_tmp, point_sqdis_tmp); 
	
	point_search_ind[i] = point_ind_tmp;
	point_search_sqdis[i] = point_sqdis_tmp;
    }

    // 3. PCA normal estimation
    scale = 0.0;
    pcaInfos.resize( pointNum );
#pragma omp parallel for
    for ( int i = 0; i < pointNum; ++i ) 
    {
	// 
	int ki = point_search_ind[i].size();

	double h_mean_x = 0.0, h_mean_y = 0.0, h_mean_z = 0.0;
	for( int j = 0; j < ki; ++j )
	{
	    int idx = point_search_ind[i][j];
	    h_mean_x += cloud_ptr->points[idx].x;
	    h_mean_y += cloud_ptr->points[idx].y;
	    h_mean_z += cloud_ptr->points[idx].z;
	}
	h_mean_x *= 1.0/ki;  h_mean_y *= 1.0/ki; h_mean_z *= 1.0/ki;

	double h_cov_1 = 0.0, h_cov_2 = 0.0, h_cov_3 = 0.0;
	double h_cov_5 = 0.0, h_cov_6 = 0.0;
	double h_cov_9 = 0.0;
	double dx = 0.0, dy = 0.0, dz = 0.0;
	for( int j = 0; j < k; ++j )
	{
	    int idx = point_search_ind[i][j];
	    dx = cloud_ptr->points[idx].x - h_mean_x;
	    dy = cloud_ptr->points[idx].y - h_mean_y;
	    dz = cloud_ptr->points[idx].z - h_mean_z;

	    h_cov_1 += dx*dx; h_cov_2 += dx*dy; h_cov_3 += dx*dz;
	    h_cov_5 += dy*dy; h_cov_6 += dy*dz;
	    h_cov_9 += dz*dz;
	}
	cv::Matx33d h_cov(
		h_cov_1, h_cov_2, h_cov_3, 
		h_cov_2, h_cov_5, h_cov_6, 
		h_cov_3, h_cov_6, h_cov_9);
	h_cov *= 1.0/ki;

	// eigenvector
	cv::Matx33d h_cov_evectors;
	cv::Matx31d h_cov_evals;
	cv::eigen( h_cov, h_cov_evals, h_cov_evectors );

	// 
	pcaInfos[i].idxAll.resize( ki );
	std::vector<double> scalesTemp(ki);
	for ( int j =0; j<ki; ++j )
	{
	    int idx = point_search_ind[i][j];
	    pcaInfos[i].idxAll[j] = idx;
	    
	    dx = cloud_ptr->points[idx].x - cloud_ptr->points[i].x;
	    dy = cloud_ptr->points[idx].y - cloud_ptr->points[i].y;
	    dz = cloud_ptr->points[idx].z - cloud_ptr->points[i].z;
	    scalesTemp[j] = sqrt(dx*dx + dy*dy + dz*dz);
	}

	std::vector<double> sortedScales( scalesTemp.begin(), scalesTemp.end() );
        double medianScale = meadian( sortedScales );
        std::vector<double>().swap( sortedScales );
	
	pcaInfos[i].scale = medianScale;
	scale += medianScale;
	
// 	int idx = point_search_ind[i][3];
// 	dx = cloud_ptr->points[idx].x - cloud_ptr->points[i].x;
// 	dy = cloud_ptr->points[idx].y - cloud_ptr->points[i].y;
// 	dz = cloud_ptr->points[idx].z - cloud_ptr->points[i].z;
// 	double scaleTemp = sqrt(dx*dx + dy*dy + dz*dz);
// 	pcaInfos[i].scale = scaleTemp;
// 	scale += scaleTemp;

	double t = h_cov_evals.row(0).val[0] + h_cov_evals.row(1).val[0] + h_cov_evals.row(2).val[0] + ( rand()%10 + 1 ) * MINVALUE;
	pcaInfos[i].lambda0 = h_cov_evals.row(2).val[0] / t; /// the ration that the smallest eigen value over the sum of all eigens
	pcaInfos[i].normal = h_cov_evectors.row(2).t();

	// outliers removal via MCMD
	pcaInfos[i].idxIn = pcaInfos[i].idxAll;
    }

    scale /= pointNum;
    magnitd = cloud_ptr->points[0].getVector3fMap().norm();
}

void PCAFunctions::PCASingle( pcl::PointCloud<PointT>::Ptr cloud_ptr, PCAInfo &pcaInfo )
{
    int i, j;
    int k = cloud_ptr->size(); // region points size

    // 
    pcaInfo.idxIn.resize( k );
    cv::Matx31d h_mean( 0, 0, 0 );
    for( i = 0; i < k; ++i )
    {
	pcaInfo.idxIn[i] = i;
	h_mean += cv::Matx31d( cloud_ptr->points[i].x, cloud_ptr->points[i].y, cloud_ptr->points[i].z );
    }
    h_mean *= ( 1.0 / k );

    cv::Matx33d h_cov( 0, 0, 0, 0, 0, 0, 0, 0, 0 );
    for( i = 0; i < k; ++i )
    {
	cv::Matx31d hi = cv::Matx31d( cloud_ptr->points[i].x, cloud_ptr->points[i].y, cloud_ptr->points[i].z );
	h_cov += ( hi - h_mean ) * ( hi - h_mean ).t();
    }
    h_cov *=( 1.0 / k );

    // eigenvector
    cv::Matx33d h_cov_evectors;
    cv::Matx31d h_cov_evals;
    cv::eigen( h_cov, h_cov_evals, h_cov_evectors );

    // 
    pcaInfo.idxAll = pcaInfo.idxIn;
    pcaInfo.lambda0 = h_cov_evals.row(2).val[0] / ( h_cov_evals.row(0).val[0] + h_cov_evals.row(1).val[0] + h_cov_evals.row(2).val[0] ); /// normalized smallest eigenvalue
    pcaInfo.normal  = h_cov_evectors.row(2).t();
    pcaInfo.planePt = h_mean; // central point

    // outliers removal via MCMD
    MCMD_OutlierRemoval( cloud_ptr, pcaInfo );
}

void PCAFunctions::MCMD_OutlierRemoval( pcl::PointCloud<PointT>::Ptr cloud_ptr, PCAInfo &pcaInfo )
{
    double a = 1.4826;
    double thRz = 2.5;
    int num = pcaInfo.idxAll.size();

    // ODs
    cv::Matx31d h_mean( 0, 0, 0 );
    for( int j = 0; j < pcaInfo.idxIn.size(); ++j )
    {
	int idx = pcaInfo.idxIn[j];
	h_mean += cv::Matx31d( cloud_ptr->points[idx].x, cloud_ptr->points[idx].y, cloud_ptr->points[idx].z );
    }
    h_mean *= ( 1.0 / pcaInfo.idxIn.size() );

    std::vector<double> ODs( num );
    for( int j = 0; j < num; ++j )
    {
	int idx = pcaInfo.idxAll[j];
	cv::Matx31d pt( cloud_ptr->points[idx].x, cloud_ptr->points[idx].y, cloud_ptr->points[idx].z );
	cv::Matx<double, 1, 1> OD_mat = ( pt - h_mean ).t() * pcaInfo.normal;
	double OD = fabs( OD_mat.val[0] );
	ODs[j] = OD; // distance to the fitting plane
    }

    // calculate the Rz-score for all points using ODs
    std::vector<double> sorted_ODs( ODs.begin(), ODs.end() );
    double median_OD = meadian( sorted_ODs );
    std::vector<double>().swap( sorted_ODs );

    std::vector<double> abs_diff_ODs( num );
    for( int j = 0; j < num; ++j )
    {
	abs_diff_ODs[j] = fabs( ODs[j] - median_OD );
    }
    double MAD_OD = a * meadian( abs_diff_ODs ) + 1e-6;
    std::vector<double>().swap( abs_diff_ODs );

    // get inlier 
    std::vector<int> idxInlier;
    for( int j = 0; j < num; ++j )
    {
	double Rzi = fabs( ODs[j] - median_OD ) / MAD_OD;
	if ( Rzi < thRz ) 
	{
	    int idx = pcaInfo.idxAll[j];
	    idxInlier.push_back( idx );
	}
    }

    // 
    pcaInfo.idxIn = idxInlier;
} 


double PCAFunctions::meadian( std::vector<double> dataset )
{
    std::sort( dataset.begin(), dataset.end(), []( const double& lhs, const double& rhs ){ return lhs < rhs; } );
    if(dataset.size()%2 == 0)
    {
	return dataset[dataset.size()/2];
    }
    else
    {
	return (dataset[dataset.size()/2] + dataset[dataset.size()/2 + 1])/2.0;
    }
}

void PointCloudFunctions::getMinMax3D(const pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices,
		                      Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt)
{
    Eigen::Array4f min_p, max_p;
    min_p.setConstant (FLT_MAX);
    max_p.setConstant (-FLT_MAX);
    
    for (std::vector<int>::const_iterator it = indices.begin (); it != indices.end (); ++it)
    {
	// Get the distance value
	const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&cloud.points[*it]);

	// Create the point structure and get the min/max
	pcl::Array4fMapConst pt = cloud.points[*it].getArray4fMap ();
	min_p = min_p.min(pt);
	max_p = max_p.max(pt);
    }
    
    min_pt = min_p;
    max_pt = max_p;
}
    
void PointCloudFunctions::approximateVoxelGridFilter(pcl::PointCloud<PointT>::Ptr input_, const double& leaf_size_,
			                             pcl::PointCloud<PointT> &output)
{
    ///< initial compute -> pcl_base
    boost::shared_ptr <std::vector<int> > indices_(new std::vector<int>); // indices for the input point cloud
    Eigen::Vector3f inverse_leaf_size_( 1/leaf_size_, 1/leaf_size_, 1/leaf_size_);
    Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;
    
    // Check if input was set
    if (input_->empty()) {
	PCL_ERROR ("[initCompute] Failed with no input point cloud!\n");
	return;
    };

    // If no point indices have been given, construct a set of indices for the entire input point cloud
    if (indices_->empty())
    {
	indices_.reset (new std::vector<int>);
	try
	{
	    indices_->resize (input_->points.size ());
	}
	catch (const std::bad_alloc&)
	{
	    PCL_ERROR ("[initCompute] Failed to allocate %lu indices.\n", input_->points.size ());
	}
	for (size_t i = 0; i < indices_->size (); ++i) { (*indices_)[i] = static_cast<int>(i); }
    }
    ///< End of initial compute
    
    ///< apply filter -> voxel frid
    // Has the input dataset been set already?
    if (!input_)
    {
	PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n");
	output.width = output.height = 0;
	output.points.clear ();
	return;
    }

    // Copy the header (and thus the frame_id) + allocate enough space for points
    output.height       = 1;                    // downsampling breaks the organized structure
    output.is_dense     = true;                 // we filter out invalid points

    Eigen::Vector4f min_p, max_p;
    // Get the minimum and maximum dimensions
    getMinMax3D(*input_, *indices_, min_p, max_p);

    // Check that the leaf size is not too small, given the size of the data
    int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0])+1;
    int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1])+1;
    int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2])+1;

    if ((dx*dy*dz) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()))
    {
	PCL_WARN("[pcl::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.\n");
	output = *input_;
	return;
    }

    // Compute the minimum and maximum bounding box values
    min_b_[0] = static_cast<int> (floor (min_p[0] * inverse_leaf_size_[0]));
    max_b_[0] = static_cast<int> (floor (max_p[0] * inverse_leaf_size_[0]));
    min_b_[1] = static_cast<int> (floor (min_p[1] * inverse_leaf_size_[1]));
    max_b_[1] = static_cast<int> (floor (max_p[1] * inverse_leaf_size_[1]));
    min_b_[2] = static_cast<int> (floor (min_p[2] * inverse_leaf_size_[2]));
    max_b_[2] = static_cast<int> (floor (max_p[2] * inverse_leaf_size_[2]));

    // Compute the number of divisions needed along all axis
    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
    div_b_[3] = 0;

    // Set up the division multiplier
    divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);

    // Storage for mapping leaf and pointcloud indexes
    std::vector<CloudPointIndexIdx> index_vector;
    index_vector.reserve (indices_->size ());
    
    // No distance filtering, process all data
    // First pass: go over all points and insert them into the index_vector vector
    // with calculated idx. Points with the same idx value will contribute to the
    // same point of resulting CloudPoint
    for (std::vector<int>::const_iterator it = indices_->begin (); it != indices_->end (); ++it)
    {
	// Check if the point is invalid
	if (!input_->is_dense)
	    if (!pcl_isfinite (input_->points[*it].x) || !pcl_isfinite (input_->points[*it].y) || !pcl_isfinite (input_->points[*it].z))
		continue;

	int ijk0 = static_cast<int>(floor(input_->points[*it].x * inverse_leaf_size_[0]) - static_cast<float> (min_b_[0]));
	int ijk1 = static_cast<int>(floor(input_->points[*it].y * inverse_leaf_size_[1]) - static_cast<float> (min_b_[1]));
	int ijk2 = static_cast<int>(floor(input_->points[*it].z * inverse_leaf_size_[2]) - static_cast<float> (min_b_[2]));

	// Compute the centroid leaf index
	int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
	index_vector.push_back (CloudPointIndexIdx (static_cast<unsigned int> (idx), *it));
    }

    // Second pass: sort the index_vector vector using value representing target cell as index
    // in effect all points belonging to the same output cell will be next to each other
    std::sort (index_vector.begin (), index_vector.end (), std::less<CloudPointIndexIdx> ());
    
    // Third pass: count output cells
    // we need to skip all the same, adjacenent idx values
    unsigned int total = 0;
    unsigned int index = 0;
    
    // first_and_last_indices_vector[i] represents the index in index_vector of the first point in
    // index_vector belonging to the voxel which corresponds to the i-th output point,
    // and of the first point not belonging to.
    std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
    
    // Worst case size
    first_and_last_indices_vector.reserve (index_vector.size ());
    while (index < index_vector.size ()) 
    {
	unsigned int i = index + 1;
	while (i < index_vector.size () && index_vector[i].idx == index_vector[index].idx) 
	    ++i;
	if (i - index >= 0)
	{
	    ++total;
	    first_and_last_indices_vector.push_back (std::pair<unsigned int, unsigned int> (index, i));
	}
	index = i;
    }

    // Fourth pass: compute centroids, insert them into their final position
    output.points.resize (total);
    
    index = 0;
    for (unsigned int cp = 0; cp < first_and_last_indices_vector.size (); ++cp)
    {
	// calculate centroid - sum values from all input points, that have the same idx value in index_vector array
	unsigned int first_index = first_and_last_indices_vector[cp].first;
	unsigned int last_index = first_and_last_indices_vector[cp].second;

	pcl::CentroidPoint<pcl::PointXYZ> centroid;

	// fill in the accumulator with leaf points
	for (unsigned int li = first_index; li < last_index; ++li) {
	    pcl::PointXYZ pt;
	    pt.x = input_->points[index_vector[li].cloud_point_index].x;
	    pt.y = input_->points[index_vector[li].cloud_point_index].y;
	    pt.z = input_->points[index_vector[li].cloud_point_index].z;
	    centroid.add (pt);  
	}

	pcl::PointXYZ cen;
	centroid.get (cen);
	
	///< search for the closest point to the centroid
	double dis = -1;
	int idx;
	for (unsigned int li = first_index; li < last_index; ++li) {
	    Eigen::Vector3d pt; 
	    pt(0) = input_->points[index_vector[li].cloud_point_index].x;
	    pt(1) = input_->points[index_vector[li].cloud_point_index].y;
	    pt(2) = input_->points[index_vector[li].cloud_point_index].z;
	    
	    double d = (pt - Eigen::Vector3d(cen.x, cen.y, cen.z)).norm();
	    
	    if(dis == -1 || d < dis) {
		dis = d;
		idx = index_vector[li].cloud_point_index;
	    }
	}
	output.points[index] = input_->points[idx];

	++index;
    }
    
    output.width = static_cast<uint32_t> (output.points.size ());
}