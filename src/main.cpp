#include <stdio.h>
#include <fstream>

#include "LineDetection3D.h"
#include "PlaneDetection.h"
#include "nanoflann.hpp"
#include "utils.h"
#include "Timer.h"


using namespace cv;
using namespace std;
using namespace nanoflann;

void readDataFromTXTFile( std::string filepath, pcl::PointCloud<PointT>::Ptr cloud )
{
    cout<<"Reading data ..."<<endl;

    // 1. read in point data
    std::ifstream ptReader( filepath );
    std::vector<cv::Point3d> lidarPoints;
    double x = 0, y = 0, z = 0, color = 0;
    double nx, ny, nz;
    int a = 0, b = 0, c = 0; 
    int labelIdx = 0;
    int count = 0;
    int countTotal = 0;
    if( ptReader.is_open() )
    {
	while ( !ptReader.eof() ) 
	{
	    //ptReader >> x >> y >> z >> a >> b >> c >> labelIdx;
	    //ptReader >> x >> y >> z >> a >> b >> c >> color;
	    //ptReader >> x >> y >> z >> color >> a >> b >> c;
	    //ptReader >> x >> y >> z >> a >> b >> c ;
	    ptReader >> x >> y >> z;
	    //ptReader >> x >> y >> z >> color;
	    //ptReader >> x >> y >> z >> nx >> ny >> nz;

	    PointT pt;
	    pt.x = x; pt.y = y; pt.z = z;
	    cloud->push_back(pt);
	}
	ptReader.close();
    }

    std::cout << "Total num of points: " << cloud->size() << "\n";
}

void readDataFromPCDFile( std::string filepath, pcl::PointCloud<PointT>::Ptr cloud )
{
    cout<<"Reading data ..."<<endl;

    // 1. read in point data
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(filepath, *pcl_cloud) == -1) {
	PCL_ERROR("Couldn't read pcd file.\n");
	return ;
    }

    pcl::copyPointCloud(*pcl_cloud, *cloud);
    std::cout << "Total num of points: " << cloud->size() << "\n";
}

void writeOutPlanes( string filePath, std::vector<PLANE> &planes, double scale )
{
    // write out bounding polygon result
    string fileEdgePoints = filePath + "planes.txt";
    FILE *fp2 = fopen( fileEdgePoints.c_str(), "w");
    for (int p=0; p<planes.size(); ++p)
    {
	int R = rand()%255;
	int G = rand()%255;
	int B = rand()%255;

	for (int i=0; i<planes[p].lines3d.size(); ++i)
	{
	    for (int j=0; j<planes[p].lines3d[i].size(); ++j)
	    {
		cv::Point3d dev = planes[p].lines3d[i][j][1] - planes[p].lines3d[i][j][0];
		double L = sqrt(dev.x*dev.x + dev.y*dev.y + dev.z*dev.z);
		int k = L/(scale/10);

		double x = planes[p].lines3d[i][j][0].x, y = planes[p].lines3d[i][j][0].y, z = planes[p].lines3d[i][j][0].z;
		double dx = dev.x/k, dy = dev.y/k, dz = dev.z/k;
		for ( int j=0; j<k; ++j)
		{
			x += dx;
			y += dy;
			z += dz;

			fprintf( fp2, "%.6lf   %.6lf   %.6lf    ", x, y, z );
			fprintf( fp2, "%d   %d   %d   %d\n", R, G, B, p );
		}
	    }
	}
    }
    fclose( fp2 );
}

void writeOutLines( string filePath, std::vector<std::vector<cv::Point3d> > &lines, double scale )
{
    // write out bounding polygon result
    string fileEdgePoints = filePath + "lines.txt";
    FILE *fp2 = fopen( fileEdgePoints.c_str(), "w");
    for (int p=0; p<lines.size(); ++p)
    {
	int R = rand()%255;
	int G = rand()%255;
	int B = rand()%255;

	cv::Point3d dev = lines[p][1] - lines[p][0];
	double L = sqrt(dev.x*dev.x + dev.y*dev.y + dev.z*dev.z);
	int k = L/(scale/10);

	double x = lines[p][0].x, y = lines[p][0].y, z = lines[p][0].z;
	double dx = dev.x/k, dy = dev.y/k, dz = dev.z/k;
	for ( int j=0; j<k; ++j)
	{
	    x += dx;
	    y += dy;
	    z += dz;

	    fprintf( fp2, "%.6lf   %.6lf   %.6lf    ", x, y, z );
	    fprintf( fp2, "%d   %d   %d   %d\n", R, G, B, p );
	}
    }
    fclose( fp2 );
}


int main(int argc, char** argv) 
{
    if (argc != 4) {
	std::cout << "usage: ./main -fileData -fileOut -fileType ('txt' or 'pcd')" << std::endl;
	return -1;
    }
    string fileData = string(argv[1]);
    string fileOut  = string(argv[2]);
    string fileType = string(argv[3]);

    // read in data
    pcl::PointCloud<PointT>::Ptr pointData(new pcl::PointCloud<PointT>()); 
    if (fileType == "txt")
	readDataFromTXTFile( fileData, pointData );
    else if (fileType == "pcd") {
	readDataFromPCDFile( fileData, pointData );
    }

    /// random downsize
    int maxSize = 500000;
    if (pointData->size() > maxSize) {
	pcl::RandomSample<PointT> rs;
	rs.setInputCloud(pointData);
	rs.setSample(maxSize);
	
	pcl::PointCloud<PointT>::Ptr pointDataTemp(new pcl::PointCloud<PointT>());
	rs.filter(*pointDataTemp);   
	pointData = pointDataTemp;
	std::cout << "Total num of points after downsampling: " << pointData->size() << "\n";	
    }

    /*PlaneDetection test;
    test.runPlaneExtraction_OBR(pointData, 30.0, 0.05, 30);
    std::printf("test done.\n")*/;
    
    int k = 30;
    LineDetection3D detector;
    std::vector<PLANE> planes;
    std::vector<std::vector<cv::Point3d> > lines;
    std::vector<double> ts;
    detector.run( pointData, k, planes, lines, ts );
    cout<<"lines number: "<<lines.size()<<endl;
    cout<<"planes number: "<<planes.size()<<endl;
    
    writeOutPlanes( fileOut, planes, detector.scale );
    writeOutLines( fileOut, lines, detector.scale );
    pcl::io::savePCDFile(fileOut + "planes.pcd", *detector.planePoints);

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    
    srand (static_cast<unsigned int> (time (0)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < planes.size (); i_segment++)
    {
	colors.push_back (static_cast<unsigned char> (rand () % 256));
	colors.push_back (static_cast<unsigned char> (rand () % 256));
	colors.push_back (static_cast<unsigned char> (rand () % 256));
    }
	
    pcl::PointCloud<PointT>::Ptr input (new pcl::PointCloud<PointT>());
    for(size_t i_segment = 0; i_segment < planes.size (); i_segment++) 
    {
	std::cout << "the " << i_segment << "th plane is displied!" << std::endl;
	input->clear();
	
	*input = planes[i_segment].points;
	for (size_t i_point = 0; i_point < input->points.size (); i_point++)
	{
	    pcl::PointXYZRGB point;
	    point.x = *(input->points[i_point].data);
	    point.y = *(input->points[i_point].data + 1);
	    point.z = *(input->points[i_point].data + 2);
	    point.r = colors[3 * i_segment];
	    point.g = colors[3 * i_segment + 1];
	    point.b = colors[3 * i_segment + 2];
	    colored_cloud->points.push_back (point);
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cen_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	cen_cloud->push_back(detector.planeCentroid->points[i_segment]);
	
	pcl::PointCloud<pcl::Normal>::Ptr norm_cloud(new pcl::PointCloud<pcl::Normal>());
	norm_cloud->push_back(detector.planeNormals->points[i_segment]);
	
	viewer.addPointCloud(colored_cloud, "cloud_plane_" + to_string(i_segment));
	viewer.addPointCloud(cen_cloud, "cloud_centroid_" + to_string(i_segment));
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cen_cloud, norm_cloud, 5, 0.5, "cloud_normals_" + to_string(i_segment));
	
	if (i_segment == planes.size ()-1) {
	    while (!viewer.wasStopped())
		viewer.spinOnce();
	} else {
	    viewer.spinOnce(1000);
	}
    }
    
    

    ///---------- PCL Region Growing ----------///
    // Estimate the normals
//     pcl::NormalEstimation<PointT, pcl::Normal> ne;
//     ne.setInputCloud (pointData);
//     pcl::search::KdTree<PointT>::Ptr tree_n (new pcl::search::KdTree<PointT>());
//     ne.setSearchMethod (tree_n);
//     ne.setRadiusSearch (0.08);
//     
//     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
//     ne.compute (*cloud_normals);
//     pcl::console::print_highlight ("Normals are computed and size is %lu\n", cloud_normals->points.size ());
// 
//     // Region growing
//     pcl::RegionGrowing<PointT, pcl::Normal> rg;
//     rg.setSmoothModeFlag (false); // Depends on the cloud being processed
//     rg.setInputCloud (pointData);
//     rg.setInputNormals (cloud_normals);
// 
//     std::vector <pcl::PointIndices> clusters;
//     pcl::StopWatch watch;
//     rg.extract (clusters);
//     pcl::console::print_highlight ("Extraction time: %f\n", watch.getTimeSeconds());
//     
//     // Writing the resulting cloud into a pcd file
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZRGB>());
//     cloud_segmented = rg.getColoredCloud ();
//     
//     pcl::PCDWriter writer;
//     pcl::console::print_highlight ("Number of segments done is %lu\n", clusters.size ());
//     writer.write<pcl::PointXYZRGB> (fileOut + "segment_result.pcd", *cloud_segmented, false);
// 
//     if (pcl::console::find_switch (argc, argv, "-dump"))
//     {
// 	pcl::console::print_highlight ("Writing clusters to clusters.dat\n");
// 	std::ofstream clusters_file;
// 	clusters_file.open ("clusters.dat");
// 	for (std::size_t i = 0; i < clusters.size (); ++i)
// 	{
// 	clusters_file << i << "#" << clusters[i].indices.size () << ": ";
// 	std::vector<int>::const_iterator pit = clusters[i].indices.begin ();
// 	clusters_file << *pit;
// 	for (; pit != clusters[i].indices.end (); ++pit)
// 	    clusters_file << " " << *pit;
// 	clusters_file << std::endl;
// 	}
// 	clusters_file.close ();
//     }
    ///-------------------------------------------------///

    return 0;
}