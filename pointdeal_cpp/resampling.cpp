#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/random_sample.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

int main(int argc, char **argv)
{
	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
	// Load bun0.pcd -- should be available with the PCL archive in test
	// pcl::io::loadPCDFile("/home/sun/WorkSpace/DealWithSHREC/pointdeal_cpp/7b53493f7944fcf2b691e708071fb777.pcd", *cloud);
	if (pcl::io::loadPLYFile(argv[1], *cloud)){
		std::cerr << "ERROR: Cannot open file " << std::endl;
		return -1;
	}else{
		std::cout << "Read " << cloud->size() << " Point Cloud File Successfully " << argv[1] << std::endl;
	}
	int clout_number = cloud->size() ;
	if( clout_number < 2048 ){
	
		// Create a KD-Tree
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

		// Init object (second point type is for the normals, even if unused)
		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;

		// Set parameters
		mls.setInputCloud(cloud);
		mls.setPolynomialOrder(2);
		mls.setSearchMethod(tree);
		mls.setSearchRadius(0.015);

		mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
		mls.setUpsamplingRadius(0.02);
		mls.setUpsamplingStepSize(0.015);

		// Reconstruct
		mls.process(*cloud_out);

		pcl::RandomSample<pcl::PointXYZ> rs;
		rs.setInputCloud(cloud_out);
		rs.setSample(2048);			// 设置输出点的数量
		rs.filter(*cloud_out);		//下采样并输出到cloud_out

	}else{
		pcl::RandomSample<pcl::PointXYZ> rs;
		rs.setInputCloud(cloud);
		rs.setSample(2048);			// 设置输出点的数量
		rs.filter(*cloud_out);		//下采样并输出到cloud_out
	}
	
	// std::cout << cloud_out->size() << std::endl;

	
	// Save output
	// pcl::io::savePCDFile ("/home/sun/WorkSpace/DealWithSHREC/pointdeal_cpp/resampling.pcd", mls_points);
	// pcl::io::savePLYFileASCII("/home/sun/WorkSpace/DealWithSHREC/pointdeal_cpp/resampling.ply", *cloud_out);
	pcl::io::savePLYFileASCII( argv[2], *cloud_out);

}