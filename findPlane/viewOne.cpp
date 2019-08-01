#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud (new pcl::PointCloud<pcl::PointXYZ>);
 
	// if(pcl::io::loadPCDFile<pcl::PointXYZ>( argv[1], *cloud) == -1){

	// 	std::cout << "Read From PCD failed." << std::endl;
	// } 
	
	if(pcl::io::loadPLYFile<pcl::PointXYZ>( argv[1], *cloud) == -1){
		
		std::cout << "Read From PLY failed." << std::endl;
		return(-1);
	}
 
	//std::cout << cloud->width << std::endl;
	//std::cout << cloud->height;
 
	pcl::visualization::PCLVisualizer viewer("view Many Render");

	// The color we will be usingcloud_B
	float bckgr_gray_level = 0.0; // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// 点云是白的
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloud, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);

	viewer.addPointCloud(cloud,cloud_color,"cloud_1");
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);

	// Set camera position and orientation
	// viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1920, 1080); // Visualiser window size
	viewer.addCoordinateSystem(1); //显示XYZ指示轴

	// Display the visualiser
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	system("pause");
	return (0);
}