#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h> // TicToc
#include <fstream>
#include <regex>
#include <vector>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

void print4x4Matrix(const Eigen::Matrix4d &matrix);


int main(int argc, char **argv)
{
	// The point clouds we will be using
	PointCloudT::Ptr cloud_1(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_2(new PointCloudT);   // Original point cloud
	PointCloudT::Ptr cloud_3(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_4(new PointCloudT);   // Original point cloud
	PointCloudT::Ptr cloud_5(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_6(new PointCloudT);   // Original point cloud
	PointCloudT::Ptr cloud_7(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_8(new PointCloudT);   // Original point cloud
	
	std::string render1 = "/home/sun/WorkSpace/DealWithSHREC/findPlane/render_view/cloud_view_9.pcd";
	std::string render2 = "/home/sun/WorkSpace/DealWithSHREC/findPlane/render_view/cloud_view_19.pcd";
	std::string render3 = "/home/sun/WorkSpace/DealWithSHREC/findPlane/render_view/cloud_view_29.pcd";
	std::string render4 = "/home/sun/WorkSpace/DealWithSHREC/findPlane/render_view/cloud_view_39.pcd";
	std::string render5 = "/home/sun/WorkSpace/DealWithSHREC/findPlane/render_view/cloud_view_45.pcd";
	std::string render6 = "/home/sun/WorkSpace/DealWithSHREC/findPlane/render_view/cloud_view_59.pcd";
	std::string render7 = "/home/sun/WorkSpace/DealWithSHREC/findPlane/render_view/cloud_view_67.pcd";
	std::string render8 = "/home/sun/WorkSpace/DealWithSHREC/findPlane/render_view/cloud_view_78.pcd";

	pcl::io::loadPCDFile<pcl::PointXYZ>(render1, *cloud_1);
	pcl::io::loadPCDFile<pcl::PointXYZ>(render2, *cloud_2);
	pcl::io::loadPCDFile<pcl::PointXYZ>(render3, *cloud_3);
	pcl::io::loadPCDFile<pcl::PointXYZ>(render4, *cloud_4);
	pcl::io::loadPCDFile<pcl::PointXYZ>(render5, *cloud_5);
	pcl::io::loadPCDFile<pcl::PointXYZ>(render6, *cloud_6);
	pcl::io::loadPCDFile<pcl::PointXYZ>(render7, *cloud_7);
	pcl::io::loadPCDFile<pcl::PointXYZ>(render8, *cloud_8);

	// pcl::console::TicToc time;
	// time.tic();
	// if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud_A) == -1)
	// {
	// 	PCL_ERROR("Error loading cloud %s.\n");
	// 	return (-1);
	// }else{
	// 	std::cout << "\nLoaded file " << argv[1] << " (" << cloud_A->size() << " points) in " << time.toc() << " ms\n" << std::endl;
	// }

	// if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *cloud_B) == -1)
	// {
	// 	PCL_ERROR("Error loading cloud %s.\n");
	// 	return (-1);
	// }else{
	// 	std::cout << "\nLoaded file " << argv[2] << " (" << cloud_B->size() << " points) in " << time.toc() << " ms\n" << std::endl;
	// }
	
	// Visualization
	pcl::visualization::PCLVisualizer viewer("view Many Render");
	int v1(0), v2(1), v3(2), v4(3),  v5(4), v6(5),  v7(6), v8(7);  
	
	viewer.createViewPort(0.00, 0.0, 0.25, 0.5, v1);
	viewer.createViewPort(0.25, 0.0, 0.50, 0.5, v2);
	viewer.createViewPort(0.50, 0.0, 0.75, 0.5, v3);
	viewer.createViewPort(0.75, 0.0, 1.00, 0.5, v4);
	viewer.createViewPort(0.00, 0.5, 0.25, 1.0, v5);
	viewer.createViewPort(0.25, 0.5, 0.50, 1.0, v6);
	viewer.createViewPort(0.50, 0.5, 0.75, 1.0, v7);
	viewer.createViewPort(0.75, 0.5, 1.00, 1.0, v8);

	// The color we will be usingcloud_B
	float bckgr_gray_level = 0.0; // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// 点云是白的
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_1(cloud_1, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_2(cloud_2, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
			pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_3(cloud_3, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
				pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_4(cloud_4, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
					pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_5(cloud_5, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
						pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_6(cloud_6, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
							pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_7(cloud_7, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_8(cloud_8, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);

	viewer.addPointCloud(cloud_1, cloud_color_1, "cloud_1",v1);
	viewer.addPointCloud(cloud_2, cloud_color_2, "cloud_2",v2);
	viewer.addPointCloud(cloud_3, cloud_color_3, "cloud_3",v3);
	viewer.addPointCloud(cloud_4, cloud_color_4, "cloud_4",v4);
	viewer.addPointCloud(cloud_5, cloud_color_5, "cloud_5",v5);
	viewer.addPointCloud(cloud_6, cloud_color_6, "cloud_6",v6);
	viewer.addPointCloud(cloud_7, cloud_color_7, "cloud_7",v7);
	viewer.addPointCloud(cloud_8, cloud_color_8, "cloud_8",v8);


	// // Transformed point cloud is green B RT变换后的点云是绿色的
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_B_color_h(cloud_B, 20, 180, 20);
	// viewer.addPointCloud(cloud_B, cloud_B_color_h, "cloud_RT_v1",v2);


	// Set background color
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);

	// Set camera position and orientation
	// viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1920, 1080); // Visualiser window size
	viewer.addCoordinateSystem(0.5); //显示XYZ指示轴

	// Display the visualiser
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return (0);
}