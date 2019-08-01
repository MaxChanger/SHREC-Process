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
	PointCloudT::Ptr cloud_0(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_1(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_2(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_3(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_4(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_5(new PointCloudT);  // Original point cloud
	// PointCloudT::Ptr cloud_6(new PointCloudT);   // Original point cloud
	// PointCloudT::Ptr cloud_7(new PointCloudT);  // Original point cloud
	// PointCloudT::Ptr cloud_8(new PointCloudT);   // Original point cloud
	// PointCloudT::Ptr cloud_9(new PointCloudT);   // Original point cloud
	// PointCloudT::Ptr cloud_10(new PointCloudT);   // Original point cloud
	
	std::string render0 = "/home/sun/WorkSpace/DealWithSHREC/findPlane/render_view/cloud_view_9.pcd";
	std::string render1 = "/home/sun/WorkSpace/DealWithSHREC/findPlane/render_view/cloud_view_9.pcd";
	std::string render2 = "/home/sun/WorkSpace/DealWithSHREC/findPlane/render_view/cloud_view_19.pcd";
	std::string render3 = "/home/sun/WorkSpace/DealWithSHREC/findPlane/render_view/cloud_view_29.pcd";
	std::string render4 = "/home/sun/WorkSpace/DealWithSHREC/findPlane/render_view/cloud_view_39.pcd";
	std::string render5 = "/home/sun/WorkSpace/DealWithSHREC/findPlane/render_view/cloud_view_45.pcd";
	// std::string render6 = "/home/sun/WorkSpace/DealWithSHREC/findPlane/render_view/cloud_view_59.pcd";
	// std::string render7 = "/home/sun/WorkSpace/DealWithSHREC/findPlane/render_view/cloud_view_67.pcd";
	// std::string render8 = "/home/sun/WorkSpace/DealWithSHREC/findPlane/render_view/cloud_view_78.pcd";
	
	pcl::io::loadPLYFile<pcl::PointXYZ>(render0, *cloud_0);
	pcl::io::loadPLYFile<pcl::PointXYZ>(render1, *cloud_1);
	pcl::io::loadPLYFile<pcl::PointXYZ>(render2, *cloud_2);
	pcl::io::loadPLYFile<pcl::PointXYZ>(render3, *cloud_3);
	pcl::io::loadPLYFile<pcl::PointXYZ>(render4, *cloud_4);
	pcl::io::loadPLYFile<pcl::PointXYZ>(render5, *cloud_5);
	// pcl::io::loadPLYFile<pcl::PointXYZ>(render6, *cloud_6);
	// pcl::io::loadPLYFile<pcl::PointXYZ>(render7, *cloud_7);
	// pcl::io::loadPLYFile<pcl::PointXYZ>(render8, *cloud_8);

	// Visualization
	pcl::visualization::PCLVisualizer viewer0("view 0");
	pcl::visualization::PCLVisualizer viewer1("view 1");
	pcl::visualization::PCLVisualizer viewer2("view 2");
	pcl::visualization::PCLVisualizer viewer3("view 3");
	pcl::visualization::PCLVisualizer viewer4("view 4");
	pcl::visualization::PCLVisualizer viewer5("view 5");
	// pcl::visualization::PCLVisualizer viewer6("view 6");
	// pcl::visualization::PCLVisualizer viewer7("view 7");
	// pcl::visualization::PCLVisualizer viewer8("view 8");
	// pcl::visualization::PCLVisualizer viewer9("view 9");
	// pcl::visualization::PCLVisualizer viewer10("view 10");

	// The color we will be usingcloud_B
	float bckgr_gray_level = 0.0; // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// 点云是白的
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_8(cloud_0, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_1(cloud_1, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_2(cloud_2, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
			pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_3(cloud_3, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
				pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_4(cloud_4, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
					pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_5(cloud_5, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
	// 					pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_6(cloud_6, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
	// 						pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_7(cloud_7, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_8(cloud_8, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_8(cloud_9, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_8(cloud_10, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);

	viewer0.addPointCloud(cloud_0, cloud_color_0, "cloud_0");
	viewer1.addPointCloud(cloud_1, cloud_color_1, "cloud_1");
	viewer2.addPointCloud(cloud_2, cloud_color_2, "cloud_2");
	viewer3.addPointCloud(cloud_3, cloud_color_3, "cloud_3");
	viewer4.addPointCloud(cloud_4, cloud_color_4, "cloud_4");
	viewer5.addPointCloud(cloud_5, cloud_color_5, "cloud_5");
	// viewer6.addPointCloud(cloud_6, cloud_color_6, "cloud_6");
	// viewer7.addPointCloud(cloud_7, cloud_color_7, "cloud_7");
	// viewer8.addPointCloud(cloud_8, cloud_color_8, "cloud_8");	
	// viewer9.addPointCloud(cloud_9, cloud_color_9, "cloud_9");
	// viewer10.addPointCloud(cloud_10, cloud_color_10, "cloud_10");


	// // Transformed point cloud is green B RT变换后的点云是绿色的
	// pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_B_color_h(cloud_B, 20, 180, 20);
	// viewer.addPointCloud(cloud_B, cloud_B_color_h, "cloud_RT_v1",v2);


	// Set background color
	viewer0.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);
	viewer1.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);
	viewer2.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);
	viewer3.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);
	viewer4.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);
	viewer5.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);
	// viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);
	// viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);

	// Set camera position and orientation
	// viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer0.setSize(1920, 1080); // Visualiser window size
	viewer0.addCoordinateSystem(1); //显示XYZ指示轴

	// Display the visualiser
	while (!viewer0.wasStopped())
	{
		viewer0.spinOnce();
	}
	while (!viewer1.wasStopped())
	{
		viewer1.spinOnce();
	}
	return (0);
}