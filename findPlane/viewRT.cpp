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
	PointCloudT::Ptr cloud_A(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_RT(new PointCloudT);  // Transformed point cloud
	PointCloudT::Ptr cloud_B(new PointCloudT);   // Original point cloud

	pcl::console::TicToc time;
	time.tic();
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud_A) == -1)
	{
		PCL_ERROR("Error loading cloud %s.\n");
		return (-1);
	}else{
		std::cout << "\nLoaded file " << argv[1] << " (" << cloud_A->size() << " points) in " << time.toc() << " ms\n" << std::endl;
	}

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *cloud_B) == -1)
	{
		PCL_ERROR("Error loading cloud %s.\n");
		return (-1);
	}else{
		std::cout << "\nLoaded file " << argv[2] << " (" << cloud_B->size() << " points) in " << time.toc() << " ms\n" << std::endl;
	}

	FILE *in_RT = fopen("../Initialize_RT.txt", "r");  //读入文件
	if( in_RT == NULL){
		printf("Open Initialize_RT File %s Error\n","../Initialize_RT.txt");
		return -1; 
	}else{
        printf("Open Initialize_RT File %s Successfully\n","../Initialize_RT.txt");

    }
    // 至此已成功读取两个pcd文件 和 Initialize_RT 旋转矩阵和平移向量文件


	// Defining a rotation matrix and translation vector
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

	// A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	// double theta = 0 / 8; // The angle of rotation in radians
	transformation_matrix(0, 0) = 1.0/50;
	transformation_matrix(1, 1) = 1.0/50;
	transformation_matrix(2, 2) = 1.0/50;

    // // 从文件读取数据到矩阵
   	// for (int i = 0; i < 3; i++) {
	// 	for (int j = 0; j < 3; j++) {
	// 		double n;
	// 		fscanf(in_RT, "%lf", &n);
	// 		// printf("(%d,%d) = %lf\n", i, j, n);
	// 		// array[i][j] = n;
	// 		transformation_matrix(i, j) = n;
	// 	}
	// }
	// for (int i = 4 ,j = 0; j < 3; j++) {
	// 		double n;
	// 		fscanf(in_RT, "%lf", &n);
	// 		// printf("(%d,%d) = %lf\n", i, j, n);
	// 		// array[i][j] = n;
	// 		transformation_matrix(j, 3) = n;
	// }
	// fclose(in_RT);

    
	// Display in terminal the transformation matrix
	std::cout << "Applying this rigid transformation to: cloud_B -> cloud_RT" << std::endl;
	print4x4Matrix(transformation_matrix);

	// // Executing the transformation
	// pcl::transformPointCloud(*cloud_B, *cloud_RT, transformation_matrix);
	cloud_RT = cloud_B;
	
	// Visualization
	pcl::visualization::PCLVisualizer viewer("viewRT Result");


	// The color we will be using
	float bckgr_gray_level = 0.0; // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// Original point cloud is white A原始点云是白的
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_A_color_h(cloud_A, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
	viewer.addPointCloud(cloud_A, cloud_A_color_h, "cloud_A_v1");
    std::string str1(argv[1]);
    str1 +=" is White";
    viewer.addText(str1, 30, 90, 30, 1, 1, 1, "cloud_A");



	// Transformed point cloud is green B RT变换后的点云是绿色的
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_RT_color_h(cloud_RT, 20, 180, 20);
	viewer.addPointCloud(cloud_RT, cloud_RT_color_h, "cloud_RT_v1");
    std::string str2(argv[2]);
    str2 +=" is Green";
    viewer.addText(str2, 30, 30, 30, 0, 1, 0, "cloud_B");


	// Set background color
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);

	// Set camera position and orientation
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1920, 1080); // Visualiser window size
	viewer.addCoordinateSystem(0.5); //显示XYZ指示轴

	// Display the visualiser
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return (0);
}

void print4x4Matrix(const Eigen::Matrix4d &matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}
