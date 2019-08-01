/**
 * 作者：孙家岱
 * 时间：2019年5月21日
 * 实现功能：提取三个平面的平面方程
 * 
 * 运行方式：./cloud_findPlane ../A.pcd
 * 运行结果: 弹出一个窗口，可视化点云的三个平面提取
 * 			在pcd文件同目录下生成两个文件：A_filter_cloud.pcd  A_normals.txt
 * 			filter_cloud为剔除外点后剩余的点云数据，并以ASCII码形式保存，可以打开看
 *			normals.txt为三个平面的平面方程，按RGB三个平面颜色的顺序保存
 * 
 * * */
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

double calculateDistance(pcl::PointXYZ node);

int main(int argc, char **argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>()); //点云对象

	if (pcl::io::loadPLYFile(argv[1], *cloud)){
		std::cerr << "ERROR: Cannot open file " << std::endl;
		return -1;
	}else{
		cout << "Read Point Cloud File Successfully " << argv[1] << endl;
	}

	std::cout << "Input cloud size :" << cloud->size() << std::endl;


	std::string cloudName = "cloud__";
	std::string planeName = "plane__";

	double center_x = 0;
	double center_y = 0;
	double center_z = 0;
	for (int i = 0; i < cloud->size(); ++i)
	{
		center_x += cloud->points.at(i).x;
		center_y += cloud->points.at(i).y;
		center_z += cloud->points.at(i).z;
	}
	center_x = center_x / cloud->size();
	center_y = center_y / cloud->size();
	center_z = center_z / cloud->size();

	double max_radius = -20;
	for (int i = 0; i < cloud->size(); ++i)
	{
		cloud->points.at(i).x -= center_x ;
		cloud->points.at(i).y -= center_y ;
		cloud->points.at(i).z -= center_z ;
		double dis = calculateDistance(cloud->points.at(i));
		if (dis > max_radius)
		{
			max_radius = dis;
		}
	}

	if( max_radius <= 0){
		cout << " max_radius <= 0 " << endl;
		return -1;
	}

	for (int i = 0; i < cloud->size(); ++i)
	{
		cloud->points.at(i).x = cloud->points.at(i).x / max_radius ;
		cloud->points.at(i).y = cloud->points.at(i).y / max_radius ;
		cloud->points.at(i).z = cloud->points.at(i).z / max_radius ;
	}


	std::string outFilePath(argv[2]);

	pcl::io::savePLYFileASCII(outFilePath,*cloud);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 0, 0); // 将平面点标为红色添加
	
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(argv[1])); //显

	// viewer->addPointCloud(cloud, red, "cloudName");		
	// viewer->addCoordinateSystem(1); //显示XYZ指示轴

	// while (!viewer->wasStopped())  //保持窗体，直到显示窗体退出
	// {
	// 	viewer->spinOnce(100);
	// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	// }

	return 0;
}



/**
 * 给出点坐标 计算到原点的距离
 **/
double calculateDistance(pcl::PointXYZ node)
{
	double x0, y0, z0;

	x0 = node.x;
	y0 = node.y;
	z0 = node.z;

	double radius = sqrt(x0 * x0 + y0 * y0 + z0 * z0);

	return radius;
}
