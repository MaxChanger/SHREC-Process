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

void printCoefficients(pcl::ModelCoefficients::Ptr coefficients);
void drawPlane(pcl::ModelCoefficients::Ptr coefficients, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, std::string planeName);
double calculateDistance(pcl::PointXYZ node, pcl::ModelCoefficients::Ptr coefficients);

std::ofstream outfile;

int main(int argc, char **argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>()); //点云对象
	pcl::PointCloud<pcl::PointXYZ>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZ>()); //点云对象

	if (pcl::io::loadPCDFile(argv[1], *cloud)){
		std::cerr << "ERROR: Cannot open file " << std::endl;
		return -1;
	}else{
		cout << "Read Point Cloud File Successfully " << argv[1] << endl;
	}

	std::string outFilePath_(argv[1]);
	std::string outFilePath(outFilePath_.substr(0,outFilePath_.length()-4));
	outFilePath.append("_normals.txt");

	outfile.open(outFilePath);
	if( !outfile  ){
		std::cerr << "ERROR: Cannot open the save file " << std::endl;
		return -1;
	}else{
		
		cout << "The Result Will Write to " << outFilePath << endl;

	}

	std::cout << "Input cloud size :" << cloud->size() << std::endl;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(argv[1])); //显示对象创建
	viewer->addPointCloud(cloud, "data");	//全局点云添加

	std::string cloudName = "cloud__";
	std::string planeName = "plane__";

	/**
	 * 提取三个平面
	 **/
	for (int loop_ = 0; loop_ < 3; loop_++)
	{
		cloudName[cloudName.size()-1] = loop_ + 48;
		planeName[planeName.size()-1] = loop_ + 48;

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); // 创建一个模型参数对象，用于记录结果

		pcl::PointIndices::Ptr inliers(new pcl::PointIndices); //inliers表示误差能容忍的点 记录的是点云的序号

		pcl::SACSegmentation<pcl::PointXYZ> seg; // 创建一个分割器
		seg.setOptimizeCoefficients(true);		 // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
		seg.setModelType(pcl::SACMODEL_PLANE);   // Mandatory-设置目标几何形状
		seg.setMethodType(pcl::SAC_RANSAC);		 // 分割方法：随机采样法
		
		/*** 此阈值用于拟合平面时挑选点 ***/
		// seg.setDistanceThreshold(0.10);
		seg.setDistanceThreshold(0.01);		 	 // 设置误差容忍范围，也就是我说过的阈值
		seg.setInputCloud(cloud);				 // 输入点云
		seg.segment(*inliers, *coefficients);	 // 分割点云，获得平面和法向量 inliers是平面上的点，然后coefficients是法向量

		printCoefficients(coefficients);			//打印平面方程												   
		drawPlane(coefficients, viewer, planeName);	// 绘制平面											   
		pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>); // 平面点获取
		for (int i = 0; i < inliers->indices.size(); ++i)
		{
			clicked_points_3d->points.push_back(cloud->points.at(inliers->indices[i]));
			save_cloud->points.push_back(cloud->points.at(inliers->indices[i]));
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

		/*** 此阈值用于拟合出平面后 从现有的点云中 剔除离平面较近的点 过滤后的点云用于提取下一个平面 ***/
		double thread_distance = 0.05;
		// double thread_distance = 0.90;
		for (int i = 0; i < cloud->size(); ++i)
		{
			double dis = calculateDistance(cloud->points.at(i), coefficients);
			if (dis > thread_distance)
			{
				cloud_filtered->points.push_back(cloud->points.at(i));
			}
		}

		double all_dis = 0.0;
		for (int i = 0; i < clicked_points_3d->size(); ++i)
		{
			double dis = calculateDistance(clicked_points_3d->points.at(i), coefficients);
			all_dis += dis;
		}

		std::cout << "cloud_indices point size :" << inliers->indices.size() << std::endl;
		std::cout << "All indices point distance to Plane "<< loop_ << ": " << all_dis << std::endl;
		std::cout << "All indices point distance to Plane "<< loop_ << ": " << all_dis/inliers->indices.size() << std::endl;


		cloud->clear();
		cloud = cloud_filtered;
		// cloud_filtered->clear();
		// std::cout << "cloud size :" << cloud->size() << std::endl;

		char str[200] = {0};
		sprintf(str,"Plane_%d  A: %4.7lf , B: %4.7lf, C: %4.7lf, D: %4.7lf", loop_, coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]);


		if (loop_ == 0)
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0); // 将平面点标为红色添加
			viewer->addPointCloud(clicked_points_3d, red, cloudName);						
			viewer->addText(str, 30, 90, 15, 125, 125, 125, "planeName_0" );
		}
		else if (loop_ == 1)
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(clicked_points_3d, 0, 255, 0); // 将平面点标为绿色添加
			viewer->addPointCloud(clicked_points_3d, green, cloudName);
			viewer->addText(str, 30, 60, 15, 125, 125, 125, "planeName_1" );

		}
		else if (loop_ == 2)
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(clicked_points_3d, 0, 0, 255); // 将平面点标为蓝色添加
			viewer->addPointCloud(clicked_points_3d, blue, cloudName);
			viewer->addText(str, 30, 30, 15, 125, 125, 125, "planeName_2" );
		}

		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloudName);
	}

	std::string path(argv[1]);
	int f = path.find(".pcd");
	path = path.substr(0,f);
	path += "_filter_cloud.pcd";
	save_cloud->width = 1;
	save_cloud->height = save_cloud->points.size();
	pcl::io::savePCDFileASCII(path,*save_cloud);
	viewer->addCoordinateSystem(1.0); //显示XYZ指示轴



	while (!viewer->wasStopped())  //保持窗体，直到显示窗体退出
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	outfile.close();
	return 0;
}


/**
 * 输出平面方程
 **/
void printCoefficients(pcl::ModelCoefficients::Ptr coefficients)
{
	std::cout << "\nPlane equation Ax + By + Cz + D = 0" << std::endl;
	std::cout << " A：" << coefficients->values[0] << std::endl;
	std::cout << " B：" << coefficients->values[1] << std::endl;
	std::cout << " C：" << coefficients->values[2] << std::endl;
	std::cout << " D：" << coefficients->values[3] << std::endl;
	outfile << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;
}

/**
 * 根据平面方程绘制 一个平面
 * TODO:如何给平面加颜色
 **/
void drawPlane(pcl::ModelCoefficients::Ptr coefficients, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, std::string planeName)
{
	pcl::ModelCoefficients coeffs;
	coeffs.values.push_back(coefficients->values[0]);
	coeffs.values.push_back(coefficients->values[1]);
	coeffs.values.push_back(coefficients->values[2]);
	coeffs.values.push_back(coefficients->values[3]);
	viewer->addPlane(coeffs, planeName);
}

/**
 * 给出点坐标 平面方程 使用公式计算点到平面的距离
 **/
double calculateDistance(pcl::PointXYZ node, pcl::ModelCoefficients::Ptr coefficients)
{
	double A, B, C, D, x0, y0, z0;
	A = coefficients->values[0];
	B = coefficients->values[1];
	C = coefficients->values[2];
	D = coefficients->values[3];
	x0 = node.x;
	y0 = node.y;
	z0 = node.z;

	double a = fabs(A * x0 + B * y0 + C * z0 + D);
	double b = sqrt(A * A + B * B + C * C);
	double dis = a / b;

	return dis;
}
