/**
 * 实现 obj / stl CAD模型的多个角度渲染
 * https://blog.csdn.net/rocachilles/article/details/89397212
 * 
 * **/


#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
 
using namespace std;
using namespace pcl;
int main(int argc, char** argv)
{
	// 读取CAD模型
	// vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	// reader->SetFileName("0705.STL");
	vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
	reader->SetFileName(argv[1]);
	// /home/sun/WorkSpace/DealWithSHREC/chair_model/0009.obj
	// reader->SetFileName("/home/sun/WorkSpace/shrec17_data_jan27/shapenet/models/1772711b91514cc4212ff51b27f0221.obj");
	reader->Update();

	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata = reader->GetOutput();
	polydata->GetNumberOfPoints();
 
	//***单视角点云获取
	// 主要是renderViewTesselatedSphere的参数设定
	// 输入
	float resx = 1024;   //显示视点图窗的X大小  分辨率，值多大，采集的点越多
	float resy = resx;  //显示视点图窗的Y大小
	std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > views_xyz;// 各视点点云对应的XYZ信息
 
	//输出
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;// 从目标坐标变换到视点相机坐标
	std::vector<float> entropies;//0-1之间，视点看到模型的百分比
 
	// 输入
	int tesselation_level = 1;	// 表示在角度下的细分数
	float view_angle = 90;		// 虚拟相机的视场
	float radius_sphere = 1;	// radius_sphere半径
	bool use_vertices = false;	// 是否采用tessellated icosahedron 的vertices
 
 
    //PCLVisualizer 显示
	pcl::visualization::PCLVisualizer vis;
	vis.addModelFromPolyData(polydata, "mesh", 0);
	vis.setRepresentationToSurfaceForAllActors();
	vis.renderViewTesselatedSphere(resx, resy, views_xyz, poses, entropies, \
		tesselation_level, view_angle, radius_sphere, use_vertices);//显示个角度点云
 
	//保存
	for (int i = 0; i < views_xyz.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZ> views_cloud;
		pcl::transformPointCloud<pcl::PointXYZ>(views_xyz[i]/*输入点云*/, views_cloud/*输出点云*/, poses[i]/*刚性变换*/);
 
		std::stringstream ss;
		ss << "../render_view/cloud_view_" << i << ".pcd";
		// pcl::io::savePLYFile(ss.str(), views_cloud);
		pcl::io::savePCDFile(ss.str(), views_cloud);
	}
 
	//显示原STL文件
	while ( ! vis.wasStopped())
	{
		vis.spinOnce();
	}
}


// #include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/obj_io.h>
// #include <pcl/PolygonMesh.h>
// //#include <pcl/ros/conversions.h>//formROSMsg所属头文件；
// #include <pcl/point_cloud.h>
// #include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
// //#include <pcl/visualization/pcl_visualizer.h>
 
// using namespace std;
// using namespace pcl;
// int main()
// {
//     pcl::PolygonMesh mesh;
//     pcl::io::loadPolygonFile("/home/sun/WorkSpace/DealWithSHREC/chair_model/0009.obj", mesh);
 
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
//     pcl::io::savePCDFileASCII("result.pcd", *cloud);
 
//     cout << cloud->size() << endl;
 
//     cout << "OK!";
//     cin.get();
//     return 0;
// }


// [in] xres       窗口x方向大小（即分辨率），分辨率越大，采样点云包含点的数目越多
// [in] yres        窗口y方向大小（即分辨率），分辨率越大，采样点云包含点的数目越多  
// [in] cloud       有XYZ信息的点云向量代表各视角下的模型
// [out] poses      从物体坐标系到各视角相机坐标系的位姿转换
// [out] enthropies      在0-1之间，各视角看到模型的比率
// [in] tesselation_level     对于原始二十面体三角形面的分割数，如果设为0，则是原始二十面体，设为1，每个三角形面会被分为4个三角形
// [in] view_angle       相机的视场角FOV，默认为45  
// [in] radius_sphere   半径，默认为1
// [in] use_vertices    设为TRUE，则使用顶点，得到12个视角（tesselation_level =0）或42个视角（tesselation_level =1），设为FALSE，则使用面，得到得到20个视角（tesselation_level =0）或80个视角（tesselation_level =1）

// 根据我尝试的，主要设置xres，yres，tesselation_level，view_angle，use_vertices 这几个参数即可，其他参数有什么trick欢迎大家补充
