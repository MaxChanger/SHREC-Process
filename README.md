# DealWithSHREC

> 2018/07/31 Jiadai Sun sunjiadai@foxmail.com

The main purpose of this project is to generate a data set in HDF5 format for PointNet as input. 

The network part is implemented at [MaxChanger/Pointnet_Pointnet2_pytorch](https://github.com/MaxChanger/Pointnet_Pointnet2_pytorch)  ([Fork & Modify From Here](https://github.com/yanx27/Pointnet_Pointnet2_pytorch) )

The directory structure looks messy. I will continue to improve it.

- [x] `pointdeal_cpp/***.cpp`

  - normalization.cpp

    Pose Normalization and Location Normalization

  - resampling.cpp

    if the number of PointClode is less than 2048, we should upsampling and downsampling to 2048

  - mesh_sampling.cpp

    Used for desampling the number of Point Cloud to `-n_samples 2048`

    If I use the `pcl_mesh_sampling` in script to deal many `*.ply / *.pcd /` file, need to close the  windows and go on to the next one. I re-compile the file and can use the parameter `-no_vis_result`  to Prohibit window pop-up

- [x] `write_hdf5.py`

  Read the csv file and PointCloud generate by PCL

- [x] `get_hdf5_data.py`

  This model is used to see some information and check the correctness of `*.h5` 

- [x] `copymodel.py` & `copyply.py`

  copy the shapenet model in ObjectNN  and `*.ply`  in scenenn we will use to this folder 

**Unused File**

- [ ] `render_obj2png.py`

  An example use blender to render some 2D image

- [ ] `visualization.py`

   



model.txt中存放了3308个来自shapenet的model,均以wss.***命名

运行 python copymodel.py 将3308个`shrec17_data_jan27/shapenet/models/***.obj` 拷贝至 `model_obj/***.obj`

运行obj2pcd.sh 调用mesh_sampling_self将`model_obj/***.obj`进行渲染, 生成制定数目的点云`model_ply/***.pcd`,调用pcl_pcd2ply将`model_ply/***.pcd`转化为`model_ply/***.ply`

运行ply_normalize.sh 调用normalization程序将`model_ply/***.ply`进行location和dimension的normalization,将`model_ply/***.ply`正则化后的文件保存至`model_ply_normalize/***.ply`

运行 python wrire_hdf5.py 将点云数据和label另存为hdf5文件,为PointNet做输入准备



1. python copyply.py
2. ./ply2pcd.sh
3. ./ply_resampling.sh
4. ./ply_normalize.sh
5. python write_hdf5.py