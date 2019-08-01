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