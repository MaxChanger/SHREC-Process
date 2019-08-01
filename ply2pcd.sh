#!/bin/bash
cd snn_train_ply
for i in `ls` ;do
    # echo ${i}
    pcl_ply2obj ${i} "../snn_train_ply_normalize/"${i%.*}".obj"
    pcl_obj2pcd "../snn_train_ply_normalize/"${i%.*}".obj" "../snn_train_ply_normalize/"${i%.*}".pcd"
    pcl_pcd2ply "../snn_train_ply_normalize/"${i%.*}".pcd" "../snn_train_ply_normalize/"${i%.*}".ply"

    rm "../snn_train_ply_normalize/"${i%.*}".obj"    
    rm "../snn_train_ply_normalize/"${i%.*}".pcd"

done
    echo "Complete"