#!/bin/bash
cd snn_train_ply_normalize
for i in `ls` ;do
    echo ${i}
    # pcl_mesh_sampling ${i} "../model_ply/"${i%.*}".pcd" -n_samples 2048
    ../normalization "/home/sun/WorkSpace/DealWithSHREC/snn_train_ply_normalize/"${i} "/home/sun/WorkSpace/DealWithSHREC/snn_train_ply_normalize/"${i}

done
    echo "Complete"
