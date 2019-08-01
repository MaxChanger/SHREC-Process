#!/bin/bash
cd bad_model_obj
for i in `ls` ;do
    echo ${i}
    # pcl_mesh_sampling ${i} "../model_ply/"${i%.*}".pcd" -n_samples 2048
    # ../mesh_sampling_self ${i} "../model_ply/"${i%.*}".pcd" -n_samples 2325 -no_vis_result
    pcl_pcd2ply "../model_ply/"${i%.*}".pcd" "../model_ply/"${i%.*}".ply"
    rm "../model_ply/"${i%.*}".pcd"
    # rm ${i}
done
    echo "Complete"
