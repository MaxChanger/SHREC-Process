import h5py
import torch
from torch.utils.data import Dataset
from torch.utils.data import DataLoader
# main_path="/home/sun/WorkSpace/PointDeal/Pointnet_Pointnet2_pytorch/data/indoor3d_sem_seg_hdf5_data/"
# train_txt_path=main_path+"train_hdf5_file_list.txt"
# valid_txt_path=main_path+"val_hdf5_file_list.txt"

# # main_path="/home/sun/WorkSpace/PointDeal/Pointnet_Pointnet2_pytorch/data/modelnet40_ply_hdf5_2048/"
# # train_txt_path=main_path+"train_files.1.txt"
# # valid_txt_path=main_path+"test_files.1.txt"
def get_data(train=True):

    main_path = '/home/sun/WorkSpace/DealWithSHREC/objecnn20_data_hdf5/'
    file_name = 'ply_data_test_snn427_2.h5'
 
    clouds_li = []
    labels_li = []
    h5 = h5py.File(main_path + file_name)
    print(h5.keys())    # <KeysViewHDF5 ['data', 'label']>
    pts = h5["data"].value
    lbl = h5["label"].value
    # for i in range(2048):
    #     for j in range(2048):
    #         print(pts[i][j][:])
    # for i in range(len(lbl)):
        # print(lbl[i])
    print("pts size:", pts.shape)   # (1000, 4096, 9)
    print("lbl size:", lbl.shape)   # (1000, 4096)
    # faceId = h5["faceId"].value
    # normal = h5["normal"].value
    # print("faceId size:", faceId.shape)   # (1000, 4096)
    # print("normal size:", normal.shape)   # (1000, 4096)

    clouds_li.append(torch.Tensor(pts))
    labels_li.append(torch.Tensor(lbl))
    clouds = torch.cat(clouds_li)
    labels = torch.cat(labels_li)
    return clouds,labels.long().squeeze()
 
class PointDataSet(Dataset):
    def __init__(self,train=True):
 
        clouds, labels = get_data(train=train)
 
        self.x_data=clouds
        self.y_data=labels
 
        self.lenth=clouds.size(0)
    def __getitem__(self, index):
        return self.x_data[index],self.y_data[index]
    def __len__(self):
        return self.lenth
 
def get_dataLoader(train=True):
    point_data_set=PointDataSet(train=train)
    data_loader=DataLoader(dataset=point_data_set,batch_size=1,shuffle=train)
    return data_loader


if __name__ == '__main__':
    get_dataLoader(train=True)