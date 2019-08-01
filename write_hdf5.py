import h5py
import numpy as np
from plyfile import PlyData, PlyElement
import sys, os, csv
import numpy
from itertools import islice  

seg_classes = {'Bag': [0], 'Bed': [1], 'Bin': [2], 'Book': [3], 'Bookshelf': [4], 'Box': [5], 'Chair': [6], 'Cup': [7], 'Desk': [8], 'Display': [9], 'Keyboard': [10], 'Light': [11], 'Machine': [12], 'Oven': [13], 'PCcase': [14], 'Pillow': [15], 'Printer': [16], 'Sofa': [17], 'Storage': [18], 'Table': [19]}
# seg_label_to_cat = {} # {0:Airplane, 1:Airplane, ...49:Table}
# for cat in seg_classes.keys():
#     for label in seg_classes[cat]:
#         seg_label_to_cat[label] = cat


def read_dataset(filename):
    dataset = {}
    with open(filename, 'rt', encoding="utf-8") as fin:
        reader = csv.reader(fin)
        next(reader, None) # 跳过首行
        for row in reader:
            fullid = row[0]
            category = row[1]
            subcategory = row[2]
            dataset[fullid] = (category, subcategory)   # 'snn.098_1606786': ('Bag', 'Backpack')
    # print(dataset)
    # print(len(dataset))
    return dataset

if __name__ == '__main__':

    ############################### 需要修改!!!!!! ###############################
    csv_dict = read_dataset('./test_answer.csv')
    # f = h5py.File("./objecnn20_data_hdf5/ply_data_train_snn.h5", 'w')
    f = h5py.File("./objecnn20_data_hdf5/ply_data_test_snn427_2.h5", 'w')
    snn_number = 427 
    ############################### 需要修改!!!!!! ###############################

    a_data = np.zeros((snn_number, 2048, 3),dtype = np.float32)
    a_label = np.zeros((snn_number, 1), dtype = np.uint8)	
    # a_noraml = np.zeros((snn_number, 2048, 3))	
    i = 0
    line = 0
    for fullid in csv_dict:
        if( fullid[:3] == 'snn'):
            # continue
            filename = fullid[4:] + '.ply'
            plypath = '/home/sun/WorkSpace/DealWithSHREC/snn_train_ply_normalize/' + filename # _normalize
            plydata = PlyData.read( plypath )
            # if len(plydata['vertex']['x']) < 2048:
            #     print(plypath)
            #     print(len(plydata['vertex']['x']))
            for j in range(0, 2048):
                a_data[i, j] = [plydata['vertex']['x'][j], plydata['vertex']['y'][j], plydata['vertex']['z'][j]]
                a_label[i] = seg_classes[csv_dict[fullid][0]]
                # a_noraml[i, j] = [plydata['vertex']['nx'][j], plydata['vertex']['ny'][j], plydata['vertex']['nz'][j]]

            print(">>>> write wss:  ", i ," row", plypath )
            i += 1
        
        elif(fullid[:3] == 'wss'):
            break
            filename = fullid[4:] + '.ply'
            plypath = '/home/sun/WorkSpace/DealWithSHREC/model_ply_normalize/' + filename # _normalize
            plydata = PlyData.read( plypath )
            # if len(plydata['vertex']['x']) < 2048:
            #     print(plypath)
            #     print(len(plydata['vertex']['x']))
            for j in range(0, 2048):
                a_data[i, j] = [plydata['vertex']['x'][j], plydata['vertex']['y'][j], plydata['vertex']['z'][j]]
                a_label[i] = seg_classes[csv_dict[fullid][0]]
                # a_noraml[i, j] = [plydata['vertex']['nx'][j], plydata['vertex']['ny'][j], plydata['vertex']['nz'][j]]

            print(">>>> write wss:  ", i ," row", plypath )
            i += 1
        line += 1
        # print(">>>> done: ", line ," row")
    
    data = f.create_dataset("data", data = a_data)
    label = f.create_dataset("label", data = a_label)