import os,shutil

# def mymovefile(srcfile,dstfile):
#     if not os.path.isfile(srcfile):
#         print "%s not exist!"%(srcfile)
#     else:
#         fpath,fname=os.path.split(dstfile)    #分离文件名和路径
#         if not os.path.exists(fpath):
#             os.makedirs(fpath)                #创建路径
#         shutil.move(srcfile,dstfile)          #移动文件
#         print "move %s -> %s"%( srcfile,dstfile)

def mycopyfile(srcfile,dstfile):
    if not os.path.isfile(srcfile):
        print(srcfile+"not exist!")
    else:
        fpath,fname=os.path.split(dstfile)    #分离文件名和路径
        if not os.path.exists(fpath):
            os.makedirs(fpath)                #创建路径
        shutil.copyfile(srcfile,dstfile)      #复制文件
        print( "copy" + srcfile + "->" + dstfile )

file=open("./model.txt",'r', encoding='utf-8')
result=list()
for line in file.readlines():
    line = line.strip('\n')
    result.append(line)    
# print(result)

number = 1
for line in result:
    line_array=line.split()
    # print(line_array[1])
    # class_ = "Chair"
    # if str(line_array[1]) == class_:
    objname = line_array[0][4:] + ".obj"
    rootpath = '/home/sun/WorkSpace/shrec17_data_jan27/shapenet/models/'
    srcfile = rootpath + objname
    dstfile = '/home/sun/WorkSpace/DealWithSHREC/model_obj/' + objname
    number += 1
    mycopyfile(srcfile,dstfile)