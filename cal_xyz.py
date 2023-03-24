"""
Angle_data  ===>  reconstruction && use the rt of icp  to refine the pointcloud
Author: Eric Geng
Date: May 2021
"""
import numpy as np
import cv2
import matplotlib.pyplot as plt
import scipy
from scipy import io
from PIL import Image, ImageEnhance
import math

def load_txt(file_name):
    scan=np.loadtxt(file_name,dtype=np.float32)
    #scan=scan.reshape(-1,7)
    return scan

def rotx(t):
    c=np.cos(t)
    s=np.sin(t)
    return np.array([[1,0,0],
                     [0,c,-s],
                     [0,s,c]])
def roty(t):
    c=np.cos(t)
    s=np.sin(t)
    return np.array([[c,0,s],
                     [0,1,0],
                     [-s,0,c]])

def rotz(t):
    c=np.cos(t)
    s=np.sin(t)
    return np.array([[c,-s,0],
                     [s,c,0],
                     [0,0,1]])

def xyz_refinement(xyz_local,imu,rt,tt): 
    #input: xyz_local       n*3
    #       imu             6*1
    #       rt  r_last_time 3*3
    #       tt  t_last_time 3*1
    #output  n*3  xyz_refinement point cloud xyz
    #remember follow the input size rules above
    rzy=np.dpt(rotz(imu(0)),roty(imu(1)))
    rzyx=np.dot(rzy,rotx(imu(2)))
    new_r=np.dot(rzyx,rt)
    #RABB=RABB1*TR_lastTime;%全局姿态
    #TT=(TR_lastTime*T_xyz')+TT_lastTime;
    new_t=np.dot(rt,tt)+np.array([[imu(3)],[imu(4)],[imu(5)]])
    #XYZ=(RABB*XYZ_raw'+(T_xyz)')';
    return   np.transpose(np.dot(new_r,np.transpose(xyz_local))+new_t)

def reconstruction(data):
    # data=data[:,2:7]
    T_X = 50.0
    T_Y = 0.0
    T_Z = 60.0
    # pitch=0.1
    pitch=1
    roll=-5
    stepFlag=21
    Z_angle= data[:,0].reshape(-1,1)
    range=data[:,1].reshape(-1,1)
    Y_angle=data[:,2].reshape(-1,1)
    pitch_y=data[:,5].reshape(-1,1)
    #update cross method,use np.multiply() to replace np.dot()
    x = np.multiply(np.multiply(range,np.cos(Y_angle / 180 * np.pi)),np.cos(Z_angle / 180 * np.pi))
    #x = np.dot(np.dot(range,np.cos(Y_angle / 180 * np.pi)),np.cos(Z_angle / 180 * np.pi))
    y = np.multiply(np.multiply(range,np.cos(Y_angle / 180 * np.pi)),np.sin(Z_angle / 180 * np.pi))
    #y = np.dot(np.dot(range,np.cos(Y_angle / 180 * np.pi)),np.sin(Z_angle / 180 * np.pi))
    z = np.multiply(range,np.sin(Y_angle / 180 * np.pi))
    #z = np.dot(range,np.sin(Y_angle / 180 * np.pi))
    xx = np.multiply((x + T_X),np.cos(pitch*pitch_y /180*np.pi)) + np.multiply(np.sin(pitch * pitch_y/180 * np.pi),(z + T_Z))
    #xx = np.dot((x + T_X),np.cos(pitch*pitch_y /180*np.pi)) + np.dot(np.sin(pitch * pitch_y/180 * np.pi),(z + T_Z))
    yy = -(y + T_Y)
    zz = -np.multiply((x + T_X),np.sin(pitch * pitch_y / 180 * np.pi)) + np.multiply(np.cos(pitch * pitch_y / 180 * np.pi),(z + T_Z))
    #zz = -np.dot((x + T_X),np.sin(pitch * pitch_y / 180 * np.pi)) + np.dot(np.cos(pitch * pitch_y / 180 * np.pi),(z + T_Z))
    xxx = xx
    yyy = yy
    zzz = zz
    index=np.where(data[:,4] >= stepFlag)
    
    yyy[index] = np.multiply(yy[index],np.cos(roll /180*np.pi)) - np.multiply(np.sin(roll /180 * np.pi),zz[index])
    zzz[index] = np.multiply(yy[index],np.sin(roll /180*np.pi)) + np.multiply(np.cos(roll /180 * np.pi),zz[index])
    
    xyz_data=np.concatenate((-yyy,xxx,zzz),axis=1)


    return  xyz_data

def image_generate(data):
    # data=data[:,2:7]
    index=np.where(data[:,4]>20)
    data=np.delete(data,index,axis=0)
    index=np.where(data[:,0]<90)
    data[index,0]=data[index,0]+360
    data[:,2]=np.round((data[:,2]-0.1*(data[:,4]-1))*10)#出图的时候需要将点头角度加上
    data[:,2]=data[:,2]-min(data[:,2])+1
    max_depth=20000
    data[:,1]=data[:,2]/max_depth*255
    data[:,0]=np.round(data[:,0]*10)
    min_au=min(data[:,0])
    data[:,0]=data[:,0]-min_au+1
    index=np.where(data[:,0]<1800)
    data=np.delete(data,index,axis=0)
    data[:,0]=data[:,0]-min(data[:,0])+1
    A=int(max(data[:,2])-min(data[:,2])+1)
    lf=A
    B=int(max(data[:,0])-min(data[:,0])+1)
    rf=B
    mm=255*np.ones((int(A),int(B)))
    a=len(data[:,0])
    for i in range(a):
        A=int(data[i,0]-1)
        B=int(data[i,2]-1)
        mm[B,A]=data[i,3]
    for k in range(0,lf-1):
        for kk in range(1,rf-1):
            if mm[k,kk]==255 and mm[k,kk-1]!=255 and mm[k,kk+1]!=255:
                mm[k,kk]=(mm[k,kk-1]+mm[k,kk+1])/2
    for k in range(0,rf-1):
        for kk in range(1,(lf-1)):
            if mm[kk,k]==255 and mm[kk-1,k]!=255 and mm[kk+1,k]!=255:
                mm[kk,k]=(mm[kk,k-1]+mm[kk+1,k])/2

    data=np.array(mm,dtype='uint8')
    data=np.rot90(data,2)
    data,_=histeq(data)
    #print(data0)
    image_mat=np.ones((lf,rf,3))
    image_mat[:,:,0]=data
    image_mat[:,:,1]=data
    image_mat[:,:,2]=data
    #image_mat2=Image.fromarray(data*0.6,"L")
    #image_mat2=ImageEnhance.Contrast(image_mat2)
    #image_mat2.enhance(2).show()
    return image_mat


def histeq(im,nbr_bins=256):
    '''对灰度图进行直方图均衡化'''
    #计算直方图
    imhist,bins = np.histogram(im.flatten(),nbr_bins,normed=True)
    cdf = imhist.cumsum()   #累积分布函数 cumulative distribution function
    cdf = 255*cdf/cdf[-1]   #归一化(灰度变换函数),使用累积分布函数的线性插值，计算新的像素值
    im2 = np.interp(im.flatten(),bins[:-1],cdf)
    return im2.reshape(im.shape),cdf
