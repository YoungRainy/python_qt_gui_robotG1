from sys import implementation
import scipy.io as scio
import numpy as np



def loadMat():
    f_pth = '/home/yys/yys/code/20210625OK/Modeling_data/1-origin_data.mat'
    data = scio.loadmat(f_pth)
    data = data['XYZ_raw1']
    data = np.reshape(data, )
    return data


