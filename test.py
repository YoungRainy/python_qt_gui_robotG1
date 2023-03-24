import os
import sys
import random
import math
import numpy as np
import skimage.io
import matplotlib
import matplotlib.pyplot as plt
sys.path.append('/home/yys/yys/code/robotG1_2/Mask_RCNN/samples/')
from sample import recong_wawa
from cal_xyz import image_generate

from PIL import Image

laser_pth = '/home/yys/yys/code/11.txt'

def loadData(f_pth):
    data = np.loadtxt(laser_pth, dtype='float')
    return data

laserData = loadData(laser_pth)

laserImg = image_generate(laserData)
# img = Image.fromarray(np.uint8(laserImg))
# img.show()
image=skimage.io.imread(os.path.join('/home/yys/Desktop/3.png'))
laserImg2 =recong_wawa(image)

img = Image.fromarray(np.uint8(laserImg2))
img.show()

pass