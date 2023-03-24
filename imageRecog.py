import re
import sys
import os

sys.path.append('/home/yys/yys/code/robotG1_2/Mask_RCNN/samples/')

from sample import recong

import PIL
from PIL import Image

import numpy as np

import time

f_pth = '/home/yys/yys/code/robotG1_2/wawa2.jpg'
img = Image.open(f_pth)
imgarray = np.array(img)

t1 = time.time()

imgrecog = recong(imgarray)

t2 = time.time()

print("time cost : {}".format(t2 - t1))

imgrecog2 = Image.fromarray(imgrecog)

imgrecog2.show()

