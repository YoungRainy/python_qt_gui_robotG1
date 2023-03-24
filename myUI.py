# demo ui
# reference: special relativity, scatterPlotWidget, Flowcharts
import re
import sys
import os

sys.path.append('/home/yys/yys/code/robotG1_2/Mask_RCNN/samples/')

from sample import recong, recong_wawa

from logging import lastResort
from time import daylight
from PyQt5.QtCore import QTime
from numpy.core.numeric import NaN
from numpy.testing._private.utils import temppath
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.widgets.RawImageWidget import RawImageGLWidget
from cal_xyz import reconstruction, image_generate

from PIL import Image
import ctypes
import time
import socket

# my lib
# from MyCameraLib import CameraThread
from myConfigLib import MyConfig
from MyLaserLib import LaserThread
from MyControllerLib import ControllerThread

imgIdx = 0
recvImageGlobal = None # global variable, the received image
recvImageGlobalMutex = QtCore.QMutex() # 

class CameraThread(QtCore.QThread):
    
    _cameraSignal = QtCore.pyqtSignal(str)
    _cameraDataSignal = QtCore.pyqtSignal(list)

    def __init__(self, config:MyConfig):
        super().__init__()

        self._config = config
        self._ip = self._config.cameraIp()
        self._port = self._config.cameraPort()
        self._imgHight = self._config.imageHight()
        self._imgWidth = self._config.imageWidth()

        self._client = None # client socket handler

    def run(self):
        self._client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._cameraSignal.emit('start connecting to camera server!')

        self._client.connect((self._ip, self._port))

        self._cameraSignal.emit('connected with camera Server!')
        self._cameraSignal.emit('start recving images from the camera.')

        self.recvStreamImg()

        # self.imageShowTest()
        pass

    def imageShowTest(self):
        while True:
            for i in range(1, 9):
                global recvImageGlobal

                imgdataTest = Image.open('/home/yys/yys/code/PyKinect2-PyQtGraph-PointClouds-master/img/image_{}.png'.format(i))
                imgdataTest = imgdataTest.convert('RGB')
                imgdataTest = imgdataTest.resize((300, 200))

                recvImageGlobalMutex.lock()

                recvImageGlobal = np.array(imgdataTest)

                recvImageGlobalMutex.unlock()

                self._cameraSignal.emit('oneImageDone')

                time.sleep(0.01)
                # self._cameraSignal.emit('oneImageDone')


    def recvLongData(self, client, recvDataLength):
        buf = b''
        c = 0
        partlen = 4000

        while c < recvDataLength:
            if recvDataLength - c >= partlen:
                new_buf = client.recv(partlen)
            else: 
                leftlen = recvDataLength - c
                new_buf = client.recv(leftlen)

            if not new_buf: return None
            buf += new_buf
            c += len(new_buf)

        return buf

    def recvOneImg(self):
        global recvImageGlobal, imgIdx

        w = self._imgWidth
        h = self._imgHight
        data = self.recvLongData(self._client, w*h*3)

        rowimg = np.frombuffer(data, dtype = 'uint8')

        rowimgreshape = rowimg.reshape((h, w, 3))
        
        rowimgreshape = np.rot90(rowimgreshape, 1)

        # 
        recvImageGlobalMutex.lock()

        # print('flag cam')

        recvImageGlobal = np.array(rowimgreshape)

        # TODO, recognize in real time

        # recvImageGlobal = recong(recvImageGlobal)

        # img = Image.fromarray(recvImageGlobal)
        # img.save('/home/yys/yys/code/img/img_{}.png'.format(imgIdx))
        # imgIdx = imgIdx + 1
        
        # print(recvImageGlobal.shape)
        # self._cameraDataSignal.emit(rowimgreshape.tolist()) # update image on the widget

        recvImageGlobalMutex.unlock()

        self._client.send(b'3')

        self._cameraSignal.emit('oneImageDone')

        time.sleep(0.01)


    def recvStreamImg(self):
        while True:
            flg = self._client.recv(1)
            # print('camera flg: {}'.format(flg))
            try:
                flgvalue = flg.decode('utf-8')
                if flgvalue == '1':
                    self._client.send(b'2')

                    self.recvOneImg()
            except:
                continue


class MyUI():
    def __init__(self) -> None:
        self._app = QtGui.QApplication([])

        self._win = pg.GraphicsWindow(title="A 2d plot window")

        # image from camera
        self._cameraImg = RawImageGLWidget() # image from camera, run mask-RCNN
        # image from the projection of the laser
        self._laserImg = RawImageGLWidget()
        
        # 3D Mapping
        self._glvw3DMap = gl.GLViewWidget()
        g = gl.GLGridItem()
        self._glvw3DMap.addItem(g)

        self._dynamic_pts_scatter_handler = None # dynamic point cloud for 3d mapping
        self._line_handler_dynamic_trajectory = None # show the trajector dynamically
        self._line_handler_predict_trajectory = None # show the predicted trajectory

        # realtime Laser
        self._glvwRealTimeLaser = gl.GLViewWidget()
        g2 = gl.GLGridItem()
        self._glvwRealTimeLaser.addItem(g2)

        self._laser_scatter_handler = None # 3d real time laser points, scatter handler
        self._map_scatter_handler = None # 3d map points, scatter handler

        self._dynamic_laser_pts = None # dynamic point cloud for real time 
        self._image_laser = None # image projected from the laser
        self._image_camera = None # image captured from camera
        self._mapPts = None # 3d points in the map
        self._mapPtsOnce = None # whole map points in one time
        self._mapPtsOneScanListN5 = None # N * 5, scan points in one time
        self._mapPtsOneScanListN3 = None # N * 3, scan points in one time

        # button for start
        self._button_start = QtGui.QPushButton('Start')
        self._button_start.clicked.connect(self.startButtonClicked)
        # button for stop
        self._button_stop = QtGui.QPushButton('Stop')
        self._button_stop.clicked.connect(self.stopButtonClicked)
        # label for showing status
        self._text_label = QtGui.QLabel('label text')
        self._text_label.setStyleSheet("background-color:white")

        # get a layout
        self._layoutgb = QtGui.QGridLayout()
        self._win.setLayout(self._layoutgb)

        # self.initLayout() # default layout
        self.initLayout2()
                
        self._cameraImg.sizeHint = lambda: pg.QtCore.QSize(100, 100)
        self._glvw3DMap.sizeHint = lambda: pg.QtCore.QSize(100, 100)
        self._glvw3DMap.setSizePolicy(self._cameraImg.sizePolicy())

        self._laserImg.sizeHint = lambda: pg.QtCore.QSize(100, 100)
        self._glvwRealTimeLaser.sizeHint = lambda: pg.QtCore.QSize(100, 100)
        self._glvwRealTimeLaser.setSizePolicy(self._laserImg.sizePolicy())

        # timer for update 3d Map
        self._timer_3dMap = QtCore.QTimer()
        self._timer_3dMap.timeout.connect(self.mapUpdate)
        self._timer_3dMap.start(10)

        # timer for update real time laser 
        self._timer_laser = QtCore.QTimer()
        self._timer_laser.timeout.connect(self.laserUpdate)
        self._timer_laser.start(10)  

        # timer for update real time camera image
        self._timer_camera_image = QtCore.QTimer()
        self._timer_camera_image.timeout.connect(self.cameaImageUpdate)
        self._timer_camera_image.start(10)  

        # timer for update image projected from the laser
        self._timer_laser_image = QtCore.QTimer()
        self._timer_laser_image.timeout.connect(self.cameaImageUpdate)
        self._timer_laser_image.start(10)  

        # status
        self._isLaserConnect = False # whether the laser is connected
        self._isCameraConnect = False # whether the camera is connected
        self._isControllerConnect = False # whether the controller is connected
        self._isLaserRecving = False # recving laser data
        self._isLaserRecvDone = False 
        self._laserRecvIdx = 0 # the number of received laser data
        self._isLaserFull = False
        self._lastLaserLen = 0 # the length of laser data in last time
        self._isFirstTimeFull = True # is the first time to capture the whole scan
        self._rotCur = np.eye(4) # current rotation matrix from current to last frame
        self._RotList = [] # the rotation matrix from the current to the start frame
        self._imuData = None # the imu rotation 
        self._imuList = [] # the imu rotation list
        self._curPth = None # path in current scan
        self._fullPth = np.zeros((1, 3)) # full path
        self._isPath = False # if the next data is the path information
        self._isPathFound = False # if the path is found according to ICP

        self._trajecPts = [] #  trajectory in the map

        self._isFirstTimeLaser = True # the first time of laser scan (total 40)

        # config
        config_pth = './config.ini'
        self._config = MyConfig(config_pth)

        # hardware
        # camera
        self._camera = CameraThread(self._config) # the camera
        self._camera._cameraSignal.connect(self.cameraThreadTestCallBack)
        self._camera._cameraDataSignal.connect(self.cameraThreadDataCallBack)

        # laser
        self._laser = LaserThread(self._config) # the laser
        self._laser._laserThreadSignal.connect(self.laserThreadTextCallBack)
        self._laser._laserDataSignal.connect(self.laserThreadDataCallBack)

        # controller
        self._controller = ControllerThread(self._config) # the controller
        self._controller._controllerThreadDataSignal.connect(self.controllerDataCallback)
        

    def controllerDataCallback(self, msg):
        # trajectory points
        trajectoryPts = np.array(msg)

        self._trajecPts.append(trajectoryPts)
        
    def controllerThreadTextCallBack(self, msg):
        print('controller thread: {}'.format(msg))

    def controllerThreadDataCallBack(self, msg):
        print('controller data info: {}'.format(msg))

    def initMap(self):
        pass

    def initLaserWin(self):
        pos = np.array([0, 0, 0])
        size = 0.1
        color = (0.0, 1.0, 0.0, 0.5)

        self._dynamic_pts_scatter_handler = gl.GLScatterPlotItem(pos=pos, size=size, color=color, pxMode=False)
        self._glvwRealTimeLaser.addItem(self._dynamic_pts_scatter_handler)

    def initMapWin(self):
        pos = np.array([0, 0, 0])
        size = 0.1
        color_green = (0.0, 1.0, 0.0, 0.5)
        color_blue = (0.0, 0.0, 1.0, 1)
        color_red = (1.0, 0.0, 0.0, 1)

        self._map_scatter_handler = gl.GLScatterPlotItem(pos=pos, size=size, color=color_green, pxMode=False)
        self._glvw3DMap.addItem(self._map_scatter_handler)

        self._line_handler_dynamic_trajectory = gl.GLLinePlotItem(pos = pos, color=color_blue, width = size*100, antialias = True, mode='line_strip')
        self._glvw3DMap.addItem(self._line_handler_dynamic_trajectory)

        self._line_handler_predict_trajectory = gl.GLLinePlotItem(pos = pos, color=color_red, width = size*100, antialias = True, mode='line_strip')
        self._glvw3DMap.addItem(self._line_handler_predict_trajectory)
    
    def desamplingPts(self, pts):
        # print('desampling')
        ramdomIndex = np.random.randint(0, len(pts)-1, size=20000)
        ptsDesampling = pts[ramdomIndex]
        # print('after desampling: {}'.format(ptsDesampling.shape))
        return ptsDesampling
        

    def laserThreadTextCallBack(self, msg):
        self._text_label.setText(msg)
        if msg == 'recvStart':
            self._isLaserRecving = True

        if msg == 'recvDone':
            self._laserRecvIdx += 1
            # print('recv laser data idex: {}'.format(self._laserRecvIdx))
        if msg == 'fullMap':
            self._isLaserFull =  True
            self._isFirstTimeLaser = True

            if self._isFirstTimeFull:
                print("first time full.")
                # the first full scan, do not need rotation matrix, world coordinate
                # N * 3
                
                onescanPts = self.desamplingPts(self._mapPtsOneScanListN3)

                print("first time one scan pts shape: {}".format(onescanPts.shape))

                self._mapPtsOnce = self.local2world(onescanPts)

                self._mapPts = self._mapPtsOnce

                self._map_scatter_handler.setData(pos = self._mapPts)

                self._isFirstTimeFull = False

            self.projectScanToImage()


        if msg == 'path':
            # the following information is the path information
            self._isPath = True
            
        if msg == 'pathFound':
            print('path found!')
            self._isPathFound = True
            self.clearCurrentScan()
            
        if msg == 'pathnotFound':
            print('path not found!')
            self._isPathFound = False
            self.clearCurrentScan()


    def processLaserThreadFun(self):
        # active according to the status of the laser thread
        return

    def laserThreadDataCallBack(self, datalist):
        # receive laser data from laser thread
        if self._isLaserRecving:

            laserData = np.array(datalist)

            # print('laser data length: {}'.format(len(laserData)))

            if len(laserData) == 9:
                # rotation matrix
                self._rotCur = np.eye(4)
                rot = np.reshape(laserData, (3, 3))
                self._rotCur[:3, :3] = rot
                # print('rotation matrix rot: {}'.format(laserData))
                return

            if len(laserData) == 3:
                # translation
                # t = np.reshape(laserData, (3, 1))
                self._rotCur[:3, 3] = laserData
                self._rotCur[3, 3] = 1
                self._RotList.append(self._rotCur)

                print('rot matrix current : {}'.format(self._rotCur))
                return

            if len(laserData) == 6:
                # imu data
                self._imuData = laserData
                self._imuList.append(laserData)
                print('imu data: {}'.format(self._imuData))
                return

            if self._isPath:

                self._isPath = False
                self.processPathInfo(laserData)
                # TODO, show path in the map
                print('got path from laser.')

                if self._isLaserFull:
                    print('process full scan.')
                    self.processFullScan()
                #     self.projectScanToImage()

                print('laser rotation')
                return

            self.processLaserScan(laserData)
            
    def local2world(self, localPts):
        print('rot list length: {}'.format(len(self._RotList)))

        # rot_w_c = np.eye(4)
        # rot_w_c =self._curPth


        # for i in range(len(self._RotList)):
        #     rot_inv = self._RotList[i]
        #     rot_33 = rot_inv[:3, :3]
        #     rot_33_inv = np.transpose(rot_33)
        #     rot_31 = rot_inv[:3, 3]
        #     rot_31_inv = -1 * np.dot(rot_33_inv, rot_31)
            
        #     rot_inv_new = np.eye(4)

        #     rot_inv_new[:3,:3] = rot_33_inv
        #     rot_inv_new[:3, 3] = rot_31_inv

        #     # rot_w_c = np.dot(rot_w_c, rot_inv_new)
        #     rot_w_c = np.dot(rot_w_c, self._RotList[i])

        # print('rot w c : {}'.format(rot_w_c))
        
        rot33 = self._rotCur[:3, :3]
        t = self._rotCur[:3, 3]

        pts_w = np.dot(rot33, np.transpose(localPts))
        pts_w = np.transpose(pts_w) + np.transpose(t)
        return pts_w
    

    def processPathInfo(self, laserData):
        self._curPth = laserData.reshape((-1, 3))

        print('cur pth: {}'.format(self._curPth))
        print('cur pth len: {}'.format(len(self._curPth)))

        # rot_w_c = np.eye(4)
        # for i in range(len(self._RotList)):
        #     rot_w_c = np.dot(rot_w_c, self._RotList[i])

        # print('rot_w_c: {}'.format(rot_w_c))

        # convert path to world path
        # pth_c = np.array(self._curPth)
        
        # self._curPth = self.local2world(pth_c)

        # self._curPth = pth_w.tolist()

        # print('got path from laser in world: {}'.format(self._curPth))

        # self._fullPth.append(self._curPth)

        # self._fullPth = np.concatenate((self._fullPth, self._curPth), axis=0)

        
        # fullpathArray = np.array(self._fullPth)

        # for i in range(len(self._fullPth)):
        #     # TODO, convert path from current to world
        #     print('path {}: {}'.format(i, self._fullPth[i]))
        
        # plot predicted path

        # fullPthN3 = self._fullPth.tolist()

        self._line_handler_predict_trajectory.setData(pos = self._curPth)

        
    def processLaserScan(self, laserData):
        laserData_N_5 = laserData.reshape((-1, 6))
        # print('laser data shape N5: {}'.format(laserData_N_5.shape))

        laserData_N_3 =reconstruction(laserData_N_5)
        laserData_N_3 = np.array(laserData_N_3)

        # print('cur laser shape N3: {}'.format(laserData_N_3.shape))

            # process current scan
        self.processCurrentScan(laserData_N_3, laserData_N_5)

    def processFullScan(self):
        # all scans for one time is captured, show data in the map window and recognize with RCNN

        print('more full scan.')
        
        # assert len(self._RotList) == len(self._imuList)
        
        # homogeneous matrix , N * 4
        # curHomoN4 = np.ones((len(self._mapPtsOneScanListN3), 4))

        # curHomoN4[:, :3] = self._mapPtsOneScanListN3

        # rot_w_c = np.eye(4)
        # for i in range(len(self._RotList)):
        #     rot_w_c = np.dot(rot_w_c, self._RotList[i])
            
        # curToWorld = np.dot(rot_w_c, np.transpose(curHomoN4)) # 4_4 * 4_N = 4_N

        # curToWorld = np.transpose(curToWorld) # N_4

        # self._mapPtsOnce = curToWorld[:, :3] # the first three columns

        onescanPts = self.desamplingPts(self._mapPtsOneScanListN3)

        # print("one scan pts shape: {}".format(onescanPts.shape))

        self._mapPtsOnce = self.local2world(onescanPts)

        self._mapPts = np.concatenate((self._mapPts, self._mapPtsOnce), axis=0)
        
        # draw map points
        self._map_scatter_handler.setData(pos = self._mapPts)
        print('more scan done.')


    def projectScanToImage(self):
        print('start projection and recognization.')
        img_laser = image_generate(self._mapPtsOneScanListN5)

        img_laser = np.rot90(img_laser, 1)

        # img_save = Image.fromarray(img_laser)
        # img_save.save('/home/yys/yys/code/robotG1_2/wawa.png')

        # time.sleep(0.01)

        # img_read = Image.open('/home/yys/yys/code/robotG1_2/wawa.jpg')

        # img_read = np.array(img_read)

        img_maskrcnn = recong_wawa(img_laser)
        
        # img_maskrcnn = recong_wawa(img_read)


        # img_maskrcnn = recong(img_laser)

        self._laserImg.setImage(np.uint8(img_maskrcnn))


    def processCurrentScan(self, curScanN3, curScanN5):
        if self._isFirstTimeLaser:
            # the frist time for laser scan in one time (1/40)
            self._mapPtsOneScanListN5 = curScanN5
            self._mapPtsOneScanListN3 = curScanN3

            # desampling 
        
            self._isFirstTimeLaser = False
        else:
            # combine map points
            self._mapPtsOneScanListN5 = np.concatenate((self._mapPtsOneScanListN5, curScanN5))
            self._mapPtsOneScanListN3 = np.concatenate((self._mapPtsOneScanListN3, curScanN3))
            # print('map size N3: {}'.format(self._mapPtsOneScanListN3.shape))
            # print('map size N5: {}'.format(self._mapPtsOneScanListN5.shape))
        # show current scan in real time
        self._dynamic_laser_pts = curScanN3
        self._dynamic_pts_scatter_handler.setData(pos = self._dynamic_laser_pts)

    def clearCurrentScan(self):
        # the map point for this time is useless, no path found
        self._isFirstTimeLaser = True # re capturing the scan
        self._isLaserFull = False
        self._mapPtsOneScanListN3 = None
        self._mapPtsOneScanListN5 = None


    def cameraThreadDataCallBack(self, datalist):

        pass


    def cameraThreadTestCallBack(self, msg):
        global recvImageGlobal

        self._text_label.setText(msg)

        if msg == 'oneImageDone':
            recvImageGlobalMutex.lock()

            # print(recvImageGlobal.shape)

            self._cameraImg.setImage(recvImageGlobal)

            recvImageGlobalMutex.unlock()

            # print('image sending done time {}'.format(time.time))


    def initLayout(self):

        self._layoutgb.addWidget(self._glvw3DMap, 0, 0, 3, 3)
        self._layoutgb.addWidget(self._cameraImg, 0, 3, 1, 1)  
        self._layoutgb.addWidget(self._glvwRealTimeLaser, 1, 3, 1, 1)
        self._layoutgb.addWidget(self._laserImg, 2, 3, 1, 1)

        self._layoutgb.addWidget(self._button_start, 3, 0, 1, 1)
        self._layoutgb.addWidget(self._button_stop, 4, 0, 1, 1)
        self._layoutgb.addWidget(self._text_label, 3, 1, 2, 3)

    def initLayout2(self):

        self._layoutgb.addWidget(self._glvw3DMap, 0, 0, 2, 2)
        self._layoutgb.addWidget(self._cameraImg, 0, 2, 1, 1)  
        self._layoutgb.addWidget(self._glvwRealTimeLaser, 1, 2, 1, 1)
        self._layoutgb.addWidget(self._laserImg, 2, 0, 1, 3)

        self._layoutgb.addWidget(self._button_start, 3, 0, 1, 1)
        self._layoutgb.addWidget(self._button_stop, 4, 0, 1, 1)
        self._layoutgb.addWidget(self._text_label, 3, 1, 2, 2)



    def mapUpdate(self):
        pass

    def laserUpdate(self):
        pass

    def cameaImageUpdate(self):
        pass
    
    def cameraLaserUpdate(self):
        pass

    def run(self):
        QtGui.QApplication.instance().exec_()

    
    def startButtonClicked(self):
        self._text_label.setText('start button clicked')

        # connect laser, camera, controller

        if self._config.isCamera() == 'True':
            self._camera.start()
            self._isCameraConnect = True
        
        if self._config.isLaser() == 'True':
            self.initLaserWin()
            self.initMapWin()

            self._laser.start()
            self._isLaserConnect = True

        if self._config.isController() == 'True':

            self._controller.start()
            self._isControllerConnect = True
            


    def stopButtonClicked(self):
        self._text_label.setText('stop button clicked')
