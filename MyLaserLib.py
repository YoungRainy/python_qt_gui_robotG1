
# this one works

import re
import struct
import socket
from PIL.Image import NONE
import numpy as np
import time

from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.widgets.RawImageWidget import RawImageGLWidget

import pyvista

# my lib
from myConfigLib import MyConfig


class LaserThread(QtCore.QThread):
    _laserThreadSignal = QtCore.pyqtSignal(str)
    _laserDataSignal = QtCore.pyqtSignal(list)

    def __init__(self, config:MyConfig):

        super().__init__()
        self._config = config
        self._ip = self._config.laserIP()
        self._port = self._config.laserPort()

        self._client = None # socket client handler

        self._mapPts = None # map points
        self._curPts = None # current scanned points
        self._curPtsLen = None # the number of current scan
        self._curPtsInMap = None # convert the scanned points to world coordinate

        # 
        self._stop = False # start or stop


    def run(self):
        
        self._client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._laserThreadSignal.emit('start connecting the laser.')
        self._client.connect((self._ip, self._port))
        self._laserThreadSignal.emit('laser connection established!')

        self.checkConnection()

        idx = 0

        while True:
            t = idx % 40

            self._laserThreadSignal.emit('recvStart')

            laserDataList = self.laserDataToList()

            # print('laser len: {}'.format(len(laserDataList)))
            if laserDataList:
                self._laserDataSignal.emit(laserDataList) 

                self._laserThreadSignal.emit('recvDone')

            # sim test
            # self.simLaserDataTest()

            idx += 1

            time.sleep(0.01)

    def simLaserDataTest(self):

        randPts = np.random.random((10000, 3)) * 10

        self._laserDataSignal.emit(randPts.tolist())

    def recvRotMatrix(self):
        # receive the rotation matrix

        rotMatrixData = self._client.recv(8 * 9)

        fmt = '{}d'.format(9)

        rotMatrix = struct.unpack(fmt, rotMatrixData)

        return rotMatrix
        

    def initLaserData(self):
        # for the first frame, create some data
        laserData = np.array([0, 0, 0])
        # Initialize color and plot the scatter points
        self._laserDataSignal.emit(laserData.tolist())


    def laserDataToList(self):
        # TODO
        laserData = self.recvLaserOnce()
        if laserData:
            return list(laserData)
        else:
            return
        # return list(laserData)


    def checkConnection(self):
        data = self._client.recv(1) # waiting the signal from the laser server
        dataval = data.decode('utf-8')
        if dataval == '9':
            self._client.send(b'8')
            self._laserThreadSignal.emit('laser connection OK!')


    def recvLaserOnce(self):
        self._curPtsLen = self.recvDataLen(self._client)
        # print('recv laser data length: {}'.format(self._curPtsLen))
        if self._curPtsLen == 1:
            laserData = self.recvFullScanDataSignal()
            return laserData
        elif self._curPtsLen == 9:
            laserData = self.recvRotMatrix()
            return laserData
        elif self._curPtsLen == 3:
            laserData = self.recvTranslation()
            return laserData
        elif self._curPtsLen == 6:
            laserData = self.recvIMU()
            return laserData
        elif self._curPtsLen == 999:
            # got the full map signal
            self._laserThreadSignal.emit('fullMap')
            time.sleep(0.001)
            return
            # reConfigLaserMapLength = self.recvDataLen(self._client)
            # laserData = self.recvLaserData(self._client, reConfigLaserMapLength)
        elif self._curPtsLen == 888:
            self._laserThreadSignal.emit('path')
            time.sleep(0.01)
            pathLength = self.recvDataLen(self._client)
            print('recv path length: {}'.format(pathLength))
            
            laserData = self.recvLaserData(self._client, pathLength)

            print('cur path: {}'.format(laserData))
            return laserData   
            
        elif self._curPtsLen == 777:
            # the path is found
            self._laserThreadSignal.emit('pathFound')
            time.sleep(0.001)
            return 
        elif self._curPtsLen == 666:
            self._laserThreadSignal.emit('pathnotFound')
            time.sleep(0.001)
            return 
        else:
            # laser scanning data
            laserData = self.recvLaserData(self._client, self._curPtsLen)
            return laserData

    def recvIMU(self):
        imuData = self._client.recv(8 * 6)

        fmt = '{}d'.format(6)

        imuD = struct.unpack(fmt, imuData)

        return imuD
    

    def recvTranslation(self):
        t = self._client.recv(8 * 3)

        fmt = '{}d'.format(3)

        trans = struct.unpack(fmt, t)

        return trans

    def recvFullScanDataSignal(self):
        temp = self._client.recv(8)

        fmt = '{}d'.format(1)

        fullScanSignal = struct.unpack(fmt, temp)

        return fullScanSignal

    def recvFullScanDataSignal2(self):
        
        pass

    def recvDataLen(self, client):
        dataLen = client.recv(8)
        dataLenVal = struct.unpack('d', dataLen)[0]
        return int(dataLenVal)

    def recvLaserData(self, client, valLen):

        recvData = self.recvLongData(client, 8 * valLen)

        fmt = '{}d'.format(valLen)

        recvVal = struct.unpack(fmt, recvData)

        return recvVal

    def recvLongData(self, client, recvDataLength):
        buf = b''
        c = 0
        partlen = 4096

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



