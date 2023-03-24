
# this one works

import socket
import numpy as np
import time
from pyqtgraph.Qt import QtCore

from pyqtgraph.widgets.RawImageWidget import RawImageGLWidget

from PIL import Image

# my lib
from myConfigLib import MyConfig

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
        # self._client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self._cameraSignal.emit('start connecting to camera server!')

        # self._client.connect((self._ip, self._port))

        # self._cameraSignal.emit('connected with camera Server!')
        # self._cameraSignal.emit('start recving images from the camera.')

        # self.recvStreamImg()

        # self.imageShowTest()
        pass

    def imageShowTest(self):
        while True:
            print('----')
            print('read image time: {}'.format(time.time()))
            # imgdataTest = Image.open('/home/yys/yys/code/PyKinect2-PyQtGraph-PointClouds-master/img/image_1.png')
            # imgdataTest = imgdataTest.convert('RGB')
            # imgdataTest = imgdataTest.resize((300, 200))

            # imgdataTest = np.array(imgdataTest)

            # print(imgdataTest.shape)
            
            # imgdataTest.show(0)



            # print(imgdataTest)

            # imgdataTest = np.random.uniform((300, 200, 3))
            # imgdataTest = np.array(imgdataTest)

            # print('read done time: {}'.format(time.time()))
            # self._cameraDataSignal.emit(imgdataTest.tolist())
            # print('send image time: {}'.format(time.time()))
            time.sleep(0.01)
            self._cameraSignal.emit('oneImageDone')


    def recvLongData(self, client, recvDataLength):
        buf = b''
        c = 0
        partlen = 6000

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
        w = self._imgWidth
        h = self._imgHight
        data = self.recvLongData(self._client, w*h*3)

        rowimg = np.frombuffer(data, dtype = 'uint8')
        rowimgreshape = rowimg.reshape((h, w, 3))

        rowimgreshape = np.array(rowimgreshape)

        self._cameraDataSignal.emit(rowimgreshape.tolist()) # update image on the widget

        self._client.send(b'3')

        time.sleep(0.01)


    def recvStreamImg(self):
        while True:
            flg = self._client.recv(1)
            flgvalue = flg.decode('utf-8')
            if flgvalue == '1':
                self._client.send(b'2')

                self.recvOneImg()


