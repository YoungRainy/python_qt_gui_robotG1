
import struct
import socket

import numpy as np 
from pyqtgraph.Qt import QtGui, QtCore, rotate

# my lib
from myConfigLib import MyConfig

class ControllerThread(QtCore.QThread):
    _controllerThreadTestSignal = QtCore.pyqtSignal(str)
    _controllerThreadDataSignal = QtCore.pyqtSignal(list)

    def __init__(self, config:MyConfig):
        super().__init__()
        self._config = config
        self._ip = self._config.controllerIp()
        self._port = self._config.controllerPort()

        self._recvData = None # received data from the controller
        self._client = None


    def run(self):
        self._client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self._client.connect((self._ip, self._port))

        recvdata = self._client.recv(2)

        flgvalue = recvdata.decode('utf-8')

        print('recv flg : {}'.format(flgvalue))

        if flgvalue == 'DL':
            while True:
                rotval = self._client.recv(100)
                # print('rotval: {}'.format(rotval))
                rotArray = np.frombuffer(rotval, dtype='uint8')
                rotArray = np.array(rotArray)
                self._controllerThreadDataSignal.emit(rotArray.tolist())
                # print(rotArray)

        




