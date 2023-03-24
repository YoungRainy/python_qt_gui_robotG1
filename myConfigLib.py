import configparser


class MyConfig():
    def __init__(self, f_pth):
        self.pth_ = f_pth
        self.config_ = configparser.ConfigParser()
        self.config_.read(self.pth_)

    def localIp(self):
        return self.config_['RobotConfig']['localIp']

    def cameraIp(self):
        return self.config_['RobotConfig']['cameraIp']

    def imageHight(self):
        return int(self.config_['RobotConfig']['imgHight'])

    def imageWidth(self):
        return int(self.config_['RobotConfig']['imgWidth'])

    def laserIP(self):
        return self.config_['RobotConfig']['laserIp']

    def controllerIp(self):
        return self.config_['RobotConfig']['controllerIp']

    def localPort(self):
        return self.config_['RobotConfig']['localPort']

    def cameraPort(self):
        return int(self.config_['RobotConfig']['cameraPort'])

    def laserPort(self):
        return int(self.config_['RobotConfig']['laserPort'])

    def controllerPort(self):
        return int(self.config_['RobotConfig']['controllerPort'])

    def isCamera(self):
        return self.config_['RobotConfig']['isCamera']

    def isLaser(self):
        return self.config_['RobotConfig']['isLaser']

    def isController(self):
        return self.config_['RobotConfig']['isController']