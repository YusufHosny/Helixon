import threading, queue
import os.path as osp

X = 0
Y = 1
Z = 2


class DataEntry:

    def __init__(self):
        self.ts = 0
        self.linaccel = [0., 0., 0.]
        self.gyro = [0., 0., 0.]
        self.magn = [0., 0., 0.]
        self.rpy = [0., 0., 0.]
        self.tempbno = 0
        self.tempbmp = 0.
        self.pressure = 0.

class WifiDataEntry:

    def __init__(self):
        self.ts = 0
        self.rssiCnt = 0
        self.ssids = []
        self.rssis = []

    def addData(self, ssid: str, rssi: int):
        self.ssids += [ssid]
        self.rssis += [rssi]

class DataManager:
    def __init__(self, path):
        self._path = osp.join(osp.dirname(osp.realpath(__file__)), path)
        with open(self._path, 'a') as f:
                if f.tell() == 0:
                    f.write('timestamp, linaccelx, linaccely, linaccelz, gyrox, gryoy, gyroz, magnx, magny, magnz, roll, pitch, yaw, tempbno, tempbmp, pressure\n')

    def __enter__(self):
        self.to_write = queue.Queue()

        def writer():
            with open(self._path, 'a') as f:
                for d in iter(self.to_write.get, None):
                    d: DataEntry
                    f.write(f'{d.ts}, ' + \
                        f'{d.linaccel[X]}, {d.linaccel[Y]}, {d.linaccel[Z]}, ' + \
                        f'{d.magn[X]}, {d.magn[Y]}, {d.magn[Z]}, ' + \
                        f'{d.gyro[X]}, {d.gyro[Y]}, {d.gyro[Z]}, ' + \
                        f'{d.rpy[X]}, {d.rpy[Y]}, {d.rpy[Z]}, ' + \
                        f'{d.tempbno}, {d.tempbmp}, {d.pressure}\n')
                    
        self.t = threading.Thread(target=writer)
        self.t.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.to_write.put(None)

    
    def write(self, d: DataEntry):
        self.to_write.put(d)

class WifiDataManager:
    def __init__(self, path):
        self._path = osp.join(osp.dirname(osp.realpath(__file__)), path)

    def __enter__(self):
        self.to_write = queue.Queue()

        def writer():
            with open(self._path, 'a') as f:
                for d in iter(self.to_write.get, None):
                    d: WifiDataEntry
                    f.write(f'{d.rssiCnt},' + ''.join([f' {''.join([hex(bssidbyte)[2:] + ':' for bssidbyte in bssid]).rstrip(':')}, {rssi},' \
                                                       for bssid, rssi in zip(d.ssids, d.rssis)]).rstrip(',') + '\n')
                    
        self.t = threading.Thread(target=writer)
        self.t.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.to_write.put(None)

    
    def write(self, d: WifiDataEntry):
        self.to_write.put(d)
        

