import threading, queue
import os.path as osp

X = 0
Y = 1
Z = 2


class DataEntry:

    def __init__(self):
        self.ts = 0
        self.accel = [0., 0., 0.]
        self.gyro = [0., 0., 0.]
        self.magn = [0., 0., 0.]
        self.rpy = [0., 0., 0.]
        self.tempbno = 0
        self.tempbmp = 0.
        self.pressure = 0.


class DataManager:
    def __init__(self, path):
        self._path = osp.join(osp.dirname(osp.realpath(__file__)), path)

    def __enter__(self):
        self.to_write = queue.Queue()

        def writer():
            with open(self._path, 'a') as f:
                f.write('timestamp, accelx, accely, accelz, gyrox, gryoy, gyroz, magnx, magny, magnz, roll, pitch, yaw, tempbno, tempbmp, pressure\n')
                for d in iter(self.to_write.get, None):
                    d: DataEntry
                    f.write(f'{d.ts}, ' + \
                        f'{d.accel[X]}, {d.accel[Y]}, {d.accel[Z]}, ' + \
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
        

