import socket as sock
import time
import struct
from dataStore import WifiDataEntry, WifiDataManager, DataEntry, DataManager
from dataStream.dataStream import DataStream

class CollectionDataStream(DataStream):

    def streamThread(self,):
        # data format
        # struct DataEntry { // size 128bytes: 117 + 11 alignment padding
        #     unsigned long microsT; // 4 bytes + 4 padding
        #     double accelx, accely, accelz; // 8 bytes * 3
        #     double gyrox, gyroy, gyroz; // 8 bytes * 3
        #     double magnx, magny, magnz; // 8 bytes * 3
        #     double roll, pitch, yaw; // 8 bytes * 3
        #     int8_t tempbno;  // 1 byte + 7 padding
        #     double tempbmp; // 8 bytes
        #     double pressure; // 8 bytes 
        # };

        X = 0
        Y = 1
        Z = 2    
        with sock.socket(sock.AF_INET, sock.SOCK_STREAM) as s:
            s.settimeout(11)
            s.connect((self.host, self.port))
            print("connected")
            d = DataEntry()
            while not self._done:
                with WifiDataManager('wifi.csv') as dm:
                    wifid = WifiDataEntry()
                    s.sendall(b'wifi\n')
                    raw_data = s.recv(604)
                    
                    data_entry_struct = struct.unpack('<b'+'20s'*25+'3x'+'i'*25, raw_data) # manually padding
                    wifid.rssiCnt = data_entry_struct[0]
                    for i, ssid, rssi in zip(range(wifid.rssiCnt), data_entry_struct[1:26], data_entry_struct[26:]):
                        wifid.addData(ssid.decode().rstrip('\x00'), rssi)
                    
                    dm.write(wifid)

                with DataManager('raw.csv') as dm:
                    for _ in range(50):
                        s.sendall(b'data\n')
                        raw_data = s.recv(128)
                        data_entry_struct = struct.unpack('<L4xddddddddddddB7xdd', raw_data) # manually padding
                        d.ts = data_entry_struct[0]
                        d.accel[X] = data_entry_struct[1]
                        d.accel[Y] = data_entry_struct[2]
                        d.accel[Z] = data_entry_struct[3]
                        d.gyro[X] = data_entry_struct[4]
                        d.gyro[Y] = data_entry_struct[5]
                        d.gyro[Z] = data_entry_struct[6]
                        d.magn[X] = data_entry_struct[7]
                        d.magn[Y] = data_entry_struct[8]
                        d.magn[Z] = data_entry_struct[9]
                        d.rpy[X] = data_entry_struct[10]
                        d.rpy[Y] = data_entry_struct[11]
                        d.rpy[Z] = data_entry_struct[12]
                        d.tempbno = data_entry_struct[13]
                        d.tempbmp = data_entry_struct[14]
                        d.pressure = data_entry_struct[15]              
                        
                        dm.write(d)