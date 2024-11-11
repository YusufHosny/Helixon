import socket as sock
import time
import struct
from dataStore import DataEntry, DataManager
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
        with DataManager('test.csv') as dm:
            with sock.socket(sock.AF_INET, sock.SOCK_STREAM) as s:
                s.settimeout(11)
                s.connect((self.host, self.port))
                s.sendall(b'RDY\n')
                print("connected")
                d = DataEntry()
                while not self._done:
                    tprev = None
                    try:
                        raw_data = s.recv(128)
                        data_entry_struct = struct.unpack('<LLddddddddddddBBBBLdd', raw_data) # manually padding
                        d.ts = data_entry_struct[0]
                        d.accel[X] = data_entry_struct[2] # skip bc of padding see struct definition
                        d.accel[Y] = data_entry_struct[3]
                        d.accel[Z] = data_entry_struct[4]
                        d.gyro[X] = data_entry_struct[5]
                        d.gyro[Y] = data_entry_struct[6]
                        d.gyro[Z] = data_entry_struct[7]
                        d.magn[X] = data_entry_struct[8]
                        d.magn[Y] = data_entry_struct[9]
                        d.magn[Z] = data_entry_struct[10]
                        d.rpy[X] = data_entry_struct[11]
                        d.rpy[Y] = data_entry_struct[12]
                        d.rpy[Z] = data_entry_struct[13]
                        d.tempbno = data_entry_struct[14]
                        d.tempbmp = data_entry_struct[19] # padding skip
                        d.pressure = data_entry_struct[20]              
                        
                        dm.write(d)
                        # if tprev is not None:
                        #     time.sleep((d.ts - tprev)*1e-6 + 10e-3)
                        # tprev = d.ts
                        time.sleep(1./50.)
                    except:
                        pass
