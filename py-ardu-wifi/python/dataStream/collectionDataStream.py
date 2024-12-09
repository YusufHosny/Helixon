import socket as sock
import struct
from dataStore import WifiDataEntry, WifiDataManager, DataEntry, DataManager
from dataStream.dataStream import DataStream
import time

class TCPDataStream(DataStream):

    def streamThread(self,):

        X = 0
        Y = 1
        Z = 2
        while not self._done:
            try:
                with sock.socket(sock.AF_INET, sock.SOCK_STREAM) as s:
                    s.settimeout(5)
                    s.connect((self.host, self.port))
                    print("connected")
                    d = DataEntry()
                    with WifiDataManager('wifi.csv') as wdm:
                        with DataManager('raw.csv') as dm:
                            while not self._done:
                                for _ in range(10):
                                    s.sendall(b'data\n')
                                    for _ in range(10):
                                        raw_data = s.recv(128)
                                        data_entry_struct = struct.unpack('<L4x3d3d3d3dB7xdd', raw_data) # manually padding
                                        d.ts            = data_entry_struct[0]
                                        d.linaccel[X]   = data_entry_struct[1]
                                        d.linaccel[Y]   = data_entry_struct[2]
                                        d.linaccel[Z]   = data_entry_struct[3]
                                        d.gyro[X]       = data_entry_struct[4]
                                        d.gyro[Y]       = data_entry_struct[5]
                                        d.gyro[Z]       = data_entry_struct[6]
                                        d.magn[X]       = data_entry_struct[7]
                                        d.magn[Y]       = data_entry_struct[8]
                                        d.magn[Z]       = data_entry_struct[9]
                                        d.rpy[X]        = data_entry_struct[10]
                                        d.rpy[Y]        = data_entry_struct[11]
                                        d.rpy[Z]        = data_entry_struct[12]
                                        d.tempbno       = data_entry_struct[13]
                                        d.tempbmp       = data_entry_struct[14]
                                        d.pressure      = data_entry_struct[15]    

                                        dm.write(d)
                                        time.sleep(.001)
                                wifid = WifiDataEntry()
                                s.sendall(b'wifi\n')
                                raw_wifi_data = s.recv(256)
                                
                                data_entry_struct = struct.unpack('<Lb'+'6B'*25+'x'+'i'*25, raw_wifi_data) # manually padding
                                wifid.ts = data_entry_struct[0]
                                wifid.rssiCnt = data_entry_struct[1]
                                for i in range(wifid.rssiCnt):
                                    bssid_offset = 2
                                    rssi_offset = bssid_offset+6*25
                                    bssid = data_entry_struct[bssid_offset+6*i:bssid_offset+6*(i+1)]
                                    rssi = data_entry_struct[rssi_offset+i]
                                    wifid.addData(bssid, rssi)
                                
                                wdm.write(wifid)
                                time.sleep(.01)
            except Exception as e:
                print(e)

class UDPDataStream(DataStream):

    def streamThread(self,):
        X = 0
        Y = 1
        Z = 2

        with sock.socket(sock.AF_INET, sock.SOCK_DGRAM) as s:  # UDP socket
            s.settimeout(5)  # Set timeout for receiving data
            print("Ready to send/receive data")
            with WifiDataManager('wifi.csv') as wdm:
                with DataManager('raw.csv') as dm:
                    # Send start command
                    s.sendto(b'strt\n', (self.host, self.port))
                    print("Sent start command to Arduino.")
                    while not self._done:
                        # try:
                            # Receive packet of 257 bytes
                            packet, addr = s.recvfrom(257)
                            packet_type = packet[0:1].decode('utf-8')  # First byte determines type

                            if packet_type == 'w':  # Wi-Fi data
                                wifid = WifiDataEntry()
                                raw_wifi_data = packet[1:]  # Remaining 256 bytes
                                data_entry_struct = struct.unpack('<Lb' + '6B'*25 + 'x' + 'i'*25, raw_wifi_data)
                                wifid.ts = data_entry_struct[0]
                                wifid.rssiCnt = data_entry_struct[1]
                                for i in range(wifid.rssiCnt):
                                    bssid_offset = 2
                                    rssi_offset = bssid_offset + 6*25
                                    bssid = data_entry_struct[bssid_offset + 6*i:bssid_offset + 6*(i+1)]
                                    rssi = data_entry_struct[rssi_offset + i]
                                    wifid.addData(bssid, rssi)
                                wdm.write(wifid)


                            elif packet_type == 'r':  # Raw data
                                raw_data = packet[1:]  # Remaining 256 bytes
                                # Split into two 128-byte packets
                                for i in range(2):
                                    chunk = raw_data[i*128:(i+1)*128]
                                    data_entry_struct = struct.unpack('<L4x3d3d3d3dB7xdd', chunk)
                                    d = DataEntry()
                                    d.ts            = data_entry_struct[0]
                                    d.linaccel[X]   = data_entry_struct[1]
                                    d.linaccel[Y]   = data_entry_struct[2]
                                    d.linaccel[Z]   = data_entry_struct[3]
                                    d.gyro[X]       = data_entry_struct[4]
                                    d.gyro[Y]       = data_entry_struct[5]
                                    d.gyro[Z]       = data_entry_struct[6]
                                    d.magn[X]       = data_entry_struct[7]
                                    d.magn[Y]       = data_entry_struct[8]
                                    d.magn[Z]       = data_entry_struct[9]
                                    d.rpy[X]        = data_entry_struct[10]
                                    d.rpy[Y]        = data_entry_struct[11]
                                    d.rpy[Z]        = data_entry_struct[12]
                                    d.tempbno       = data_entry_struct[13]
                                    d.tempbmp       = data_entry_struct[14]
                                    d.pressure      = data_entry_struct[15]
                                    dm.write(d)

                        # except sock.timeout:
                        #     print("Socket timeout, no data received.")

                    # Send stop command
                    s.sendto(b'stop\n', (self.host, self.port))
                    print("Sent stop command to Arduino.")
