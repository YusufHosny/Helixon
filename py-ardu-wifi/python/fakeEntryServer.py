import socket as sock
from dataStore import DataEntry
import struct
import time

with sock.socket(sock.AF_INET, sock.SOCK_STREAM) as s:
    s.bind(('127.0.0.1', 3435))
    s.listen()
    conn, address = s.accept()
    print(f'recvd: {conn.recv(4).decode()}')

    d = DataEntry()
    while True:
        d.ts += 1
        data_binary = struct.pack('<LLddddddddddddBBBBLdd', \
                                    d.ts, d.ts, \
                                    d.accel[0], d.accel[1], d.accel[2], \
                                    d.gyro[0], d.gyro[1], d.gyro[2], \
                                    d.magn[0], d.magn[1], d.magn[2], \
                                    d.rpy[0], d.rpy[1], d.rpy[2], \
                                    d.tempbno, d.tempbno, d.tempbno, d.tempbno, d.ts, \
                                    d.tempbmp, d.pressure)
        conn.sendall(data_binary)
        time.sleep(0.005)