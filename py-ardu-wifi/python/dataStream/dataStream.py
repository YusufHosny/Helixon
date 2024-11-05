import numpy as np
import threading as ts
import socket as sock
import time
import struct
from dataStore import DataEntry, DataManager

class DataStream:
    
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self._thread = None
        self._done = False

    def start(self, type: str, callback = None) -> None:
            self._thread = ts.Thread(target=self.streamThread, name='streamThread')
            self._callback = callback
            self._thread.start()
            
    def terminate(self, ) -> None:
        self._done = True
        if self._thread is not None:
            self._thread.join()
            self._thread = None
    
    def streamThread(self,):
        pass

    def rttTest(self, ):
        with sock.socket(sock.AF_INET, sock.SOCK_STREAM) as s:
            s.connect((self.host, self.port))
            s.send(b'rr')
            print('connected')
            data = s.recv(1)
            s.sendall(data)


