import socket as sock
from dataStream.dataStream import DataStream

class DataStream(DataStream):
    
    def __init__(self,):
        super.__init__()
        # TODO add swipe callback

    def streamThread(self,):
        with sock.socket(sock.AF_INET, sock.SOCK_STREAM) as s:
            s.connect((self.host, self.port))
            s.sendall(b'swipe?\n')
            print("connected")

            while not self._done:
                # read start bytes
                swipeState = s.recv(1).decode()
                if swipeState not in 'udi': continue

                if swipeState in 'ud':
                    s.sendall(b'ack\n')
                    self._callback(swipeState)
                
