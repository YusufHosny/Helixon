
# from dataStream.collectionDataStream import UDPDataStream,TCPDataStream
from visualizer import Visualizer
from dotenv import load_dotenv
from dataStream.realTimeKalman import UDPDataStream
import os
import time


load_dotenv()

ds = UDPDataStream(os.getenv('ip'), int(os.getenv('port')))
# ds = TCPDataStream(os.getenv('ip'), int(os.getenv('port')))
ds.start()


time.sleep(5*60)

ds.terminate()
print("done.")  