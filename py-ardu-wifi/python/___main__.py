
from dataStream.collectionDataStream import CollectionDataStream
from visualizer import Visualizer
from dotenv import load_dotenv
import os
import time


load_dotenv()

ds = CollectionDataStream(os.getenv('ip'), int(os.getenv('port')))
ds.start()


time.sleep(10)

ds.terminate()
print("done.")  