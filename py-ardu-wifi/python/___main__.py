
from dataStream.collectionDataStream import CollectionDataStream
from visualizer import Visualizer
from dotenv import load_dotenv
import os
import time


load_dotenv()

ds = CollectionDataStream(os.getenv('ip'), int(os.getenv('port')))
ds.start('data')


time.sleep(12*60)

ds.terminate()
print("done.")  