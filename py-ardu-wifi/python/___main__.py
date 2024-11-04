
from dataStream.collectionDataStream import CollectionDataStream
from visualizer import Visualizer

import time


ds = CollectionDataStream('192.168.137.59', 3435)
ds.start('data')


time.sleep(2*60)

ds.terminate()
print("done.")  