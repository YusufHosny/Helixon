import pyautogui as pg

from dataStream.dataStream import DataStream
from visualizer import Visualizer

import time

def swiper(s: str) -> None:
    print(s)
    pg.press('left' if s == 'u' else 'right')

ds = DataStream('192.168.137.203', 1337)
ds.start('swipe', swiper)

while(input('stop?') != 'y'):
    time.sleep(1)

ds.terminate()
print("done.")  