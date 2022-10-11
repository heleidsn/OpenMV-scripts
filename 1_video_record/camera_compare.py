# Untitled - By: helei - 周二 10月 11 2022

import sensor, image, time
from ulab import numpy as np

sensor.reset()
# sensor.set_pixformat(sensor.RGB565)
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.VGA)
sensor.set_vflip(True)
sensor.skip_frames(time = 2000)

clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()
    print(clock.fps())
