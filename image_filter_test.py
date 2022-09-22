# Untitled - By: helei - 周一 5月 23 2022
# 测试图像卷积

import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

clock = time.clock()

edge_01 = (-1,-1,-1,0,0,0,1,1,1)
kernel = [-1, -1, -1,\
          -1, +8, -1,\
          -1, -1, -1]

while(True):
    clock.tick()
    img = sensor.snapshot()
    edge = img.morph(1, kernel, mul=0.1)
    img.replace(edge)
    print(clock.fps())
