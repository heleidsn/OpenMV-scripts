# Untitled - By: helei - 周三 3月 22 2023
# image difference test

import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 200)

clock = time.clock()

img_last = sensor.snapshot()

# 设置额外的frame buffer
extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.GRAYSCALE)
extra_fb.replace(sensor.snapshot())

while(True):
    clock.tick()
    img = sensor.snapshot()
    img = img.difference(extra_fb)

    extra_fb.replace(sensor.snapshot())
    print(clock.fps())
