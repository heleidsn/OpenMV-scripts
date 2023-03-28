# Untitled - By: helei - 周三 3月 22 2023
# image difference test
# 0328-实现图像差分，最大能处理QVGA分辨率灰度图像 46fps

import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 200)

clock = time.clock()

img_last = sensor.snapshot()

while(True):
    clock.tick()

    img = sensor.snapshot()  # 获取图像
    img_old = img.copy()     # 保留获取的图像
    diff = img.difference(img_last)  # 计算差分

    img_last = img_old       # 替换last

    print(clock.fps())
