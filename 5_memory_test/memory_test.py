# 对openmv的memory进行测试
# Lei He
# 2023-02-15

# 总结： 如果需要使用更大的内存，在进行image copy的时候需要将其copy到frame buffer中
#       对于QVGA图像来说，256k的heap最多只能同时存在3张照片，最后还剩下5k的memory可以用，速度能到9fps，如果用frame_buffer，只能到6-7fps
#       对于QQVGA图像来说，好像还可以，速度能到30fps，如果用frame_buffer，只能到16fps
#       此外，还可以通过设置sensor.set_windowing(roi)来实现图像的剪裁，这个nice

import sensor, image, time, gc, micropython

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 200)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

# 手动设置曝光时间
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, 50000) # 设置曝光时间（us）

sensor.set_windowing((320, 200)) #取中间的640*80区域

print(micropython.mem_info())

print('Used: ' + str(gc.mem_alloc()) + ' Free: ' + str(gc.mem_free()))

gc.enable()

sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.GRAYSCALE) # 创建额外的frame buffer


edge_01_l = (-1,-1,-1,0,0,0,1,1,1)
edge_01_r = (1,1,1,0,0,0,-1,-1,-1)
edge_10_l = (-1,0,1,-1,0,1,-1,0,1)
edge_10_r = (1,0,-1,1,0,-1,1,0,-1)

MUL = 0.15
COPY_TO_FB = False

while(True):
    clock.tick()                    # Update the FPS clock.
    img_curr = sensor.snapshot()         # Take a picture and return the image.

    img_01_l = img_curr.copy(copy_to_fb=COPY_TO_FB).morph(1, edge_01_l, mul=MUL)  # 得到左边边界
    img_01_r = img_curr.copy(copy_to_fb=COPY_TO_FB).morph(1, edge_01_r, mul=MUL)  # 得到右边边界
    img_01 = img_01_l.add(img_01_r)  # 得到垂直边界
    del img_01_l, img_01_r  # 删除图像，得到更多内存
    print('Used: ' + str(gc.mem_alloc()) + ' Free: ' + str(gc.mem_free())) # NOTE:神奇，需要把这一句加上才不会报错，ugly
    gc.collect()

    img_10_l = img_curr.copy(copy_to_fb=COPY_TO_FB).morph(1, edge_10_l, mul=MUL)  # 得到上边界
    img_10_r = img_curr.copy(copy_to_fb=COPY_TO_FB).morph(1, edge_10_r, mul=MUL)  # 得到下边界
    img_10 = img_10_l.add(img_10_r)  # 得到水平边界
    del img_10_l, img_10_r
    # print('2-Used: ' + str(gc.mem_alloc()) + ' Free: ' + str(gc.mem_free()))
    gc.collect()

    img_edge = img_10.add(img_01)  # 相加得到最终边界
    del img_01, img_10
    img_00 = img_curr.copy(copy_to_fb=COPY_TO_FB).mean(1)  # 均值滤波

    img_curr.replace(img_00)
    img_moment = img_00.div(img_edge, invert=True) # edge/img_00  得到moment图像

    del img_edge, img_00
    del img_moment
    gc.collect()

    # print('3-Used: ' + str(gc.mem_alloc()) + ' Free: ' + str(gc.mem_free()))
    # Note: OpenMV Cam runs about half as fast when connected to the IDE. The FPS should increase once disconnected.
    print(clock.fps(), 'Used: ' + str(gc.mem_alloc()) + ' Free: ' + str(gc.mem_free()))
