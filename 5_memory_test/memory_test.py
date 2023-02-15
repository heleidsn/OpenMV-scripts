# 对openmv的memory进行测试
# Lei He
# 2023-02-15

# 总结： 如果需要使用更大的内存，在进行image copy的时候需要将其copy到frame buffer中
#       对于QVGA图像来说，256k的heap最多只能同时存在3张照片，最后还剩下5k的memory可以用，速度能到9fps，如果用frame_buffer，只能到6-7fps
#       对于QQVGA图像来说，好像还可以，速度能到30fps，如果用frame_buffer，只能到16fps
#       此外，还可以通过设置sensor.set_windowing(roi)来实现图像的剪裁，这个nice

import sensor, image, time, gc, micropython


class LGMD():
    def __init__(self, moment_mode):
        self.init_ok = False
        self.img_g_curr = None  # 当前灰度图像输入
        self.img_m_curr = None  # 当前moment图像
        self.img_m_prev = None  # 上一帧moment图像

        self.p_layer = None
        self.p_prev = None
        self.i_layer = None
        self.s_layer = None

        self.wi = (0.125, 0.25, 0.125, 0.25, 0, 0.25, 0.125, 0.25, 0.125)
        self.w_1 = (1,1,1,1,1,1,1,1,1)
        self.Ki = 0.35

        self.lgmd_out = 0

        self.moment_mode = moment_mode
        self.moment_min = 25
        self.moment_max = 50

    def update(self, img_g):
        # 输入为moment图像
        self.img_g_curr = img_g

        if self.moment_mode:
            # 直接输出moment的分割图像
            self.movement_mean_pool = self.img_g_curr.mean_pooled(int(self.img_g_curr.width()/5), self.img_g_curr.height())
            self.mean_value_list = []
            norm_mean_value = True
            for i in range(5):
                mean_value = self.movement_mean_pool.get_pixel(i, 0)
                if norm_mean_value:
                    if mean_value > self.moment_max:
                        mean_value = self.moment_max
                    elif mean_value < self.moment_min:
                        mean_value = self.moment_min
                    self.mean_value_list.append((mean_value - self.moment_min)/(self.moment_max - self.moment_min))
                else:
                    self.mean_value_list.append(mean_value)
        else:
            if self.init_ok:
                # get p i s layer output
                self.p_layer = self.img_g_curr.copy().difference(self.img_g_prev)
                self.i_layer = self.p_prev.copy().morph(1, self.w_1, mul=1)
                self.i_layer = self.i_layer.mean(7)
                self.s_layer = self.p_layer.copy().sub(self.i_layer)

                # self.s_layer_mean_pool = self.s_layer.mean_pooled(34, 120)
                # print(self.s_layer_mean_pool)
                self.mean_value_list = []
                for i in range(5):
                    # mean_value = self.s_layer_mean_pool.get_pixel(i, 0) - 10
                    mean_value = 0
                    if mean_value < 0:
                        mean_value = 0
                    self.mean_value_list.append(mean_value)
                # print(mean_value_list)

                self.img_g_prev = self.img_g_curr.copy()
                self.p_prev = self.p_layer.copy()
            else:
                # 首次进入，需要初始化
                # print('start init lgmd')
                print('before start init lgmd: ', gc.mem_free())
                self.img_g_prev = self.img_g_curr.copy()  # 上一帧图像
                print('1-', gc.mem_free())
                self.p_layer = self.img_g_curr.difference(self.img_g_prev)
                print('2-', gc.mem_free())
                self.p_prev = self.p_layer.copy()
                print('3-', gc.mem_free())
                self.i_layer = self.p_layer.copy()
                print('4-', gc.mem_free())
                self.s_layer = self.p_layer.copy()
                print('5-', gc.mem_free())

                # self.s_layer_mean_pool = self.s_layer.mean_pooled(34, 120)
                # print(self.s_layer_mean_pool)

                self.mean_value_list = []
                for i in range(5):
                    # mean_value = self.s_layer_mean_pool.get_pixel(i, 0)
                    mean_value = 0
                    self.mean_value_list.append(mean_value)
                # print(self.mean_value_list)

                self.lgmd_out = 0
                self.init_ok = True

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 200)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

# 手动设置曝光时间
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, 50000) # 设置曝光时间（us）

# sensor.set_windowing((320, 200)) #取中间的640*80区域

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

moment_mode = False  # 设置使用LGMD还是moment
lgmd = LGMD(moment_mode)

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


    img_moment = img_00.div(img_edge, invert=True) # edge/img_00  得到moment图像

    lgmd.update(img_moment)

    # img_curr.replace(img_moment)

    img_curr.replace(lgmd.s_layer)

    del img_edge, img_00
    del img_moment
    gc.collect()

    # print('3-Used: ' + str(gc.mem_alloc()) + ' Free: ' + str(gc.mem_free()))
    # Note: OpenMV Cam runs about half as fast when connected to the IDE. The FPS should increase once disconnected.
    print(clock.fps(), 'Used: ' + str(gc.mem_alloc()) + ' Free: ' + str(gc.mem_free()))
