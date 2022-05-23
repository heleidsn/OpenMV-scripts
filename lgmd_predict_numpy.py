# LGMD openMV4 plus

import sensor, image, time, tf
import ulab
from ulab import numpy as np

class LGMD():
    def __init__(self):
        self.init_ok = False
        self.img_g_curr = None
        self.img_g_prev = None

        self.p_layer = None
        self.p_prev = None
        self.i_layer = None
        self.s_layer = None

        self.wi = (0.125, 0.25, 0.125, 0.25, 0, 0.25, 0.125, 0.25, 0.125)
        self.Ki = 0.35

        self.edge_01 = (-1,-1,-1,0,0,0,1,1,1)
        self.edge_10 = (-1,0,1,-1,0,1,-1,0,1)

        self.lgmd_out = 0

    def update(self, img_g):
        self.img_g_curr = img_g
        self.img_g_curr = self.get_moment_norm(img_g)

        if self.init_ok:
            # get p i s layer output
            self.p_layer = self.img_g_curr.copy().difference(self.img_g_prev)
            self.i_layer = self.p_layer.copy().mean(1)
            self.s_layer = self.p_layer.copy().sub(self.i_layer.copy())

            self.s_layer_mean_pool = self.s_layer.mean_pooled(34, 120)
            # print(self.s_layer_mean_pool)
            self.mean_value_list = []
            for i in range(5):
                mean_value = self.s_layer_mean_pool.get_pixel(i, 0) - 10
                if mean_value < 0:
                    mean_value = 0
                self.mean_value_list.append(mean_value)
            # print(mean_value_list)

            self.img_g_prev = self.img_g_curr.copy()
            self.p_prev = self.p_layer.copy()
        else:
            self.img_g_prev = self.img_g_curr.copy()  # 上一帧图像
            self.p_layer = self.img_g_curr.copy().difference(self.img_g_prev)
            self.p_prev = self.p_layer.copy()
            self.i_layer = self.p_layer
            self.s_layer = self.p_layer

            self.s_layer_mean_pool = self.s_layer.mean_pooled(34, 120)
            # print(self.s_layer_mean_pool)

            self.mean_value_list = []
            for i in range(5):
                mean_value = self.s_layer_mean_pool.get_pixel(i, 0)
                self.mean_value_list.append(mean_value)
            # print(self.mean_value_list)

            self.lgmd_out = 0
            self.init_ok = True

    def get_moment_norm(self, img_curr):
        img_01 = img_curr.copy().morph(1, self.edge_01)
        img_10 = img_curr.copy().morph(1, self.edge_10)
        img_add = img_01.add(img_10)
        img_00 = img_curr.copy().mean(1)  # Standard mean blurring filter 标准均值滤波
        img_moment = img_00.div(img_add, invert=True)

        return img_moment

# --------------------------以下代码为主函数--------------------------------

sensor.reset()                          #复位和初始化摄像头，执行sensor.run(0)停止。

# 设置信鸽上使用方向
# sensor.set_vflip(True)
# sensor.set_hmirror(True)

# 基本设置
sensor.set_pixformat(sensor.GRAYSCALE)  # 设置像素格式为彩色 RGB565 (或灰色GRAYSCALE)
sensor.set_framesize(sensor.QSIF)       # 设置帧大小为 QVGA (320x240) QCIF(176x144) QSIF(176*120)
sensor.skip_frames(time = 2000)         # 等待设置生效.
# sensor.set_gainceiling(8)             #设置增益，这是官方推荐的参数
clock = time.clock()                    # 创建一个时钟来追踪 FPS（每秒拍摄帧数）

# 创建lgmd类
lgmd = LGMD()
img_g_prev = sensor.snapshot()

# 加载numpy模型参数
w_1 = np.load('net_weights/w_1.npy')
w_2 = np.load('net_weights/w_2.npy')
w_3 = np.load('net_weights/w_3.npy')
b_1 = np.load('net_weights/b_1.npy')
b_2 = np.load('net_weights/b_2.npy')
b_3 = np.load('net_weights/b_3.npy')
b_1 = b_1.reshape((b_1.shape[0], 1))
b_2 = b_2.reshape((b_2.shape[0], 1))
b_3 = b_3.reshape((b_3.shape[0], 1))

# 串口通信设置

while(True):
    clock.tick()                    # 更新 FPS 时钟.
    img_curr = sensor.snapshot()
    lgmd.update(img_curr)
    img_curr.replace(lgmd.s_layer)

    lgmd_feature = lgmd.mean_value_list
    state_feature = [127, 127, 127]
    feature_all = lgmd_feature + state_feature

    input = np.array(feature_all).reshape((8, 1))
    out_1 = np.dot(w_1, input)
    out_1_b = out_1 + b_1
    out_1_b = np.maximum(0,out_1_b)
    out_2 = np.dot(w_2, out_1_b)
    out_2_b = out_2 + b_2
    out_2_b = np.maximum(0,out_2_b)
    out_3 = np.dot(w_3, out_2_b)
    out_3_b = out_3 + b_3
    out_3_b = np.tanh(out_3_b)

    roll_cmd = out_3_b[0][0] * 0.6980 * 57.3  # 转换成角度，最大40度

    # time.sleep_ms(100)

    print('feature_all: ', feature_all, 'roll_cmd: ', roll_cmd, 'FPS: ', clock.fps())

print('finish')
