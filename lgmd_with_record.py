# LGMD openMV4 plus

import sensor, image, time, tf, mjpeg, struct, math
from pyb import Pin, Timer, LED, RTC, ExtInt
import ulab
from ulab import numpy as np
from pyb import UART

'''
输入：
    LGMD图像信息（5）
    当前位置
输出：
    滚转角度指令（1）

注意：
    1. 中间会使用目标位置和当前位置来确定
    2. 刚开始使用moment image进行判断，但是会记录lgmd的数据
'''

class LGMD():
    def __init__(self):
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

        self.moment_mode = True
        self.moment_min = 25
        self.moment_max = 50

    def update(self, img_g):
        # 输入为moment图像
        self.img_g_curr = img_g

        if self.moment_mode:
            # 直接输出moment的分割图像
            self.movement_mean_pool = self.img_g_curr.mean_pooled(34, 120)
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
                self.img_g_prev = self.img_g_curr.copy()  # 上一帧图像
                self.p_layer = self.img_g_curr.difference(self.img_g_prev)
                self.p_prev = self.p_layer.copy()
                self.i_layer = self.p_layer.copy()
                self.s_layer = self.p_layer.copy()

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

# ----------------------------串口设置-------------------------
uart = UART(3, 921600)
# 需要更新的状态量： x, y, z, yaw -pi to pi
pose_and_yaw = [0, 0, 0, 0]
goal_pose = [160, 40]



packet_sequence = 0
MAV_system_id = 1
MAV_component_id = 0x54
MAVLINK_MSG_ID = 254  # DEBUG ( #254 )


MAVLINK_MESSAGE_CRCS = [50, 124, 137, 0, 237, 217, 104, 119, 117, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 137, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 78, 196, 132, 0, 15, 3, 0, 0, 0, 0, 0, 167, 183, 119, 191, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158, 152, 143, 0, 0, 14, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63, 54, 47, 0, 0, 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 138, 108, 32, 185, 84, 34, 174, 124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25, 226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 47, 72, 131, 127, 0, 103, 154, 178, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 189, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 36, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 208, 0, 0, 0, 0, 163, 105, 151, 35, 150, 179, 0, 0, 0, 0, 0, 90, 104, 85, 95, 130, 184, 81, 8, 204, 49, 170, 44, 83, 46, 0]

def checksum(data, extra): # https://github.com/mavlink/c_library_v1/blob/master/checksum.h
    output = 0xFFFF
    for i in range(len(data)):
        tmp = data[i] ^ (output & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    tmp = extra ^ (output & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return output

def send_debug_value(roll_cmd):
    global packet_sequence
    temp = struct.pack("<lfb",
                       0,  # time_boot_ms uint32_t
                       roll_cmd,              # value
                       1                      # uint8_t ind
                       )

    # out = struct.unpack("<l10sf", temp)
    # print(out, len(out))

    temp = struct.pack("<bbbbb9s",
                       9,                     # len
                       packet_sequence & 0xFF, # seq
                       MAV_system_id,          # sid
                       MAV_component_id,       # cid
                       MAVLINK_MSG_ID,         # mid
                       temp)                   # data
    temp = struct.pack("<b14sh",
                       0xFE,                   # head
                       temp,                   # data
                       checksum(temp, MAVLINK_MESSAGE_CRCS[MAVLINK_MSG_ID]))
    packet_sequence += 1
    uart.write(temp)
    time.sleep_ms(10)

def get_roll_test(uart):
    data = uart.read()
    if data is not None:
        LEN = int.from_bytes(data[1:2], "big")
        SEQ = int.from_bytes(data[2:3], "big")
        SID = int.from_bytes(data[3:4], "big")
        CID = int.from_bytes(data[4:5], "big")
        MID = int.from_bytes(data[5:6], "big")
        PAYLOAD = data[6:LEN+6]
        if MID == 30:
            # check payload lenght. Only unpack correct massages.
            if len(PAYLOAD)==28:
                roll_rad = struct.unpack('f', PAYLOAD[4:8])[0]
            else:
                roll_rad = 0.00

    return roll_rad

def get_local_position(uart):
    local_pose = [None, None, None]
    data = uart.read()
    if data is not None:
        LEN = int.from_bytes(data[1:2], "big")
        SEQ = int.from_bytes(data[2:3], "big")
        SID = int.from_bytes(data[3:4], "big")
        CID = int.from_bytes(data[4:5], "big")
        MID = int.from_bytes(data[5:6], "big")
        PAYLOAD = data[6:LEN+6]
        if MID == 32:
            # check payload lenght. Only unpack correct massages.
            if len(PAYLOAD)==28:
                local_pose[0] = struct.unpack('f', PAYLOAD[4:8])[0]
                local_pose[1] = struct.unpack('f', PAYLOAD[8:12])[0]
                local_pose[2] = struct.unpack('f', PAYLOAD[12:16])[0]

    return local_pose

def update_state(uart, pose_and_yaw):
    state = pose_and_yaw
    data = uart.read()
    if data is not None:
        LEN = int.from_bytes(data[1:2], "big")
        SEQ = int.from_bytes(data[2:3], "big")
        SID = int.from_bytes(data[3:4], "big")
        CID = int.from_bytes(data[4:5], "big")
        MID = int.from_bytes(data[5:6], "big")
        PAYLOAD = data[6:LEN+6]
        # update local pose
        if MID == 32:
            # check payload lenght. Only unpack correct massages.
            if len(PAYLOAD)==28:
                state[0] = struct.unpack('f', PAYLOAD[4:8])[0]
                state[1] = struct.unpack('f', PAYLOAD[8:12])[0]
                state[2] = struct.unpack('f', PAYLOAD[12:16])[0]
        # update yaw
        if MID == 30:
            # check payload lenght. Only unpack correct massages.
            if len(PAYLOAD)==28:
                state[3] = struct.unpack('f', PAYLOAD[12:16])[0] # yaw -pi to pi

    return state

def get_yaw_error(goal_pose, pose_and_yaw):
    # 根据目标点位置、当前位置得到相对航迹角
    # 再根据航迹角和当前yaw得到yaw_err
    y_err = goal_pose[1] - pose_and_yaw[1]
    x_err = goal_pose[0] - pose_and_yaw[0]
    yaw_sp = math.atan2(y_err, x_err)

    yaw_error = yaw_sp - pose_and_yaw[2]

    # print(yaw_sp* 57.3, pose_and_yaw[2] * 57.3, yaw_error * 57.3)

    if yaw_error > math.pi:
        yaw_error -= 2*math.pi
    elif yaw_error < -math.pi:
        yaw_error += 2*math.pi

    return yaw_error

# --------------------------以下代码为主函数--------------------------------

sensor.reset()                          #复位和初始化摄像头，执行sensor.run(0)停止。

# 设置信鸽上使用方向
flag_is_flapping_wing = True
if flag_is_flapping_wing:
    sensor.set_vflip(True)
    sensor.set_hmirror(True)

# 基本设置
sensor.set_pixformat(sensor.GRAYSCALE)  # 设置像素格式为彩色 RGB565 (或灰色GRAYSCALE)
sensor.set_framesize(sensor.QSIF)       # 设置帧大小为 QVGA (320x240) QCIF(176x144) QSIF(176*120)
sensor.skip_frames(time = 200)         # 等待设置生效.
# sensor.set_gainceiling(8)             #设置增益，这是官方推荐的参数
clock = time.clock()                    # 创建一个时钟来追踪 FPS（每秒拍摄帧数）

# 创建lgmd类
lgmd = LGMD()
img_g_prev = sensor.snapshot()


# 设置控制类型
control_type = 2 # 1-神经网络控制  2-线性控制器

if control_type == 1:
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
elif control_type == 2:
    control_weight = np.array([1, 2, 3, -2, -1, 3])
else:
    print("Please set control type: 1-神经网络控制  2-线性控制器")


edge_01_l = (-1,-1,-1,0,0,0,1,1,1)
edge_01_r = (1,1,1,0,0,0,-1,-1,-1)
edge_10_l = (-1,0,1,-1,0,1,-1,0,1)
edge_10_r = (1,0,-1,1,0,-1,1,0,-1)

##############################################
#  视频录制
##############################################

flag_record_image = False

#时间初始化，用于给图片命名
rtc=RTC()
m = mjpeg.Mjpeg("FW_image_record_300s_VGA.mjpeg")
m_2 = mjpeg.Mjpeg("moment_img.mjpeg")
m_3 = mjpeg.Mjpeg("lgmd_img.mjpeg")

# 按键和回调函数
blue_led  = LED(3)                     # 蓝色指示灯
red_led = LED(1)
key_node = False  #按键标志位

KEY = Pin('P0', Pin.IN, Pin.PULL_UP)


# openmv4plus RAM Layout
# 256KB .DATA/.BSS/Heap/Stack
# 32MB Frame Buffer/Stack
# 256 KB DMA Buffers
# 目前在QSIF下只能同时存在9幅图片


##############################################
#  主循环
##############################################
while(True):
    clock.tick()                        # 更新 FPS 时钟.

    # 更新位置信息
    pose_and_yaw = update_state(uart, pose_and_yaw)

    # 更新yaw_error
    yaw_error = get_yaw_error(goal_pose, pose_and_yaw)

    # 图像处理
    img_curr = sensor.snapshot()        # 获取图像
    if flag_record_image:
        m.add_frame(img_curr, quality=100)  # 存储mjpeg文件

    img_01_l = img_curr.copy().morph(1, edge_01_l, mul=0.15)  # 得到左边边界
    img_01_r = img_curr.copy().morph(1, edge_01_r, mul=0.15)  # 得到右边边界
    img_01 = img_01_l.add(img_01_r)  # 得到垂直边界
    del img_01_l, img_01_r  # 删除图像，得到更多内存
    img_10_l = img_curr.copy().morph(1, edge_10_l, mul=0.15)  # 得到上边界
    img_10_r = img_curr.copy().morph(1, edge_10_r, mul=0.15)  # 得到下边界
    img_10 = img_10_l.add(img_10_r)  # 得到水平边界
    del img_10_l, img_10_r
    img_edge = img_10.add(img_01)  # 相加得到最终边界
    img_00 = img_curr.copy().mean(1)  # 均值滤波
    img_moment = img_00.div(img_edge, invert=True) # edge/img_00  得到moment图像
    del img_edge, img_00

    # 删除不用的图像 节约内存
    # 计算image moment能在QSIF下跑到36fps
    # img_curr.replace(img_moment)

    # 计算lgmd输出 23fps
    if flag_record_image:
        m_2.add_frame(img_moment, quality=100)

    lgmd.update(img_moment)
    img_curr.replace(img_moment)

    if flag_record_image:
        m_3.add_frame(lgmd.s_layer, quality=100)

    lgmd_feature = lgmd.mean_value_list
    state_feature = [yaw_error]
    feature_all = lgmd_feature + state_feature

    input = np.array(feature_all).reshape((6, 1))

    if control_type == 1:
        # Control-1：使用神经网络控制
        out_1 = np.dot(w_1, input)
        out_1_b = out_1 + b_1
        out_1_b = np.maximum(0,out_1_b)
        out_2 = np.dot(w_2, out_1_b)
        out_2_b = out_2 + b_2
        out_2_b = np.maximum(0,out_2_b)
        out_3 = np.dot(w_3, out_2_b)
        out_3_b = out_3 + b_3
        out_3_b = np.tanh(out_3_b)
        control_out = out_3_b[0][0]
    elif control_type == 2:
        # control_out = lgmd_featrue * control_weight
        control_out = np.dot(control_weight, input)[0]
        if control_out > 1:
            control_out = 1
        elif control_out < -1:
            control_out = -1

    # Control-2：使用线性控制器控制
    roll_cmd = control_out * 0.5235 * 57.3  # 转换成角度，最大30度

    send_debug_value(float(roll_cmd))

    # time.sleep_ms(100)

    print('input: ', feature_all, 'roll_cmd: ', roll_cmd, 'c_out', control_out, 'FPS: ', clock.fps())
    # print(pose_and_yaw, 'FPS: ', clock.fps())


print('finish')

if flag_record_image:
    m.close(clock.fps())
    m_2.close(clock.fps())
    m_3.close(clock.fps())
    print('image saved')
