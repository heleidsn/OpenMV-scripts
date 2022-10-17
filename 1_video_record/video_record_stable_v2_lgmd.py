# Video_stable_v2
# 使用rotation_correction对视频进行稳定

import sensor, image, time, mjpeg, pyb, gc, struct, math
from pyb import Pin, Timer, LED, RTC, ExtInt
from pyb import UART


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

def get_attitude_force():
    global uart
    attitude_rad = [0, 0, 0]
    done = False

    while not done:
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
                    attitude_rad[0] = struct.unpack('f', PAYLOAD[4:8])[0]
                    attitude_rad[1] = struct.unpack('f', PAYLOAD[8:12])[0]
                    attitude_rad[2] = struct.unpack('f', PAYLOAD[12:16])[0]
                    done = True

    return attitude_rad

def get_rc_force():
    global uart
    global rc_in

    msg = 0
    done = False

    while not done:
        data = uart.read()
        if data is not None:
            LEN = int.from_bytes(data[1:2], "big")
            SEQ = int.from_bytes(data[2:3], "big")
            SID = int.from_bytes(data[3:4], "big")
            CID = int.from_bytes(data[4:5], "big")
            MID = int.from_bytes(data[5:6], "big")
            PAYLOAD = data[6:LEN+6]
            if MID == 65:
                if len(PAYLOAD)==42:
                    chanel = [7, 8]
                    rc_in[0] = struct.unpack('h', PAYLOAD[4+(chanel[0]*2-2):4+(chanel[0]*2)])[0]
                    rc_in[1] = struct.unpack('h', PAYLOAD[4+(chanel[1]*2-2):4+(chanel[1]*2)])[0]
                    done = True

############################################################################
#                          Program starts here
############################################################################
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA) # or sensor.QQVGA (or others)
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time = 200) # 让新的设置生效

video_folder = 'video_record_stable_1012'

rtc=RTC()
blue_led  = LED(3)
red_led = LED(1)
uart = UART(3, 3000000)
clock = time.clock() # 跟踪FPS帧率
now = time.ticks_ms()

flag_record_image = False
flag_record_save = False
rc_in = [0, 0]                    # rc_in分别对应通道7和通道8
rc_in_last = [0, 0]               # 通道7用来控制是否录像 通道8用来控制录像准类
step = 1 # used for update RC input

flag_use_stable = True
flag_use_roi = True

X_OFFSET = 0
Y_OFFSET = 0
y_rotation_counter = 0
z_rotation_counter = 0
ZOOM_AMOUNT = 1.3
FOV_WINDOW = 65

pitch_offset_deg = 0  # set for pitch is not zero


img_curr = sensor.snapshot()
img_size = [img_curr.height(), img_curr.width()]  # 480 640
img_fov_deg = [65.8, 81.8]

img_stable_fov_h_deg = 40
pitch_offset_deg = 10
img_stable_h = int(img_size[0] * img_stable_fov_h_deg / img_fov_deg[0])
print('img_stable_h: ', img_stable_h)


# -----------------------------LGMD----------------------------------------------
# -------------------------------------------------------------------------------
moment_mode = False  # 设置使用LGMD还是moment
lgmd = LGMD(moment_mode)
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

# ---------------------------------------Main Loop----------------------------------------------
while(True):
    clock.tick()                        # 更新 FPS 时钟.
    step = step + 1

    # ------------------更新数据 pitch---------------------------------
    attitude_rad = get_attitude_force()  # 单独force更新能到80HZ 带上获取图像46Hz 带上

    # -----------------check if change record mode-------------------
    if step >= 50:
        # 每50步强制更新一下RC，将主要时间让给姿态更新
        get_rc_force()
        step = 1

    if rc_in[0] == 0:
        # 等待RC输入
        red_led.on()
        blue_led.off()
    elif rc_in[0] == 1094:
        # 获得RC，等待开始录制
        flag_record_image = False
        flag_video_saved = False
        blue_led.off()
        red_led.toggle()
    elif rc_in[0] == 1514:
        if rc_in_last[0] == 1094:
            # 开始录制
            print('start recording')
            dateTime = rtc.datetime()
            hour = '%02d' % dateTime[4]
            minute = '%02d' % dateTime[5]
            second = '%02d' % dateTime[6]

            video_path = video_folder + '/original_{}_{}_{}.mjpeg'.format(hour, minute, second)
            if flag_use_roi:
                m = mjpeg.Mjpeg(video_path, width=img_size[1], height=img_stable_h)
            else:
                m = mjpeg.Mjpeg(video_path)

            flag_record_image = True
            red_led.off()
            blue_led.on()
    elif rc_in[0] == 1934:
        if rc_in_last[0] == 1514:
            # 结束录制
            flag_record_save = True
            red_led.off()
            blue_led.off()
    else:
        red_led.toggle()

    rc_in_last = rc_in.copy()

    # -----------------获取原始图像-------------------------------------
    pitch = math.degrees(attitude_rad[1])
    roll = math.degrees(attitude_rad[0])
    yaw = math.degrees(attitude_rad[2])
    if flag_use_stable:
        img_curr = sensor.snapshot()
        img_curr = img_curr.lens_corr(2.3, 1)
        img_curr = img_curr.rotation_corr(x_rotation = pitch - pitch_offset_deg,
                                                   y_rotation = 0,
                                                   z_rotation = roll,
                                                   x_translation = X_OFFSET,
                                                   y_translation = Y_OFFSET,
                                                   zoom = ZOOM_AMOUNT,
                                                   fov = FOV_WINDOW)
    else:
        img_curr = sensor.snapshot()        # 获取图像

    pitch_with_bias = math.radians(pitch) - math.radians(pitch_offset_deg)

    y_diff = int((img_size[0]/2) * math.tan(pitch_with_bias) / math.tan(math.radians(img_fov_deg[0] / 2)))

    # roi = x,y,w,h 需要计算一个x x=0对应最大的fov 55.6度  current y_center 55
    y_center = int((img_size[0] - img_stable_h) / 2) + y_diff

    if y_center < 0:
        y_center = 0
    elif y_center > int((img_size[0] - img_stable_h)):
        y_center = int((img_size[0] - img_stable_h))

    if flag_use_roi:
        img_curr.to_grayscale(roi=(0, y_center, img_size[1], img_stable_h))

    if flag_record_image:
        print('recordng')
        m.add_frame(img_curr, quality=100)  # 存储mjpeg文件
        blue_led.toggle()

    # -------------------LGMD with stable-------------------------------------
    # print('3 - Used: ' + str(gc.mem_alloc()) + ' Free: ' + str(gc.mem_free()))
    img_01_l = img_curr.copy().morph(1, edge_01_l, mul=0.15)  # 得到左边边界
    img_01_r = img_curr.copy().morph(1, edge_01_r, mul=0.15)  # 得到右边边界
    img_01 = img_01_l.add(img_01_r)  # 得到垂直边界
    # print('4 - Used: ' + str(gc.mem_alloc()) + ' Free: ' + str(gc.mem_free()))
    del img_01_l, img_01_r  # 删除图像，得到更多内存
    # print('5 - Used: ' + str(gc.mem_alloc()) + ' Free: ' + str(gc.mem_free()))
    img_10_l = img_curr.copy().morph(1, edge_10_l, mul=0.15)  # 得到上边界
    img_10_r = img_curr.copy().morph(1, edge_10_r, mul=0.15)  # 得到下边界
    img_10 = img_10_l.add(img_10_r)  # 得到水平边界
    del img_10_l, img_10_r
    img_edge = img_10.add(img_01)  # 相加得到最终边界
    img_00 = img_curr.copy().mean(1)  # 均值滤波
    img_moment = img_00.div(img_edge, invert=True) # edge/img_00  得到moment图像
    del img_edge, img_00, img_01, img_10

    lgmd.update(img_moment)

    # ----------------------get control command------------------------------

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

    # ----------------check if save videos-----------------------------
    if flag_record_save:
        m.close(fps)
        flag_record_image = False
        flag_record_save = False
        print('video saved')

    fps = clock.fps()
    print('RC: ', rc_in, 'FPS: %.2f' % fps, 'time: ', time.ticks_ms() - now,
          # ' Free: ' + str(gc.mem_free()), 'step', step,
          'roll: %2.2f' % math.degrees(attitude_rad[0]),
          'pitch: %2.2f' % math.degrees(attitude_rad[1]),
          'yaw: %.2f' % math.degrees(attitude_rad[2]),
          'recording: ', flag_record_image
          )
    now = time.ticks_ms()

# -------------------------------Finish---------------------------------------------------------
print('finish')
