# 最终避障版本
# 使用rotation_correction对视频进行稳定，应该是最终用于实验的版本了
# 2022-10-12
# Lei He

'''
Change logs:
软件部分：
2023-01-30 上机测试
2023-03-18 新校区测试 原始openmv损坏 无法检测到相机 更换新的openmv
2023-03-21 能够同时记录稳定前和稳定后的视频 发现可以在不同阶段添加记录即可，不需要增加framebuffer
2023-03-22 搞定兴趣区域，能够实现增稳和非增稳的切换，使用RC8, 调整了录制文件命名规则
2023-03-22 增加LGMD相关内容
2023-03-28 尝试添加LGMD所有内容 不做LGMD计算的情况下QVGA-210*100 gray输入，15 FPS 决定不做LGMD 直接用moment的图像 实现了图像分割
2023-03-29 新校区实验

硬件部分：
    发现串口无法通信，将线重新做了一遍，可以了
'''

import sensor, image, time, mjpeg, pyb, gc, struct, math
from pyb import Pin, Timer, LED, RTC, ExtInt
from pyb import UART
from ulab import numpy as np

# 计算yaw error
def get_yaw_error(goal_pose, pose_xyz, yaw):
    '''
    根据目标点位置、当前位置得到相对航迹角
    再根据航迹角和当前yaw得到yaw_err
    '''
    y_err = goal_pose[1] - pose_xyz[1]
    x_err = goal_pose[0] - pose_xyz[0]
    yaw_sp = math.atan2(y_err, x_err)

    yaw_error = yaw_sp - yaw

    # print('yaw_sp: ', yaw_sp* 57.3, 'yaw: ', yaw * 57.3, 'yaw_err: ', yaw_error * 57.3)

    if yaw_error > math.pi:
        yaw_error -= 2*math.pi
    elif yaw_error < -math.pi:
        yaw_error += 2*math.pi

    return yaw_error



# LGMD相关代码
def get_moment_image(img_curr):
    '''
    输入：灰度图像
    输出：该图像的moment
    '''
    edge_01_l = (-1,-1,-1,0,0,0,1,1,1)
    edge_01_r = (1,1,1,0,0,0,-1,-1,-1)
    edge_10_l = (-1,0,1,-1,0,1,-1,0,1)
    edge_10_r = (1,0,-1,1,0,-1,1,0,-1)

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

    return img_moment

def get_split_out(img_moment):
    '''
    将图像进行分割，得到输出
    '''
    movement_mean_pool = img_moment.mean_pooled(int(img_moment.width()/5),img_moment.height()) # 计算均值
    mean_value_list = []

    for i in range(5):
        mean_value = movement_mean_pool.get_pixel(i, 0)
        mean_value_list.append(mean_value)

    return mean_value_list

# mavlink相关代码
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

def get_attitude_force(is_sim=False):
    # 强制更新姿态信息
    global uart
    attitude_rad = [0, 0, 0]
    done = False

    if is_sim:
        done = True

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

def get_position_force(is_sim=False):
    # 强制更新位置信息
    global uart
    pose_xyz = [0, 0, 0]
    done = False

    if is_sim:
        done = True

    while not done:
        data = uart.read()
        # print(data)
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
                    pose_xyz[0] = struct.unpack('f', PAYLOAD[4:8])[0]
                    pose_xyz[1] = struct.unpack('f', PAYLOAD[8:12])[0]
                    pose_xyz[2] = struct.unpack('f', PAYLOAD[12:16])[0]
                    done = True

    return pose_xyz

def get_rc_force(is_sim=False):
    # 强制更新遥控器信息
    global uart
    global rc_in

    msg = 0
    done = False

    if is_sim:
        done = True

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
#                          主程序
############################################################################
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)  # GRAYSCALE or RGB565
sensor.set_framesize(sensor.QVGA) # or sensor.QQVGA (or others)
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time = 200) # 让新的设置生效

rtc=RTC()

blue_led  = LED(3)
red_led = LED(1)

record_index_file = 'record_name.txt'       # 录制文件命名记录

video_folder = 'video_record_stable_03_28_lgmd'  # 设置video记录地址

control_weight = np.array([1, 2, 3, -2, -1, 1])  # 用于控制的权重，前五位为LGMD输出，后一位为yaw偏角
roll_cmd_max = 40
goal_pose = [90, -20]
pose_xyz = [0, 0, 0]

uart = UART(3, 921600)

flag_record_image = False
flag_record_save = False
flag_is_debug = False          # 是否为debug模式，即不需要飞控数据
flag_use_stable = False

rc_in = [0, 0]                    # rc_in分别对应通道7和通道8
rc_in_last = [0, 0]               # 通道7用来控制是否录像 通道8用来控制录像准类

clock = time.clock() # 跟踪FPS帧率
now = time.ticks_ms()

step = 1 # used for update RC input



X_OFFSET = 0
Y_OFFSET = 0
y_rotation_counter = 0
z_rotation_counter = 0
ZOOM_AMOUNT = 1
FOV_WINDOW = 80

# 获取当前图像大小
img_curr = sensor.snapshot()
img_size = [img_curr.height(), img_curr.width()]  # 480 640
print(img_curr.height(), img_curr.width())

# 设置裁切大小
w_crop = 210
h_crop = 100
x_crop = int((img_size[1] - w_crop) / 2)
y_crop = int((img_size[0] - h_crop) / 2)

m_crop_last = img_curr.crop(roi=(x_crop, y_crop, w_crop, h_crop), copy=True)

# 设置镜头FoV
# img_fov_deg = [65.8, 81.8] # 标准镜头
img_fov_deg = [90, 120]  # 广角镜头

img_stable_fov_h_deg = 40
pitch_offset_deg = 15
img_stable_h = int(img_size[0] * img_stable_fov_h_deg / img_fov_deg[0])
print('img_stable_h: ', img_stable_h)

# -----------------------------主循环----------------------------------
while(True):
    clock.tick()                        # 更新 FPS 时钟.
    step = step + 1

    # ------------------更新姿态和遥控信息---------------------------------
    # print('wait for attitude')
    attitude_rad = get_attitude_force(is_sim=flag_is_debug)  # 单独force更新能到80HZ 带上获取图像46Hz 带上

    # print('wait for position')

    pose_xyz = get_position_force(is_sim=flag_is_debug)
    # print(pose_xyz)
    # pose_xyz = [0, 0, 0]

    if step >= 10:
        # 每10步或者50步强制更新一下RC，将主要时间让给姿态更新
        get_rc_force(is_sim=flag_is_debug)
        step = 1

    # ------------------------判断是否进行录制---------------------------
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
            # 从1094变到1514：开始录制
            print('start recording')

            # 获取文件名
            with open(record_index_file, 'r+') as f:
                lines = f.readlines() #读取所有行
                last_line = lines[-1] #取最后一行
                # print(last_line)
                new_name = int(last_line) + 1

            with open(record_index_file, 'a') as f:
                f.write("\n")
                f.write('{}'.format(new_name))
                print('file_name: ', new_name)

            # 创建录制文件
            video_path_stable = video_folder + '/{}_stable.mjpeg'.format(new_name)
            video_path_origin = video_folder + '/{}_origin.mjpeg'.format(new_name)
            video_path_moment = video_folder + '/{}_moment.mjpeg'.format(new_name)

            m_origin = mjpeg.Mjpeg(video_path_origin, width=img_size[1], height=img_size[0])    # 原始视频
            m_stable = mjpeg.Mjpeg(video_path_stable, width=w_crop, height=h_crop)              # 稳定后视频
            m_moment = mjpeg.Mjpeg(video_path_moment, width=w_crop, height=h_crop)              # moment视频

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

    if rc_in[1] == 1094:  # 用于选择直接录制原始视频还是录制带稳定的视频
        flag_use_stable = True
    else:
        flag_use_stable = False

    rc_in_last = rc_in.copy()

    # -------------------------离线调试-------------------------------------
    flag_use_stable = True

    # --------------------------稳定图像-------------------------------------
    pitch = math.degrees(attitude_rad[1])
    roll = math.degrees(attitude_rad[0])
    yaw = math.degrees(attitude_rad[2])

    pitch_with_bias = math.radians(pitch) - math.radians(pitch_offset_deg)

    y_diff = int((img_size[0]/2) * math.tan(pitch_with_bias) / math.tan(math.radians(img_fov_deg[0] / 2)))

    # roi = x,y,w,h 需要计算一个x x=0对应最大的fov 55.6度  current y_center 55
    y_center = int((img_size[0] - img_stable_h) / 2) + y_diff

    if y_center < 0:
        y_center = 0
    elif y_center > int((img_size[0] - img_stable_h)):
        y_center = int((img_size[0] - img_stable_h))

    if flag_use_stable:
        '''
        边处理边存储图像
        由于OpenMV堆栈大小限制，无法对图像进行拷贝等，只能一边处理一边记录
        '''

        # 1 - 获取当前传感器图像
        img_curr = sensor.snapshot()
        # if flag_record_image:
            # m_origin.add_frame(img_curr, quality=100)  # 存储原始图像

        # 2 - 图像校正和消抖
        img_curr = img_curr.lens_corr(2.2, 1)  # 镜头校正 原始值2.3 1
        img_curr = img_curr.rotation_corr(x_rotation = pitch - pitch_offset_deg,
                                           y_rotation = 0,
                                           z_rotation = roll,
                                           x_translation = X_OFFSET,
                                           y_translation = -y_diff,
                                           zoom = ZOOM_AMOUNT,
                                           fov = FOV_WINDOW)

        # 3 - 裁切 得到稳定后图像 320*240 220*100 roi=(x, y, w, h)
        img_crop = img_curr.crop(roi=(x_crop, y_crop, w_crop, h_crop))
        if flag_record_image:
            m_stable.add_frame(img_crop, quality=100)   # 存储稳定后图像

        # 4 - 计算图像的moment
        img_moment = get_moment_image(img_crop)
        img_crop.replace(img_moment)  # 显示img_moment

        # 计算图像分割
        split_out = get_split_out(img_moment)

        if flag_record_image:
            m_moment.add_frame(img_moment, quality=100)  # 存储moment视频
            blue_led.toggle()  #  指示灯变化
            print('recording')

    else:
        # 只存储原始图像，准备废弃
        img_curr = sensor.snapshot()        # 获取图像
        if flag_record_image:
            m_origin.add_frame(img_curr, quality=100)  # 存储原始图像
        split_out = np.array([0, 0, 0, 0, 0])

    # ----------------------------计算yaw error-----------------------
    # print('cal yaw_error')
    yaw_error = get_yaw_error(goal_pose, pose_xyz, attitude_rad[2])

    # ----------------------------计算控制指令--------------------------
    lgmd_feature = split_out
    # print(lgmd_feature)

    # 需要对lgmd_feature进行归一化，从10-40 归一化到 0-1
    lgmd_min = 10
    lgmd_max = 30
    for i in range(len(lgmd_feature)):
        lgmd_feature[i] = (lgmd_feature[i] - lgmd_min) / (lgmd_max - lgmd_min)
        if lgmd_feature[i] > 1:
            lgmd_feature[i] = 1
        if lgmd_feature[i] < 0:
            lgmd_feature[i] = 0

        # lgmd_feature[i] = 0 # for debug

    state_feature = [yaw_error]
    feature_all = lgmd_feature + state_feature

    feature_input = np.array(feature_all).reshape((6, 1))
    control_out = np.dot(control_weight, feature_input)[0]
    if control_out > 1:
        control_out = 1
    elif control_out < -1:
        control_out = -1
    roll_cmd_deg = control_out * math.radians(40) * 57.3  # 转换成角度，最大30度 这个0.5235是啥没有搞清楚

    send_debug_value(float(math.radians(roll_cmd_deg)))
    # ----------------------------保存录像-----------------------------
    if flag_record_save:
        m_origin.close(fps)
        m_stable.close(fps)
        m_moment.close(fps)
        flag_record_image = False
        flag_record_save = False
        print('video saved')

    # 串口输出
    fps = clock.fps()

    print('RC: ', rc_in, 'FPS: %.2f' % fps,
          # 'time: ', time.ticks_ms() - now,
          # ' Free: ' + str(gc.mem_free()), 'step', step,
          'r: %2.2f' % math.degrees(attitude_rad[0]),
          'r_cmd: %2.2f' % roll_cmd_deg,
          'p: %2.2f' % math.degrees(attitude_rad[1]),
          'y: %.2f' % math.degrees(attitude_rad[2]),
          'y-e', yaw_error,
          # 'pose', pose_xyz,
          # 'recording: ', flag_record_image,
          'f: ', feature_all
          )

    # print('input: ', feature_all, 'roll_cmd: ', roll_cmd, 'yaw_error', yaw_error, 'FPS: ', fps, ' Free: ' + str(gc.mem_free()))
    now = time.ticks_ms()

print('finish')
