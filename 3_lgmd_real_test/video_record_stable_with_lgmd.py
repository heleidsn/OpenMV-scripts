# Video_stable_v2
# 使用rotation_correction对视频进行稳定
# 2022-10-12
# Lei He
# 2023-01-30 上机测试
# 2023-03-18 新校区测试 原始openmv损坏 无法检测到相机 更换新的openmv
# 2023-03-21 能够同时记录稳定前和稳定后的视频 发现可以在不同阶段添加记录即可，不需要增加framebuffer
# 2023-03-22 搞定兴趣区域，能够实现增稳和非增稳的切换，使用RC8, 调整了录制文件命名规则
# 2023-03-22 增加LGMD相关内容
# 发现串口无法通信

import sensor, image, time, mjpeg, pyb, gc, struct, math
from pyb import Pin, Timer, LED, RTC, ExtInt
from pyb import UART

# mavlink相关代码
packet_sequence = 0
MAV_system_id = 1
MAV_component_id = 0x54

def get_attitude_force():
    # 强制更新姿态信息
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
    # 强制更新遥控器信息
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

video_folder = 'video_record_stable_03_21'  # 设置video记录地址

uart = UART(3, 3000000)

flag_record_image = False
flag_record_save = False

rc_in = [0, 0]                    # rc_in分别对应通道7和通道8
rc_in_last = [0, 0]               # 通道7用来控制是否录像 通道8用来控制录像准类

clock = time.clock() # 跟踪FPS帧率
now = time.ticks_ms()

step = 1 # used for update RC input

flag_use_stable = False

X_OFFSET = 0
Y_OFFSET = 0
y_rotation_counter = 0
z_rotation_counter = 0
ZOOM_AMOUNT = 1
FOV_WINDOW = 80

# 获取当前图像大小
img_curr = sensor.snapshot()
img_size = [img_curr.height(), img_curr.width()]  # 480 640

# 设置裁切大小
w_crop = 220
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

    # 串口测试
    # data = uart.read()
    # print(data)


    # ------------------更新姿态和遥控信息---------------------------------
    # print('wait for attitude')
    attitude_rad = get_attitude_force()  # 单独force更新能到80HZ 带上获取图像46Hz 带上

    if step >= 10:
        # 每50步强制更新一下RC，将主要时间让给姿态更新
        get_rc_force()
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
            video_path = video_folder + '/{}_stable.mjpeg'.format(new_name)
            video_path_origin = video_folder + '/{}_origin.mjpeg'.format(new_name)
            video_path_crop = video_folder + '/{}_crop.mjpeg'.format(new_name)

            m_origin = mjpeg.Mjpeg(video_path_origin)
            m = mjpeg.Mjpeg(video_path)
            m_crop = mjpeg.Mjpeg(video_path_crop, width=w_crop, height=h_crop)

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
        if flag_record_image:
            '''
            边处理边存储图像
            由于OpenMV堆栈大小限制，无法对图像进行拷贝等，只能一边处理一边记录
            '''

            # 1 - 获取当前传感器图像
            img_curr = sensor.snapshot()
            m_origin.add_frame(img_curr, quality=100)  # 存储原始图像

            # 2- 图像校正和消抖
            img_curr = img_curr.lens_corr(2.2, 1)  # 镜头校正 原始值2.3 1
            img_curr = img_curr.rotation_corr(x_rotation = pitch - pitch_offset_deg,
                                               y_rotation = 0,
                                               z_rotation = roll,
                                               x_translation = X_OFFSET,
                                               y_translation = -y_diff,
                                               zoom = ZOOM_AMOUNT,
                                               fov = FOV_WINDOW)
            m.add_frame(img_curr, quality=100)  # 存储mjpeg文件
            # 3 - # 裁切 320*240 220*100 roi=(x, y, w, h)
            img_crop = img_curr.crop(roi=(x_crop, y_crop, w_crop, h_crop), copy=True)
            m_crop.add_frame(img_crop, quality=100)  # 存储mjpeg文件
            # 4 - 指示灯变化
            blue_led.toggle()
            print('recording')
        else:
            img_curr = sensor.snapshot()  # 获取当前传感器图像

            img_curr = img_curr.lens_corr(2.2, 1)  # 原始值2.3 1
            img_curr = img_curr.rotation_corr(x_rotation = pitch - pitch_offset_deg,
                                               y_rotation = 0,
                                               z_rotation = roll,
                                               x_translation = X_OFFSET,
                                               y_translation = -y_diff,
                                               zoom = ZOOM_AMOUNT,
                                               fov = FOV_WINDOW)

            m_crop = img_curr.crop(roi=(x_crop, y_crop, w_crop, h_crop))

            # 获取moment 或者 lmgd

            # 1 获得lgmd
            diff = m_crop.difference(m_crop_last)
            m_crop_last = m_crop.copy(copy_to_fb=True)

    else:
        img_curr = sensor.snapshot()        # 获取图像
        if flag_record_image:
            m_origin.add_frame(img_curr, quality=100)  # 存储原始图像

    # ----------------------------保存录像-----------------------------
    if flag_record_save:
        m_origin.close(fps)
        m.close(fps)
        m_crop.close(fps)
        flag_record_image = False
        flag_record_save = False
        print('video saved')

    # 串口输出
    fps = clock.fps()
    print('RC: ', rc_in, 'FPS: %.2f' % fps, 'time: ', time.ticks_ms() - now,
          # ' Free: ' + str(gc.mem_free()), 'step', step,
          'roll: %2.2f' % math.degrees(attitude_rad[0]),
          'pitch: %2.2f' % math.degrees(attitude_rad[1]),
          'yaw: %.2f' % math.degrees(attitude_rad[2]),
          'recording: ', flag_record_image
          )
    now = time.ticks_ms()

print('finish')
