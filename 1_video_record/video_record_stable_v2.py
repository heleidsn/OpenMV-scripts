# Video_stable_v2
# 使用rotation_correction对视频进行稳定

import sensor, image, time, mjpeg, pyb, gc, struct, math
from pyb import Pin, Timer, LED, RTC, ExtInt
from pyb import UART

packet_sequence = 0
MAV_system_id = 1
MAV_component_id = 0x54

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
#                          Main Loop start here
############################################################################
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA) # or sensor.QQVGA (or others)
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time = 200) # 让新的设置生效

rtc=RTC()

blue_led  = LED(3)
red_led = LED(1)

video_folder = 'video_record_stable_1012'

uart = UART(3, 3000000)

flag_record_image = False
flag_record_save = False

rc_in = [0, 0]                    # rc_in分别对应通道7和通道8
rc_in_last = [0, 0]               # 通道7用来控制是否录像 通道8用来控制录像准类

clock = time.clock() # 跟踪FPS帧率
now = time.ticks_ms()

step = 1 # used for update RC input

flag_use_stable = True
flag_use_roi = True

X_OFFSET = 0
Y_OFFSET = 0
y_rotation_counter = 0
z_rotation_counter = 0
ZOOM_AMOUNT = 1.3
FOV_WINDOW = 65

pitch_offset_deg = 0


img_curr = sensor.snapshot()
img_size = [img_curr.height(), img_curr.width()]  # 480 640
img_fov_deg = [65.8, 81.8]

img_stable_fov_h_deg = 40
pitch_offset_deg = 10
img_stable_h = int(img_size[0] * img_stable_fov_h_deg / img_fov_deg[0])
print('img_stable_h: ', img_stable_h)

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


print('finish')
