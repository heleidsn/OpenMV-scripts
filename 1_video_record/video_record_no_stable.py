# 保存视频例程
#
# 注意：您将需要SD卡来运行此演示。
#
# 您可以使用OpenMV Cam来录制mjpeg文件。 您可以为记录器对象提供JPEG帧
# 或RGB565 /灰度帧。 一旦你完成了一个Mjpeg文件的录制，你可以使用VLC
# 来播放它。 如果你在Ubuntu上，那么内置的视频播放器也可以工作。
import sensor, image, time, mjpeg, pyb, gc, struct, math
from pyb import Pin, Timer, LED, RTC, ExtInt
from pyb import UART

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

def get_attitude_force():
    global uart
    state = [0, 0, 0]
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
                    state[0] = struct.unpack('f', PAYLOAD[4:8])[0]
                    state[1] = struct.unpack('f', PAYLOAD[8:12])[0]
                    state[2] = struct.unpack('f', PAYLOAD[12:16])[0]
                    done = True

    return state

def get_rc_force():
    global uart
    global rc_7

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
                    chanel = 7
                    rc_7 = struct.unpack('h', PAYLOAD[4+(chanel*2-2):4+(chanel*2)])[0]
                    done = True

# -----------------------------Main Loop--------------------------

RED_LED_PIN = 1
BLUE_LED_PIN = 3

rc_7 = 0
rc_7_last = 0

sensor.reset() # 初始化sensor
sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.GRAYSCALE
#设置图像色彩格式，有RGB565色彩图和GRAYSCALE灰度图两种
sensor.set_framesize(sensor.QVGA) # or sensor.QQVGA (or others)
sensor.set_vflip(True)
sensor.set_hmirror(True)
#设置图像像素大小

sensor.skip_frames(time = 200) # 让新的设置生效

#时间初始化，用于给图片命名
rtc=RTC()
dateTime = rtc.datetime()
hour = '%02d' % dateTime[4]
minute = '%02d' % dateTime[5]
second = '%02d' % dateTime[6]

##############################################
#  按键和其回调函数
##############################################
# KEY = Pin('P0', Pin.IN, Pin.PULL_UP)   # 按键
blue_led  = LED(3)                     # 蓝色指示灯
red_led = LED(1)
key_node = False  #按键标志位

# get image size
img_curr = sensor.snapshot()
img_size = [img_curr.height(), img_curr.width()]  # 480 640
img_fov_deg = [55.6, 70.8]

# img_stable_h = 150  # 得到角度  300在480中对应34.75度
img_stable_fov_h_deg = 30
pitch_offset_deg = 10
img_stable_h = int(img_size[0] * img_stable_fov_h_deg / img_fov_deg[0])
print('img_stable_h: ', img_stable_h)

video_folder = 'video_record_1011'
m = mjpeg.Mjpeg(video_folder + '/original_{}_{}_{}.mjpeg'.format(hour, minute, second))
m_2 = mjpeg.Mjpeg(video_folder + '/stable_{}_{}_{}.mjpeg'.format(hour, minute, second), width=img_size[1], height=img_stable_h)


uart = UART(3, 3000000)

clock = time.clock() # 跟踪FPS帧率

flag_record_image = False
flag_record_save = False

now = time.ticks_ms()

step = 1 # used for update RC input

flag_use_stable = True

while(True):
    clock.tick()                        # 更新 FPS 时钟.
    step = step + 1

    # -----------------获取原始图像-------------------------------------
    img_curr = sensor.snapshot()        # 获取图像 目前是480 * 640

    if flag_record_image:
        m.add_frame(img_curr, quality=100)  # 存储mjpeg文件

    # ------------------更新数据 pitch---------------------------------
    if flag_use_stable:
        attitude = get_attitude_force()  # 单独force更新能到80HZ 带上获取图像46Hz 带上

    if step >= 50:
        get_rc_force()
        step = 1

    # -----------------check if change record mode-------------------
    if rc_7 == 0:
        # waiting for RC
        red_led.on()
        blue_led.off()
    elif rc_7 == 1094:
        # get rc
        flag_record_image = False
        flag_video_saved = False
        blue_led.off()
        red_led.toggle()
    elif rc_7 == 1514:
        if rc_7_last == 1094:
            flag_record_image = True
            red_led.off()
            blue_led.on()
    elif rc_7 == 1934:
        if rc_7_last == 1514:
            flag_record_save = True
            red_led.off()
            blue_led.off()
    else:
        red_led.toggle()

    rc_7_last = rc_7

    # --------------stable image according to pitch angle----------------
    if flag_use_stable:
        pitch = attitude[1] - math.radians(pitch_offset_deg)
        y_diff = int((img_size[0]/2) * math.tan(pitch) / math.tan(math.radians(img_fov_deg[0] / 2)))

        # roi = x,y,w,h 需要计算一个x x=0对应最大的fov 55.6度  current y_center 55
        y_center = int((img_size[0] - img_stable_h) / 2) + y_diff

        if y_center < 0:
            y_center = 0
        elif y_center > int((img_size[0] - img_stable_h)):
            y_center = int((img_size[0] - img_stable_h))


        img_curr.to_grayscale(roi=(0, y_center, img_size[1], img_stable_h))

        if flag_record_image:
            m_2.add_frame(img_curr, quality=100)
            blue_led.toggle()

    # ----------------check if save videos-----------------------------
    if flag_record_save:
        m.close(fps)
        m_2.close(fps)

        # 新建mjpeg文件
        rtc=RTC()
        dateTime = rtc.datetime()
        hour = '%02d' % dateTime[4]
        minute = '%02d' % dateTime[5]
        second = '%02d' % dateTime[6]

        m = mjpeg.Mjpeg(video_folder + '/original_{}_{}_{}.mjpeg'.format(hour, minute, second))
        m_2 = mjpeg.Mjpeg(video_folder + '/stable_{}_{}_{}.mjpeg'.format(hour, minute, second), width=img_size[1], height=img_stable_h)

        flag_record_image = False
        flag_record_save = False
        print('video saved')

    fps = clock.fps()
    print('data: ', rc_7, 'FPS: ', fps, 'time: ', time.ticks_ms() - now,
          ' Free: ' + str(gc.mem_free()), 'step', step, 'pitch_deg', math.degrees(pitch),
          'y_diff', y_diff, 'y_center', y_center)
    now = time.ticks_ms()


print('finish')
