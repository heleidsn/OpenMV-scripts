# 保存视频例程
# record VGA videos according to RC7
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

blue_led  = LED(3)                     # 蓝色指示灯
red_led = LED(1)

rc_7 = 0
rc_7_last = 0

sensor.reset() # 初始化sensor
sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.GRAYSCALE
sensor.set_framesize(sensor.QVGA) # or sensor.QQVGA (or others)
sensor.set_vflip(True)
sensor.set_hmirror(True)
#设置图像像素大小

sensor.skip_frames(time = 200) # 让新的设置生效

#时间初始化，用于给图片命名
rtc=RTC()

# get image size
img_curr = sensor.snapshot()
img_size = [img_curr.height(), img_curr.width()]  # 480 640
img_fov_deg = [55.6, 70.8]

video_folder = 'video_record_1011'

uart = UART(3, 3000000)

clock = time.clock() # 跟踪FPS帧率

flag_record_image = False
flag_record_save = False

now = time.ticks_ms()

step = 1 # used for update RC input

while(True):
    clock.tick()                        # 更新 FPS 时钟.
    step = step + 1

    # -----------------获取原始图像-------------------------------------
    img_curr = sensor.snapshot()        # 获取图像 目前是480 * 640

    if flag_record_image:
        m.add_frame(img_curr, quality=100)  # 存储mjpeg文件
        blue_led.toggle()

    # -----------------check if change record mode-------------------
    if step >= 50:
        get_rc_force()
        step = 1

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
            # start recording
            flag_record_image = True

            dateTime = rtc.datetime()
            hour = '%02d' % dateTime[4]
            minute = '%02d' % dateTime[5]
            second = '%02d' % dateTime[6]
            m = mjpeg.Mjpeg(video_folder + '/original_{}_{}_{}.mjpeg'.format(hour, minute, second))

            red_led.off()
            blue_led.on()
    elif rc_7 == 1934:
        if rc_7_last == 1514:
            # end recording
            flag_record_save = True
            red_led.off()
            blue_led.off()
    else:
        red_led.toggle()

    rc_7_last = rc_7

    # ----------------check if save videos-----------------------------
    if flag_record_save:
        m.close(fps)
        flag_record_image = False
        flag_record_save = False
        print('video saved')

    fps = clock.fps()
    print('data: ', rc_7, 'FPS: ', fps, 'time: ', time.ticks_ms() - now,
          ' Free: ' + str(gc.mem_free()), 'step', step)
    now = time.ticks_ms()


print('finish')
