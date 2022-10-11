# 保存视频例程
#
# 注意：您将需要SD卡来运行此演示。
#
# 您可以使用OpenMV Cam来录制mjpeg文件。 您可以为记录器对象提供JPEG帧
# 或RGB565 /灰度帧。 一旦你完成了一个Mjpeg文件的录制，你可以使用VLC
# 来播放它。 如果你在Ubuntu上，那么内置的视频播放器也可以工作。
import sensor, image, time, mjpeg, pyb
from pyb import Pin, Timer, LED, RTC, ExtInt

RED_LED_PIN = 1
BLUE_LED_PIN = 3

sensor.reset() # 初始化sensor

sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.GRAYSCALE
#设置图像色彩格式，有RGB565色彩图和GRAYSCALE灰度图两种
sensor.set_vflip(True)
sensor.set_hmirror(True)

sensor.set_framesize(sensor.VGA) # or sensor.QQVGA (or others)
#设置图像像素大小

sensor.skip_frames(time = 2000) # 让新的设置生效


#时间初始化，用于给图片命名
rtc=RTC()

##############################################
#  按键和其回调函数
##############################################
# KEY = Pin('P0', Pin.IN, Pin.PULL_UP)   # 按键
blue_led  = LED(3)                     # 蓝色指示灯
red_led = LED(1)
key_node = False  #按键标志位

KEY = Pin('P0', Pin.IN, Pin.PULL_UP)
clock = time.clock() # 跟踪FPS帧率

# 主循环
recording = False
while(True):
    # 延迟按键，按下1s后触发
    key_press_num = 0
    red_led.off()
    while(KEY.value()==0 and key_press_num<10): #按键被按下接地
        time.sleep_ms(100)
        key_press_num += 1
        red_led.on()

    red_led.off()
    if key_press_num >= 10:
        blue_led.on()
        recording = True
        time.sleep_ms(1000)

        m = mjpeg.Mjpeg(str(rtc.datetime()) + "test_VGA.mjpeg")


    while(recording):

        # 录制视频
        clock.tick()
        m.add_frame(sensor.snapshot(), quality=100)
        blue_led.toggle()
        fps = clock.fps()
        print(fps)

        key_press_num = 0
        while(KEY.value()==0 and key_press_num < 10): #按键被按下接地
            time.sleep_ms(100)
            key_press_num += 1
            red_led.on()
        red_led.off()
        if key_press_num >= 10:
            blue_led.off()
            recording = False
            m.close(fps)
            print(fps)
            print('recording end')



# --------------video_record_old------------------------
m = mjpeg.Mjpeg("FW_image_record_300s_VGA.mjpeg")

clock = time.clock() # 跟踪FPS帧率

time_total = 300  # 录制时间， 秒
frequence = 42   # 录制频率， HZ

step = int(time_total * frequence)

for i in range(step):
    clock.tick()
    m.add_frame(sensor.snapshot(), quality=100)
    blue_led.toggle()
    # time.sleep_ms(int(1000/frequence))
    print(clock.fps())

# m.close(frequence)
m.close(clock.fps())

blue_led.off()

print("Done! Reset the camera to see the saved recording.")
