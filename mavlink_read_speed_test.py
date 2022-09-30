# Untitled - By: helei - Fri May 20 2022
# openmv serial read

import sensor, image, time, mjpeg, pyb, gc, struct, math
from pyb import Pin, Timer, LED, RTC, ExtInt
from pyb import UART

uart = UART(3, 3000000)


now = time.ticks_ms()

roll_cmd = 0.0

clock = time.clock() # 跟踪FPS帧率

while(True):
    clock.tick()
    data = uart.read()

    if data is not None:
        LEN = int.from_bytes(data[1:2], "big")
        SEQ = int.from_bytes(data[2:3], "big")
        SID = int.from_bytes(data[3:4], "big")
        CID = int.from_bytes(data[4:5], "big")
        MID = int.from_bytes(data[5:6], "big")
        PAYLOAD = data[6:LEN+6]

        if MID == 30:
            # if len(PAYLOAD)==42:
            print('time: ', time.ticks_ms() - now)
            now = time.ticks_ms()
