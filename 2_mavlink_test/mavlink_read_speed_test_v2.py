# Untitled - By: helei - Fri May 20 2022
# openmv serial read

import sensor, image, time, mjpeg, pyb, gc, struct, math
from pyb import Pin, Timer, LED, RTC, ExtInt
from pyb import UART


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

def get_pose_force():
    global uart
    global pose_and_yaw
    # print('start_get_pose_force')

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
            if MID == 32:
                # check payload lenght. Only unpack correct massages.
                if len(PAYLOAD)==28:
                    pose_and_yaw[0] = struct.unpack('f', PAYLOAD[4:8])[0]
                    pose_and_yaw[1] = struct.unpack('f', PAYLOAD[8:12])[0]
                    pose_and_yaw[2] = struct.unpack('f', PAYLOAD[12:16])[0]
                    done = True

    return pose_and_yaw

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



uart = UART(3, 3000000)
pose_and_yaw = [0, 0, 0, 0]

now = time.ticks_ms()

roll_cmd = 0.0

clock = time.clock() # 跟踪FPS帧率

while(True):
    clock.tick()

    attitude_rad = get_attitude_force()  # 单独force更新能到80HZ 带上获取图像46Hz 带上
    # pose_and_yaw = get_pose_force()

    print('time: ', time.ticks_ms() - now)
    now = time.ticks_ms()
