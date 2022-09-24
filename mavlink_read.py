# Untitled - By: helei - Fri May 20 2022
# openmv serial read

import sensor, image, time, struct

from pyb import UART

uart = UART(3, 921600)


now = time.ticks_ms()

roll_cmd = 0.0

while(True):
    data = uart.read()
    '''
    if data is not None:
        print(data)
    '''

    if data is not None:
        LEN = int.from_bytes(data[1:2], "big")
        SEQ = int.from_bytes(data[2:3], "big")
        SID = int.from_bytes(data[3:4], "big")
        CID = int.from_bytes(data[4:5], "big")
        MID = int.from_bytes(data[5:6], "big")
        PAYLOAD = data[6:LEN+6]
        if MID == 32:
            # check payload lenght. Only unpack correct massages.
            if len(PAYLOAD)==28:  # 长度28，包含一个uint32(4)和6个float（4）
                x = struct.unpack('f', PAYLOAD[4:8])[0]
                y = struct.unpack('f', PAYLOAD[8:12])[0]
                z = struct.unpack('f', PAYLOAD[12:16])[0]
            else:
                x = None
                y = None
                z = None

            # print('1')

            print("x:", x, 'y:', y, 'z:', z, 'time: ', time.ticks_ms() - now)

            now = time.ticks_ms()

