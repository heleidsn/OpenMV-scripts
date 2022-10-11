# Untitled - By: helei - Fri May 20 2022
# openmv serial read

import sensor, image, time, struct

from pyb import UART

uart = UART(3, 921600)


now = time.ticks_ms()

roll_cmd = 0.0

while(True):
    data = uart.read()

    # print(data)

    if data is not None:
        clock.tick()
        LEN = int.from_bytes(data[1:2], "big")
        SEQ = int.from_bytes(data[2:3], "big")
        SID = int.from_bytes(data[3:4], "big")
        CID = int.from_bytes(data[4:5], "big")
        MID = int.from_bytes(data[5:6], "big")
        PAYLOAD = data[6:LEN+6]

        if MID == 30:
            # print(len(PAYLOAD))
            if len(PAYLOAD)==28:
                chanel = 7
                rc_7 = struct.unpack('h', PAYLOAD[4+(chanel*2-2):4+(chanel*2)])[0]
                print(rc_7, 'time: ', time.ticks_ms() - now)
                now = time.ticks_ms()
                # print(len(PAYLOAD))

