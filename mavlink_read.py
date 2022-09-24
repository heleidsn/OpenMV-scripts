# Untitled - By: helei - Fri May 20 2022
# openmv serial read

import sensor, image, time, struct

from pyb import UART

uart = UART(3, 921600)


now = time.ticks_ms()

roll_cmd = 0.0

while(True):
    data = uart.read()
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
        if MID == 30:

            roll_rad = struct.unpack('f', PAYLOAD[4:8])[0]


            roll_cmd += 0.01
            output_str = "R%f" % roll_cmd
            uart.write(output_str)

            print('roll: ', roll_rad * 57.3, 'time: ', time.ticks_ms() - now, 'sent: ', output_str)
            now = time.ticks_ms()
    '''
