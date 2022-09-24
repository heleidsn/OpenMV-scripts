# Untitled - By: helei - Fri May 20 2022\
# uart2 recieve roll data and send roll_cmd


import sensor, image, time, struct

from pyb import UART


# ----------------------------settings-------------------------
uart = UART(3, 921600)

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

def send_distance_sensor_packet(roll_cmd):
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
    # print(temp)

# ---------------------------main function------------------------------------

now = time.ticks_ms()

roll_cmd = 0.0

while(True):

    # ---------------------wait for roll data----------------------------
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
                roll_rad = struct.unpack('f', PAYLOAD[4:8])[0]
            else:
                roll_rad = 0.00

            # --------------send roll_cmd data-------------------------
            roll_cmd += 0.005
            if roll_cmd > 1:
                roll_cmd = -1
            send_distance_sensor_packet(float(roll_cmd))


            print('roll: ', roll_rad * 57.3, 'time: ', time.ticks_ms() - now, 'sent: ', roll_cmd)
            now = time.ticks_ms()
