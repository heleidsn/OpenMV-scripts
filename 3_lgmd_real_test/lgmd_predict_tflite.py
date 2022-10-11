# Hello World Example
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

import sensor, image, time
import tf
import time
# import microlite

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

model_name = 'model.tflite'
net = tf.load(model_name)

img = image.Image(1, 13, sensor.GRAYSCALE)
print(img)

test = net.classify(img)

print(test)
print(test[0][4][0])

i = 0

while(True):
    clock.tick()                    # Update the FPS clock.
    img = image.Image(1, 13, sensor.GRAYSCALE)
    img.set_pixel(0, 10, i)
    print(img, i)
    i = i + 1
    if i > 255:
        i = 255
    test = net.classify(img)

    time.sleep_ms(200)
    print(test[0][4][0])
    print(clock.fps())              # Note: OpenMV Cam runs about half as fast when connected


