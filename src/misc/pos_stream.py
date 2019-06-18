import numpy as np
import cv2
import pyrealsense as pyrs
from pyrealsense import offline
import os
import datetime
import dronekit
import time

import ctypes
from pyrealsense.stream import * 
from pyrealsense.constants import rs_stream, rs_format

class InfraredStream2(Stream):
    """Second infrared stream from device, with default parameters.
    """
    def __init__(self, name='infrared2', width=640, height=480, fps=30):
        self.native = True
        self.stream = rs_stream.RS_STREAM_INFRARED2
        self.format = rs_format.RS_FORMAT_Y8
        self.shape = (height, width)
        self.dtype = ctypes.c_uint8
        super(InfraredStream2, self).__init__(name, self.native, self.stream, width, height, self.format, fps)

def main():
    streams = [ColorStream('color', 1920, 1080, 15), DepthStream(), InfraredStream(), InfraredStream2()]
        
    global xacc
    xacc = 0
    
    def on_msg(vehicle, name, msg):
        global xacc
        xacc = msg.xacc
        #print('{} | {}\n'.format(msg, vehicle.attitude))
        #print(vehicle.attitude.yaw)
        
    vehicle = dronekit.connect('tcp:127.0.0.1:5760')
    vehicle.add_message_listener('HIGHRES_IMU', on_msg)
        
    xacc_offset = 0
    calib_iter = 1000
        
    for i in range(calib_iter):
        xacc_offset += xacc
        time.sleep(0.001)
        
    xacc_offset /= calib_iter
    
    print('Offset: {}'.format(xacc_offset))
    
    window = [0] * 20
    
    last_acc = 0
    sample_time = 0.001
        
    cnt = 0
    v = 0
        
    while True:
        window = window[1:-1]
        window.append(xacc - xacc_offset)
        
        acc = sum(window) / len(window)
        v += (last_acc + acc) * sample_time / 2
        last_acc = acc
        
        cnt += 1
        if cnt % 100 == 0:
            print(acc, v)
        time.sleep(sample_time)
    
    # with pyrs.Service() as serv:
        # with serv.Device(0, streams) as dev:
            # vehicle = dronekit.connect('tcp:127.0.0.1:5760')
            # vehicle.add_message_listener('HIGHRES_IMU', on_msg)

            # stream_writers = {}
                           
            # while True:
                # try:
                    # dev.wait_for_frames()
		
                    # color = cv2.cvtColor(dev.color, cv2.COLOR_RGB2BGR)
                    
                    # depth = np.uint8(dev.depth / 256.)
                    # depth = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
               
                    # infrared = cv2.cvtColor(dev.infrared, cv2.COLOR_GRAY2BGR)
                    
                    # infrared2 = cv2.cvtColor(dev.infrared2, cv2.COLOR_GRAY2BGR)
                    

                   # if cv2.waitKey(1) & 0xFF == ord('q'):
                       # break
                # except KeyboardInterrupt:
                    # print('Keyboard interrupt - stopping...')
                    # print('')
                    # break                

            # vehicle.remove_message_listener('HIGHRES_IMU', on_msg)

if __name__ == '__main__':
    main()
