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
    streams = [ColorStream(), DepthStream(), InfraredStream(), InfraredStream2()]
        
    #vehicle = dronekit.connect('tcp:127.0.0.1:5760')

    global threshold
    global color
    global depth    

    threshold = 0


    def on_mouse(event, x, y, flags, apram):
        global threshold
        global color
        global depth
        
        if event == cv2.EVENT_LBUTTONDOWN:
            threshold = depth[y][x]
            print(threshold)
    
    cv2.namedWindow('color')
    cv2.setMouseCallback('color', on_mouse)    

    with pyrs.Service() as serv:
        with serv.Device(0, streams) as dev:
            while True:
                try:
                    dev.wait_for_frames()
		
                    color = cv2.cvtColor(dev.color, cv2.COLOR_RGB2BGR)
                    
                    depth = np.uint8(dev.depth / 256)
               
                    infrared = cv2.cvtColor(dev.infrared, cv2.COLOR_GRAY2BGR)
                  
                    infrared2 = cv2.cvtColor(dev.infrared2, cv2.COLOR_GRAY2BGR)
                    
                    cv2.imshow('depth', depth)
                    cv2.imshow('color', color)

                    lower_bound = threshold - 1
                    if lower_bound < 1:
                        lower_bound = 1

                    upper_bound = threshold + 1

                    bw = cv2.inRange(depth, lower_bound, upper_bound)
                    cv2.imshow('bw', bw)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                       break
                except KeyboardInterrupt:
                    print('Keyboard interrupt - stopping...')
                    print('')
                    break                

if __name__ == '__main__':
    main()
