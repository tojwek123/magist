import numpy as np
import cv2
import pyrealsense as pyrs
from pyrealsense import offline
import os
import datetime

import ctypes
from pyrealsense.stream import * 
from pyrealsense.constants import rs_stream, rs_format

Show_Streams = False

Base_Directory = '/home/aero/flying-tests/record-data'

Streams = {
    'color': {
        'fourcc': 'XVID',
        'fps': 30,
        'size': (640, 480)
    },
    'depth': {
        'fourcc': 'XVID',
        'fps': 30,
        'size': (640, 480)
    },
    'infrared': {
        'fourcc': 'XVID',
        'fps': 30,
        'size': (640, 480)
    },
    'infrared2': {
        'fourcc': 'XVID',
        'fps': 30,
        'size': (640, 480)
    }
}

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

    with pyrs.Service() as serv:
        with serv.Device(0, streams) as dev:
            
            current_time_str = '{0:%Y-%m-%d_%H-%M-%S}'.format(datetime.datetime.now()) 
            write_directory = os.path.join(Base_Directory, current_time_str) 

            try:
            	os.makedirs(write_directory)            
            except FileExistsError:
                pass

            stream_writers = {}

            for stream in Streams:
                path = os.path.join(write_directory, stream + '.avi')
                fourcc = cv2.VideoWriter_fourcc(*Streams[stream]['fourcc'])
                writer = cv2.VideoWriter(path, fourcc, Streams[stream]['fps'], Streams[stream]['size'])
                stream_writers[stream] = writer

            print('Saving data to "{}"'.format(write_directory))
            
            offline.save_depth_intrinsics(dev, fileloc=write_directory) 
           
            while True:
                try:
                    dev.wait_for_frames()
		
                    color = cv2.cvtColor(dev.color, cv2.COLOR_RGB2BGR)
                    stream_writers['color'].write(color)
                    
                    depth = np.uint8(dev.depth / 256.)
                    depth = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
                    stream_writers['depth'].write(depth)
               
                    infrared = cv2.cvtColor(dev.infrared, cv2.COLOR_GRAY2BGR)
                    stream_writers['infrared'].write(infrared)
                    
                    infrared2 = cv2.cvtColor(dev.infrared2, cv2.COLOR_GRAY2BGR)
                    stream_writers['infrared2'].write(infrared2)   
                    
                    if Show_Streams:
                       cv2.imshow('color', color)
                       cv2.imshow('depth', depth)
                       cv2.imshow('infrared', infrared)
                       cv2.imshow('infrared2', infrared2)

                       if cv2.waitKey(1) & 0xFF == ord('q'):
                           break
                except KeyboardInterrupt:
                    print('Keyboard interrupt - stopping...')
                    print('')
                    break                

            for writer in stream_writers:
                stream_writers[writer].release()

if __name__ == '__main__':
    main()
