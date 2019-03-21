import numpy as np
import cv2
import pyrealsense as pyrs
from pyrealsense import offline
import os
import datetime

Show_Streams = False

Base_Directory = '/media/aero/APOLONIUSZ'

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
    }
}

def main():
    with pyrs.Service() as serv:
        with serv.Device() as dev:
            
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
                dev.wait_for_frames()
                
                color = cv2.cvtColor(dev.color, cv2.COLOR_RGB2BGR)
                stream_writers['color'].write(color)
                
                depth = np.uint8(dev.depth / 256.)
                depth = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
                stream_writers['depth'].write(depth)
            
                if Show_Streams:
                   cv2.imshow('color', color)
                   cv2.imshow('depth', depth)

                   if cv2.waitKey(1) & 0xFF == ord('q'):
                       break

            for writer in stream_writers:
                stream_writers[writer].release()

if __name__ == '__main__':
    main()
