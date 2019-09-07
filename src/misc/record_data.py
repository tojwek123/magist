import numpy as np
import cv2
import pyrealsense as pyrs
from pyrealsense import offline
import os
import datetime
import dronekit
import sys

import ctypes
from pyrealsense.stream import * 
from pyrealsense.constants import rs_stream, rs_format

Show_Streams = False
Video_As_Png = False
Base_Directory = '/home/aero/flying-tests/record-data'
Rpi_Addr = ''
Rpi_Lite_Udp_Port = 6969
Rpi_Sweep_Udp_Port = 6868

Streams = {
    'color': {
        'fourcc': 'XVID',
        'fps': 15,
        'size': (1920, 1080)
    },
    'depth': {
        'fourcc': 'XVID',
        'fps': 15,
        'size': (640, 480)
    },
    'infrared': {
        'fourcc': 'XVID',
        'fps': 15,
        'size': (640, 480)
    },
    'infrared2': {
        'fourcc': 'XVID',
        'fps': 15,
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

imu_file = None
lite_file = None
sweep_file = None
stop_lite = False
stop_sweep = False

def on_msg(vehicle, name, msg):
    global imu_file
    imu_file.write('{} | {}\n'.format(msg, vehicle.attitude))

def on_lite_reading(distance):
    global lite_file
    lite_file.write('{}: {} cm\n'.format(time.time(), distance_cm))
       
def on_sweep_reading(scan):
    global sweep_file
    sweep_file.write('{}: {}\n'.format(time.time(), scan))

def on_lite_error(error):
    print('Lite error: {}'.format(error))

def on_sweep_error(error):
    print('Sweep error: {}'.format(error))
        
def sweep_reader():
    global stop_sweep
    global sweep_file

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((Rpi_Addr, Rpi_Sweep_Udp_Port))
    
    while not stop_sweep:
        data, addr = sock.recvfrom(32 * 1024)
        samples = []
        
        for i in range(len(data) / 7):
            sample = struct.unpack('>LHB', data[:7])
            samples.append(sample)
            data = data[7:]
        
        lite_file.write('{}: {}'.format(time.time(), samples))
        
def main():
    record_all = len(sys.argv) == 1
    record_color = 'color' in sys.argv or record_all
    record_depth = 'depth' in sys.argv or record_all
    record_infrared = 'infrared' in sys.argv or record_all
    record_infrared2 = 'infrared2' in sys.argv or record_all
    record_imu = 'imu' in sys.argv or record_all
    record_sweep = 'sweep' in sys.argv or record_all
    record_lite = 'lite' in sys.argv or record_all

    streams = [ColorStream('color', 1920, 1080, 15), DepthStream(), InfraredStream(), InfraredStream2()]

    with pyrs.Service() as serv:
        with serv.Device(0, streams) as dev:
            
            current_time_str = '{0:%Y-%m-%d_%H-%M-%S-%f}'.format(datetime.datetime.now()) 
            write_directory = os.path.join(Base_Directory, current_time_str) 

            try:
            	os.makedirs(write_directory)            
            except FileExistsError:
                pass
            
            if record_imu:
                global imu_file
                imu_file = open(os.path.join(write_directory, 'imu.txt'), 'w')
                vehicle = dronekit.connect('tcp:127.0.0.1:5760')
                vehicle.add_message_listener('HIGHRES_IMU', on_msg)
                
            if record_sweep:
                global sweep_file
                sweep_file = open(os.path.join(write_directory, 'sweep.txt'), 'w')
                sweep_thread = threading.Thread(target=sweep_reader)
                sweep_thread.start()
                
            if record_lite:
                global lite_file
                lite_file = open(os.path.join(write_directory, 'lite.txt'), 'w')
                lite_thread = threading.Thread(target=lite_reader)
                lite_thread.start()

            if not record_color:
                del Streams['color']
            if not record_depth:
                del Streams['depth']
            if not record_infrared:
                del Streams['infrared']
            if not record_infrared2:
                del Streams['infrared2']

            if Video_As_Png:
                for stream in Streams:
                    try:
                        os.makedirs(os.path.join(write_directory, stream))       
                    except FileExistsError:
                        pass
            else:
                stream_writers = {}

                for stream in Streams:
                    path = os.path.join(write_directory, stream + '.avi')
                    fourcc = cv2.VideoWriter_fourcc(*Streams[stream]['fourcc'])
                    writer = cv2.VideoWriter(path, fourcc, Streams[stream]['fps'], Streams[stream]['size'])
                    stream_writers[stream] = writer

            print('Saving data to "{}"'.format(write_directory))
            
            offline.save_depth_intrinsics(dev, fileloc=write_directory) 
            frame_no = 0
            
            while True:
                try:
                    dev.wait_for_frames()
		
                    color = cv2.cvtColor(dev.color, cv2.COLOR_RGB2BGR)
                    if record_color:
                        if Video_As_Png:
                            cv2.imwrite(os.path.join(write_directory, 'color', str(frame_no)), color)
                        else:
                            stream_writers['color'].write(color)
                    
                    depth = np.uint8(dev.depth / 256.)
                    depth = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
                    if record_depth:
                        if Video_As_Png:
                            cv2.imwrite(os.path.join(write_directory, 'depth', str(frame_no)), depth)
                        else:
                            stream_writers['depth'].write(depth)
               
                    infrared = cv2.cvtColor(dev.infrared, cv2.COLOR_GRAY2BGR)
                    if record_infrared
                        if Video_As_Png:
                            cv2.imwrite(os.path.join(write_directory, 'infrared', str(frame_no)), infrared)
                        else:
                            stream_writers['infrared'].write(infrared)
                    
                    infrared2 = cv2.cvtColor(dev.infrared2, cv2.COLOR_GRAY2BGR)
                    if record_infrared2:
                        if Video_As_Png:
                            cv2.imwrite(os.path.join(write_directory, 'infrared2', str(frame_no)), infrared2)
                        else:
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

            if record_imu:
                vehicle.remove_message_listener('HIGHRES_IMU', on_msg)
                imu_file.close()
                
            if record_sweep:
                global stop_sweep
                stop_sweep = True
                sweep_thread.join()
                sweep_file.close()
                
            if record_lite:
                global stop_lite
                stop_lite = True
                lite_thread.join()
                lite_file.close()

            if not Video_As_Png:
                for writer in stream_writers:
                    stream_writers[writer].release()

if __name__ == '__main__':
    main()
