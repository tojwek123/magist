import cv2
import numpy as np
import pyrealsense as pyrs
import time
import math
from basic_odometry import Basic_Odometry


def get_fps():
    current_time = time.time()
    fps = 1 / (current_time - get_fps.last_time)
    get_fps.last_time = current_time
    return fps
get_fps.last_time = 0

def get_frames(dev):
    dev.wait_for_frames()
    color = cv2.cvtColor(dev.color, cv2.COLOR_RGB2BGR)
    depth = dev.depth * dev.depth_scale
    
    return color, depth

def limit_value(value, lower_bound, upper_bound):
    if value < lower_bound:
        value = lower_bound
    elif value > upper_bound:
        value = upper_bound
    return value

def main():
    with pyrs.Service() as serv:
        with serv.Device() as dev:

            #Create Feature Detector and Matcher
            feature_detector = cv2.ORB_create()
            matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
           
            odometry = Basic_Odometry(feature_detector, matcher)            

            #Variable for position
            position = {
                'roll' : 0,
                'pitch': 0,
                'yaw'  : 0,
                'x'    : 0,
                'y'    : 0,
                'z'    : 0,
            }        

            #Create window and callback for position preview
            cv2.namedWindow('Roll and z')
            
            def roll_and_z_preview_mouse_cb(event, x, y, flags, param):
                if event == cv2.EVENT_LBUTTONDOWN:
                    print('Resetting roll and z')
                    position['roll'] = 0
                    position['z']    = 0

            cv2.setMouseCallback('Roll and z', roll_and_z_preview_mouse_cb)

            #bottom_cap = cv2.VideoCapture(14)
            #bottom_odometry = Basic_Odometry(feature_detector, matcher)

            horizontal = 0

            while True:
                #Store FPS value to check performance
                fps = get_fps()

                #Get new frames and detect features
                color, depth = get_frames(dev)
                #ret, bottom = bottom_cap.read()
                #cv2.imshow('bottom', bottom)
                #bottom_odometry.get_displacement(bottom)
                
                cv2.imshow('depth', depth / 64.)

                #Display frames
                color_preview = color.copy()
                cv2.putText(color_preview, '{:.2f} FPS'.format(fps), (0,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
                cv2.imshow('Color', color_preview)

                displacement = odometry.get_displacement(color, depth)
                            
                if displacement is not None:
                    position['roll'] += displacement['roll']
                    position['z']    += displacement['vertical']
                    horizontal += displacement['horizontal']
                    print(horizontal)                  
                   
                #Display roll and z visualization
                scene_size = (640, 480)
                roll_and_z_preview = np.ones((scene_size[1], scene_size[0], 3), dtype=np.uint8) * 255
                z_pos = position['z'] + scene_size[1] / 2
                cv2.circle(roll_and_z_preview, (scene_size[0]//2, int(z_pos)), 3, (0,0,255), 1, -1)
                a = (scene_size[0] / 2) * math.tan(position['roll'])
                
                roll_pt1 = (0            , int(scene_size[1]/2 + a))
                roll_pt2 = (scene_size[0], int(scene_size[1]/2 - a))
                cv2.line(roll_and_z_preview, roll_pt1, roll_pt2, (255,0,0), 1)

                cv2.imshow('Roll and z', roll_and_z_preview)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        

if __name__ == '__main__':
    main()
