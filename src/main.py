import cv2
import numpy as np
import pyrealsense as pyrs
import time
import math

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

def get_points_from_matches(kp2, kp1, matches):
    pts2, pts1 = [], []
    
    for match in matches:
        pts2.append(kp2[match.queryIdx].pt)
        pts1.append(kp1[match.trainIdx].pt)

    return np.array(pts2), np.array(pts1)

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
 
            #Data from previous match
            color1, depth1 = None, None
            kp1, des1 = None, None
            
            while True:
                #Store FPS value to check performance
                fps = get_fps()

                #Get new frames and detect features
                color2, depth2 = get_frames(dev)
                kp2, des2 = feature_detector.detectAndCompute(color2, None)
               
                #Display frames
                color_preview = color2.copy()
                cv2.putText(color_preview, '{:.2f} FPS'.format(fps), (0,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
                cv2.imshow('Color', color_preview)

 
                #Process only if descriptors are not None
                if des2 is not None and des1 is not None:

                    #Match points from 2 frames and sort them by distance (score)
                    matches = matcher.match(des2, des1)
                    matches = sorted(matches, key=lambda x: x.distance)
                    
                    #Get best X percentage of results
                    matches = matches[:int(0.05 * len(matches))]
 
                    #Process further only if there are some matches
                    if len(matches) > 0:   
                        pt2, pt1 = get_points_from_matches(kp2, kp1, matches)
                        tfMat = cv2.estimateRigidTransform(pt2, pt1, False, 500, 0.5, 3)
                        
                        if tfMat is not None:
                            a11, a12, b1 =  tfMat[0][0], tfMat[0][1], tfMat[0][2]
                            a12, a11, b2 = -tfMat[1][0], tfMat[1][1], tfMat[1][2]              
                            
                            #Calculate position change and update overall position
                            d_roll = math.atan2(-a12, a11)
                            d_z    = b2
                            
                            position['roll'] += d_roll
                            position['z']    += d_z
                    
                    #Display matches
                    matches_preview = cv2.drawMatches(color2, kp2, color1, kp1, matches, None, flags=2)
                    cv2.imshow('Matches', matches_preview)

                   
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

                #        visualization = np.ones((600, 600, 3), dtype=np.uint8) * 255
                #        cv2.circle(visualization, (int(overall_displacement[0] + 300), int(overall_displacement[1] + 300)), 3, (0,0,255), 1, -1)
                #       cv2.imshow('vis', visualization)

                #Save current frame only if there was any keypoint 
                if des2 is not None:
                    color1, depth1 = color2, depth2
                    kp1, des1 = kp2, des2
                    
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        

if __name__ == '__main__':
    main()
