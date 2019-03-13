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

def get_displacements(kp1, kp2, matches):
    displacements = np.zeros((len(matches), 3), dtype=np.float32)
    for i, match in enumerate(matches):
        pt1 = kp1[match.queryIdx].pt
        pt2 = kp2[match.trainIdx].pt
        
        displacements[i][0] = pt1[0] - pt2[0]
        displacements[i][1] = pt1[1] - pt2[1]       
        displacements[i][2] = ((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)**(1/2)

    return displacements 

def main():
    with pyrs.Service() as serv:
        with serv.Device() as dev:

            feature_detector = cv2.ORB_create()
            matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            
            overall_displacement = [0, 0]  
            overall_angle = 0
            
            last_color, last_depth = get_frames(dev)
            last_kp, last_des = feature_detector.detectAndCompute(last_color, None)
            
            while True:
                fps = get_fps()

                color, depth = get_frames(dev)
                cv2.imshow('color', color)
                kp, des = feature_detector.detectAndCompute(color, None)
                
                if des is not None and last_des is not None:
                    matches = matcher.match(des, last_des)
                    matches = sorted(matches, key=lambda x: x.distance)
                    
                    #Get best X percentage of results
                    matches = matches[:int(0.1 * len(matches))]
                    
                    pt = []
                    last_pt = []

                    for match in matches:
                        pt.append(kp[match.queryIdx].pt)
                        last_pt.append(last_kp[match.trainIdx].pt)

                    pt = np.array(pt)
                    last_pt = np.array(last_pt)

                    if len(matches) == 0:   
                        continue

                    res = cv2.estimateRigidTransform(pt, last_pt, False, 500, 0.5, 3)
                    #print(res)     

                    
                    if res is not None:                   
                        angle = math.atan2(-res[0][1], res[0][0])
                        overall_angle += angle
                        print(overall_angle * 180 / math.pi)

                        overall_displacement[0] += res[0][2]
                        overall_displacement[1] += res[1][2]                   

                    if overall_displacement[0] > 300:
                        overall_displacement[0] = 300
                    elif overall_displacement[0] < -300:
                        overall_displacement[0] = -300
                    if overall_displacement[1] > 300:
                        overall_displacement[1] = 300
                    elif overall_displacement[1] < -300:
                        overall_displacement[1] = -300

                    visualization = np.ones((600, 600, 3), dtype=np.uint8) * 255
                    cv2.circle(visualization, (int(overall_displacement[0] + 300), int(overall_displacement[1] + 300)), 3, (0,0,255), 1, -1)
                    cv2.imshow('vis', visualization)

                    preview = cv2.drawMatches(color, kp, last_color, last_kp, matches, None, flags=2)

                    #color = cv2.drawKeypoints(color, kp, None, (255,0,0), 4)
                    
                    cv2.putText(preview, '{:.2f} FPS'.format(fps), (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))

                    cv2.imshow('preview', preview)
                
                last_color, last_depth = color, depth
                last_kp, last_des = kp, des
                    
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        

if __name__ == '__main__':
    main()
