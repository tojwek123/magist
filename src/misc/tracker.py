import cv2
import numpy as np
import pyrealsense as pyrs

rect_start = (0, 0)
rect_end = (0, 0)
area_selected = False
mouse_pressed = False

def on_mouse(event, x, y, flags, param):
    global color, depth, area_selected, rect_start, rect_end, mouse_pressed
    
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_pressed = True
        area_selected = False
        rect_start = (x, y)
        rect_end = (x, y)
    elif event == cv2.EVENT_MOUSEMOVE:
        if mouse_pressed:
            disp_color = color.copy()
            rect_end = (x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        mouse_pressed = False
        rect_end = (x, y)
        if rect_end != rect_start:
            area_selected = True

       
def main():
    global color, depth, area_selected, rect_start, rect_end

    cv2.namedWindow('color')
    cv2.setMouseCallback('color', on_mouse)
    
    tracker_initialized = False
    with pyrs.Service() as serv:
        with serv.Device(0) as dev:
            while True:
                dev.wait_for_frames()
                color = cv2.cvtColor(dev.color, cv2.COLOR_RGB2BGR)
                dev.depth[dev.depth > 10000] = 10000
                depth = np.uint8(dev.depth / np.max(dev.depth) * 255)           
                disp_color = color.copy()

                if area_selected:
                    if not tracker_initialized:
                        tracker = cv2.TrackerMedianFlow_create()
                        bbox = [0] * 4
                        if rect_start[0] > rect_end[0]:
                            bbox[0] = rect_end[0]
                        else:
                            bbox[0] = rect_start[0]
                        if rect_start[1] > rect_end[1]:
                            bbox[1] = rect_end[1]
                        else:
                            bbox[1] = rect_start[1]
                        bbox[2] = abs(rect_end[0] - rect_start[0])
                        bbox[3] = abs(rect_end[1] - rect_start[1])
                        tracker.init(color, tuple(bbox))
                        tracker_initialized = True                
                    else:
                        ret, bbox = tracker.update(color)
                   
                    p1 = (int(bbox[0]), int(bbox[1]))
                    p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                    cv2.rectangle(disp_color, p1, p2, (255,0,0), 2)
                    #markers = np.ones(color.shape[:2], dtype=np.int32)
                    #cv2.rectangle(markers, rect_start, rect_end, 0, -1)
                    #markers[rect_end[1] - (rect_end[1] - rect_start[1])//2][rect_end[0] - (rect_end[0] - rect_start[0])//2] = 2
                    
                    #preproc_color = color.copy()
                    #preproc_color = cv2.GaussianBlur(preproc_color, (33,33), 1)
                    #cv2.imshow('preproc', preproc_color)
                    #preproc_depth = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
                    #preproc_depth = cv2.GaussianBlur(preproc_depth, (33, 33), 1)
                    #cv2.watershed(preproc_depth, markers)
                    
                    #cv2.imshow('preproc_depth', preproc_depth) 
                    #mask = np.zeros(color.shape[:2], dtype=np.uint8)
                    #mask[markers == 2] = 255
                    #cv2.imshow("mask", mask)
                    #moments = cv2.moments(mask)
                    #center = (int(moments["m10"]/moments["m00"]), int(moments["m01"]/moments["m00"]))
                    
                    #disp_color[markers == 2] = [0,0,255]
                    #cv2.circle(disp_color, center, 3, (255, 0, 0), -1)
                elif mouse_pressed:
                    cv2.rectangle(disp_color, rect_start, rect_end, (0,0,255), 1)
                    tracker_initialized = False
            
                cv2.imshow('color', disp_color)
                cv2.imshow('depth', depth)
 
                if ord('q') == cv2.waitKey(1):
                    break
    
if __name__ == '__main__':
    main()
