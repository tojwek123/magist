import cv2
import numpy as np

mouse_pressed = False
bb = [0,0,0,0]

#MAV_CMD_CONDITION_YAW 

def on_mouse(event, x, y, flags, param):
    global frame, mouse_pressed, bb
    
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_pressed = True
        on_mouse.x_start, on_mouse.y_start = x, y
    elif event == cv2.EVENT_LBUTTONUP:
        mouse_pressed = False           
    
    x_start, y_start = on_mouse.x_start, on_mouse.y_start
    x_end, y_end = x, y
            
    if y_start > y_end:
        y_start, y_end = y_end, y_start
    if x_start > x_end:
        x_start, x_end = x_end, x_start
            
    bb = [x_start, y_start, x_end - x_start, y_end - y_start]
on_mouse.x_start = 0
on_mouse.y_start = 0
        
def im_print(im, pt, text, color, scale, thickness):
    cv2.putText(im, text, (pt[0] - 1, pt[1]), cv2.FONT_HERSHEY_SIMPLEX, scale, (0,0,0), 2)
    cv2.putText(im, text, (pt[0] + 1, pt[1]), cv2.FONT_HERSHEY_SIMPLEX, scale, (0,0,0), 2)
    cv2.putText(im, text, (pt[0], pt[1] - 1), cv2.FONT_HERSHEY_SIMPLEX, scale, (0,0,0), 2)
    cv2.putText(im, text, (pt[0], pt[1] + 1), cv2.FONT_HERSHEY_SIMPLEX, scale, (0,0,0), 2)
    cv2.putText(im, text, pt, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness)
   
class PID:
    def __init__(self, kp=0, ki=0, kd=0):
        self._error_sum = 0
        self._last_error = 0
        self._kp = kp
        self._ki = ki
        self._kd = kd
        
    def set_parameters(self, kp, ki=0, kd=0):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        
    def clear(self):
        self._error_sum = 0
        self._last_error = 0
        
    def feed(self, error):
        self._error_sum += error
        control = (self._kp * error) + (self._ki * self._error_sum) + (self._kd * (error - self._last_error))
        self._last_error = error
        return control
   
def main():
    global frame, mouse_pressed, bb
    
    cap = cv2.VideoCapture(0)
    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame', on_mouse)
    
    # with pyrs.Service() as serv:
        # with serv.Device(0, streams) as dev:
    
    was_pressed = False
    tracker = None
    pid_x = PID(0.01, 0)
    pid_y = PID(0.01, 0)
    
    while True:
        ret, frame = cap.read()
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # dev.wait_for_frames()
        
        disp_im = frame.copy()
        
        
        if mouse_pressed:
            bb_p1 = (int(bb[0]), int(bb[1]))
            bb_p2 = (int(bb[0] + bb[2]), int(bb[1] + bb[3]))
            cv2.rectangle(disp_im, bb_p1, bb_p2, (255,0,0), 2)
            
        if not mouse_pressed and was_pressed:
            tracker = cv2.TrackerMedianFlow_create()
            tracker.init(frame, tuple(bb))
            pid_x.clear()
            pid_y.clear()
            
        if tracker is not None:
            detected, tracker_bb = tracker.update(frame)
            
            if detected:
                tracker_bb_p1 = (int(tracker_bb[0]), int(tracker_bb[1]))
                tracker_bb_p2 = (int(tracker_bb[0] + tracker_bb[2]), int(tracker_bb[1] + tracker_bb[3]))
                cv2.rectangle(disp_im, tracker_bb_p1, tracker_bb_p2, (0,255,0), 2)
                
                bb_center = (int(tracker_bb[0] + tracker_bb[2]/2), int(tracker_bb[1] + tracker_bb[3]/2))
                frame_center = (frame.shape[1]//2, frame.shape[0]//2)
                
                cv2.circle(disp_im, bb_center, 3, (0,0,0), -1)
                cv2.circle(disp_im, frame_center, 3, (0,0,0), -1)
                cv2.line(disp_im, bb_center, frame_center, (0,0,255), 2)
                
                diff_x = -(frame_center[0] - bb_center[0])
                diff_y = frame_center[1] - bb_center[1]
            
                im_print(disp_im, bb_center, 'diff=({}, {})'.format(diff_x, diff_y), (0,0,255), 0.5, 2)
            
                control_x = pid_x.feed(diff_x)
                control_y = pid_y.feed(diff_y)
                im_print(disp_im, (0, 20), 'control=({:.3f},{:.3f})'.format(control_x, control_y), (255,255,0), 0.5, 2)
            
        cv2.imshow('frame', disp_im)
        
        was_pressed = mouse_pressed
    
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
if '__main__' == __name__:
    main()