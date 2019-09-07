import cv2
import numpy as np
import socket
import struct
import pickle

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
   
def main():
    global frame, mouse_pressed, bb
    
    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame', on_mouse)
    
    # with pyrs.Service() as serv:
        # with serv.Device(0, streams) as dev:
    
    was_pressed = False
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('127.0.0.1', 7171))
    sock.settimeout(0.1)
    
    READ_INFO = 1
    READ_FRAME = 2
    
    data = b''
    frame_length = 0
    state = READ_INFO
    frame = None
    bb = None
    
    pid_x_conf = [0.01, 0, 0]
    pid_y_conf = [0.01, 0, 0]
    
    while True:
        try:
            data += sock.recv(8*1024)
        except socket.timeout:
            pass
        
        #print(state, len(data))
        
        if READ_INFO == state:
            if len(data) > 36:
                (frame_length, *tracker_bb, control_x, control_y) = struct.unpack('>LLLLLdd', data[:36])
                print(frame_length, tracker_bb, control_x, control_y)
                data = data[36:]
                state = READ_FRAME
        if READ_FRAME == state:
            if len(data) >= frame_length:
                frame_data = data[:frame_length]
                frame = pickle.loads(frame_data, fix_imports=True, encoding='bytes')
                frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

                data = data[frame_length:]
                state = READ_INFO        
        
        if frame is not None:
            disp_im = frame.copy()
            
            if mouse_pressed:
                bb_p1 = (int(bb[0]), int(bb[1]))
                bb_p2 = (int(bb[0] + bb[2]), int(bb[1] + bb[3]))
                cv2.rectangle(disp_im, bb_p1, bb_p2, (255,0,0), 2)
            if not mouse_pressed and was_pressed:
                sock.sendall(struct.pack('>LLLLdddddd', int(bb[0]), int(bb[1]), int(bb[2]), int(bb[3]), \
                                                        pid_x_conf[0], pid_x_conf[1], pid_x_conf[2],
                                                        pid_y_conf[0], pid_y_conf[1], pid_y_conf[2]))
                
            if tracker_bb != [0,0,0,0]:                
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
                im_print(disp_im, (0, 20), 'control=({:.3f},{:.3f})'.format(control_x, control_y), (255,255,0), 0.5, 2)
                
            was_pressed = mouse_pressed
        
            cv2.imshow('frame', disp_im)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
if '__main__' == __name__:
    main()