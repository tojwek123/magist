import cv2
import numpy as np
import socket
import pickle
import struct

#MAV_CMD_CONDITION_YAW 
   
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
    cap = cv2.VideoCapture(0)
    
    # with pyrs.Service() as serv:
        # with serv.Device(0, streams) as dev:
    
    tracker = None
    tracker_bb = (0,0,0,0)
    control_x = 0
    control_y = 0
    pid_x_conf = [0.01, 0, 0]
    pid_y_conf = [0.01, 0, 0]
    pid_x = PID(0.01, pid_x_conf)
    pid_y = PID(0.01, pid_y_conf)
    
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.bind(('0.0.0.0', 7171))
    server_sock.settimeout(0.001)
    server_sock.listen(1)
    print('Listening...')
        
    client_sock = None
    cnt = 0
        
    while True: 
        ret, frame = cap.read()
        cv2.imshow('server frame', frame)        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
        if client_sock is None:
            try:
                client_sock, addr = server_sock.accept()
                recv_data = b''
            except socket.timeout:
                continue
    
            client_sock.settimeout(0.01)
            print('New client: {}'.format(addr))
        else:
            try:
                recv_data += client_sock.recv(64)
                bb = [0,0,0,0]
                try:
                    (bb[0], bb[1], bb[2], bb[3], \
                     pid_x_conf[0], pid_x_conf[1], pid_x_conf[2], \
                     pid_y_conf[0], pid_y_conf[1], pid_y_conf[2]) = struct.unpack('>LLLLdddddd', recv_data[:64])
                    recv_data = recv_data[64:]
                except struct.error:
                    pass
                else:
                    print('bb', bb)
                    print('pid_x', pid_x_conf)
                    print('pid_y', pid_y_conf)
                    tracker = cv2.TrackerMedianFlow_create()
                    tracker.init(frame, tuple(bb))
                    pid_x.set_parameters(*pid_x_conf)
                    pid_y.set_parameters(*pid_y_conf)
                    pid_x.clear()
                    pid_y.clear()
            except (socket.timeout, OSError):
                pass
                
            if tracker is not None:
                detected, tracker_bb = tracker.update(frame)
                if detected:
                    tracker_bb = (int(tracker_bb[0]), int(tracker_bb[1]), int(tracker_bb[2]), int(tracker_bb[3]))
                    bb_center = (int(tracker_bb[0] + tracker_bb[2]/2), int(tracker_bb[1] + tracker_bb[3]/2))
                    frame_center = (frame.shape[1]//2, frame.shape[0]//2)
                    diff_x = -(frame_center[0] - bb_center[0])
                    diff_y = frame_center[1] - bb_center[1]
               
                    control_x = pid_x.feed(diff_x)
                    control_y = pid_y.feed(diff_y)
                    
                print(detected, tracker_bb, control_x, control_y)
             
            if cnt > 2:
                cnt = 0
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 25]
                result, frame = cv2.imencode('.jpg', frame, encode_param)
                data = pickle.dumps(frame, 0)

                try:
                    client_sock.settimeout(None)
                    client_sock.sendall(struct.pack('>LLLLLdd', len(data), *tracker_bb, control_x, control_y) + data)
                    client_sock.settimeout(0.01)
                except:
                    print('Clinet disconnected')
                    client_sock = None
            cnt += 1
    
if '__main__' == __name__:
    main()