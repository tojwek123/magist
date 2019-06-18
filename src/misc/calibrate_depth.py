import cv2
import numpy as np

frame = None
x_start = 0
y_start = 0
x_end = 0
y_end = 0
mouse_pressed = False

def on_mouse(event, x, y, flags, param):
    global frame, x_start, y_start, x_end, y_end, mouse_pressed
    
    x_end, y_end = x, y
    
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_pressed = True
        x_start, y_start = x, y
    elif event == cv2.EVENT_LBUTTONUP:
        mouse_pressed = False
        
        if y_start > y_end:
            y_start, y_end = y_end, y_start
        if x_start > x_end:
            x_start, x_end = x_end, x_start
        
        roi = frame[y_start:y_end, x_start:x_end]
        print('Mean value: {}'.format(roi.mean()))
        
        cv2.imshow('roi', roi)
        

def main():
    global frame, x_start, y_start, x_end, y_end, mouse_pressed
    
    cap = cv2.VideoCapture(0)
    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame', on_mouse)
    
    with pyrs.Service() as serv:
        with serv.Device(0, streams) as dev:
    
            while True:
                # ret, frame = cap.read()
                # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                dev.wait_for_frames()
                frame = np.uint8(dev.depth / 256.)
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                
                disp_im = frame.copy()
                
                if mouse_pressed:
                    cv2.rectangle(disp_im, (x_start, y_start), (x_end, y_end), 0, 2)
            
                cv2.imshow('frame', disp_im)
            
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    
if '__main__' == __name__:
    main()