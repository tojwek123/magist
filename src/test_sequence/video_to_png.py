import cv2
import numpy as np

def main():
    cap = cv2.VideoCapture('krakow.avi')
    
    frame_no = 1
    every_nth_frame = 3
    
    while cap.isOpened():
        ret, frame = cap.read()
        
        if frame_no % every_nth_frame == 0:
            print('Saving frame: {}'.format(frame_no))
            cv2.imwrite('frames/{}.jpg'.format(frame_no), frame)
        frame_no += 1
    
if __name__ == '__main__':
    main()