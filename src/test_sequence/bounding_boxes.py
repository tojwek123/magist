import cv2
import numpy as np
import os

def main():
    for filename in os.listdir('gt'):
        print('Processing file {}'.format(filename))
    
        gt = cv2.imread(os.path.join('gt', filename))
        
        height = gt.shape[0]
        width = gt.shape[1]
        
        top = None
        bottom = None
        left = None
        right = None
        
        for i in range(height):
            if (255,255,255) in gt[i]:
                top = i
                break
        for i in range(height):
            if (255,255,255) in gt[height-1-i]:
                bottom = height-1-i
                break
                
        for i in range(width):
            if (255,255,255) in gt[:,i]:
                left = i
                break
        for i in range(width):
            if (255,255,255) in gt[:,width-1-i]:
                right = width-1-i
                break
                
        cv2.rectangle(gt, (left, top), (right, bottom), (255,255,255), -1)
        cv2.imwrite(os.path.join('gt_bb', filename), gt)
    
if __name__ == '__main__':
    main()