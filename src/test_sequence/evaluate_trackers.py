import cv2
import numpy as np
import os
import time

show_frames = False

def main():
    starting_bb = (478, 303, 844-478, 668-303)
    
    sizes = (
        (1920, 1080),
        (1366, 768),
        (640, 360),
        (320, 180),
    )
    
    trackers = {
        'MOSSE': cv2.TrackerMOSSE_create,
        'CSRT': cv2.TrackerCSRT_create,
        'MedianFlow': cv2.TrackerMedianFlow_create
    }
    
    result_file = open('evaluation_result.txt', 'w')
    
    for tracker_name in trackers:
        print('\n***Tracker: {}***'.format(tracker_name))
        result_file.write('\n***Tracker: {}***\n'.format(tracker_name))
        
        for size in sizes:
            print('Size: {}'.format(size))
            result_file.write('Size: {}\n'.format(size))
            
            tracker = trackers[tracker_name]()
            tracker_initialized = False
            
            scale_x = size[0]/1920
            scale_y = size[1]/1080
            bb = (starting_bb[0] * scale_x, starting_bb[1] * scale_y, starting_bb[2] * scale_x, starting_bb[3] * scale_y)
            
            tp = 0
            fp = 0
            fn = 0
            tn = 0
            
            start_time = time.time()
            i = 0
                
            for frame_no in range(108, 438, 3):
                i += 1
                print('Processing frame {}'.format(frame_no))
                frame = cv2.imread(os.path.join('frames', '{}.jpg'.format(frame_no)))
                gt = cv2.imread(os.path.join('gt', '{}.png'.format(frame_no)))
                gt_bb = cv2.imread(os.path.join('gt_bb', '{}.png'.format(frame_no)))
                
                frame = cv2.resize(frame, size)
                gt = cv2.resize(gt, size)
                gt_bb = cv2.resize(gt_bb, size)
                
                if not tracker_initialized:
                    tracker_initialized = True
                    tracker.init(frame, bb)
                else:
                    ret, bb = tracker.update(frame)
                
                print(bb)
                
                p1 = (int(bb[0]), int(bb[1]))
                p2 = (int(bb[0] + bb[2]), int(bb[1] + bb[3]))
                cv2.rectangle(frame, p1, p2, (255,0,0), 2)
                
                gt = cv2.cvtColor(gt, cv2.COLOR_BGR2GRAY)
                detection = np.zeros(gt.shape, dtype=np.uint8)
                cv2.rectangle(detection, p1, p2, 255, -1)
                
                tp += np.sum(np.logical_and((detection==255),(gt==255)))
                fp += np.sum(np.logical_and((detection==255),(gt==0)))
                fn += np.sum(np.logical_and((detection==0), (gt==255)))
                tn += np.sum(np.logical_and((detection==0), (gt==0)))
                        
                if show_frames:
                    cv2.imshow('', frame)
                    cv2.waitKey(1)
            
            end_time = time.time()
            diff_time = end_time - start_time
            
            precision = tp / (tp + fp)
            recall = tp / (tp + fn)
            accuracy = (tp + tn) / (tp + tn + fp + fn)
            f1_score = (2*tp) / (2*tp + fp + fn)
    
            print()
            
            print('precision', precision)
            print('recall', recall)
            print('accuracy', precision)
            print('f1_score', recall)
            
            result_file.write('Precision: {:.3f}\n'.format(precision))
            result_file.write('Recall: {:.3f}\n'.format(recall))
            result_file.write('Accuracy: {:.3f}\n'.format(precision))
            result_file.write('F1_score: {:.3f}\n'.format(recall))
            result_file.write('Elapsed: {:.3f}s\n'.format(diff_time))
            result_file.write('Avg frame took: {:.3f}s\n'.format(diff_time / i))
            result_file.write('=======================\n')
            
            print('Took {} seconds, avg frame processing: {:.3f} seconds'.format(diff_time, diff_time / i))
            
if __name__ == '__main__':
    main()