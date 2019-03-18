import cv2
import numpy as np
import math

class Basic_Odometry:
    
    def_config = {
        'show_matches': True,
        'best_matches_percentage': 0.05,
    }

    def __init__(self, feature_detector, matcher, config=def_config):
        """Initialize class
        
        Params:
        feature_detector -- cv2 feature detector e.g. created by cv2.ORB_create()
        matcher -- cv2 matcher e.g. created by cv2.BFMatcher()
        config -- dictionary containing configuration of class (check 'def_config' for keys)
        """
        self._feature_detector = feature_detector
        self._matcher = matcher
        self._config = config

        self._last_keypoints = None        
        self._last_descriptors = None
        self._last_frame = None

    def set_config(self, config):
        """Set odometry configuration

        Params:
        config -- new configuration
        """
        self._config = config

    def get_displacement(self, frame):
        """Calculate displacement basing on new frame

        Params:
        frame -- new frame

        Return:
        Dictionary containing position change with keys 'roll', 'horizontal', 'vertical' and 'depth' (which is based on scale change, so for no change it is 1.0), or None if displacement couldn't be calculated

        """
        displacement = None
        keypoints, descriptors = self._feature_detector.detectAndCompute(frame, None)
        
        if descriptors is not None and self._last_descriptors is not None:
            matches = self._matcher.match(descriptors, self._last_descriptors)
            matches = self._select_matches(matches)           
        
            if len(matches) > 0:
                if self._config['show_matches']:
                    matches_preview = cv2.drawMatches(frame, keypoints, self._last_frame, self._last_keypoints, matches, None, flags=2)
                    cv2.imshow('Basic_Odometry_matches_preview', matches_preview)
                        
                points, last_points = self._get_points_from_matches(keypoints, self._last_keypoints, matches)
                #It uses default values for parameters - there must be here because of broken python bindings I suppose
                tf_mat = cv2.estimateRigidTransform(points, last_points, False, 500, 0.5, 3)
                
                if tf_mat is not None:
                    a11, a12, b1 =  tf_mat[0][0], tf_mat[0][1], tf_mat[0][2]
                    a21, a22, b2 = -tf_mat[1][0], tf_mat[1][1], tf_mat[1][2]
                    roll = math.atan2(-a12, a11)                   
 
                    displacement = {
                        'roll': roll,
                        'horizontal': b1,
                        'vertical': b2,
                        'depth': math.sqrt(a11**2+a12**2) * (-1 if a11 < 0 else 1)
                    }
                                
        if descriptors is not None:        
            self._last_keypoints = keypoints
            self._last_descriptors = descriptors
            self._last_frame = frame
            
        return displacement

    def _get_points_from_matches(self, keypoints_1, keypoints_2, matches):
        """Convert keypoints to numpy array of matches

        Params:
        keypoints_1 -- first set of keypoints (query)
        keypoints_2 -- second set of keypoints (train)
        matches -- list of matches obtained from matcher

        Return:
        Two numpy array of keypoints
        """
        points_1, points_2 = [], []
        
        for match in matches:
            points_1.append(keypoints_1[match.queryIdx].pt)
            points_2.append(keypoints_2[match.trainIdx].pt)

        return np.array(points_1), np.array(points_2)

    def _select_matches(self, matches):
        """Select most appropiate matches

        Params:
        matches -- list of matches obtained from matcher

        Return:
        Selected matches (typically reduced number)
        """
        matches = sorted(matches, key=lambda x: x.distance)
        matches = matches[:int(self._config['best_matches_percentage'] * len(matches))]
        return matches


