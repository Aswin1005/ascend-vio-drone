#!/usr/bin/env python
from __future__ import print_function
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class XYOffsetChecker(object):
    def __init__(self):
        rospy.init_node('check_xy_offsets', anonymous=True)
        self.bridge = CvBridge()
        
        # Topic parameters (using Rectified Infrared 1 feed)
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/infra1/image_rect_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/infra1/camera_info')
        
        # Exact Physical Measurements for A2 Board (in meters)
        self.large_size = 0.172  # 17.2 cm
        self.medium_size = 0.0885 # 8.85 cm
        self.small_size = 0.0395  # 3.95 cm
        
        # Mapping of all marker coordinates relative to board center (X: Forward/North, Y: Right/East)
        self.marker_offsets = {
            0: (0.0, 0.0, self.large_size),
            20: (0.1378, -0.2214, self.medium_size), 21: (0.1378, -0.2214, self.small_size),
            3: (0.1378, -0.1107, self.medium_size), 4: (0.1378, -0.1107, self.small_size),
            11: (0.1378, 0.0, self.medium_size), 12: (0.1378, 0.0, self.small_size),
            5: (0.1378, 0.1107, self.medium_size), 6: (0.1378, 0.1107, self.small_size),
            22: (0.1378, 0.2214, self.medium_size), 23: (0.1378, 0.2214, self.small_size),
            
            24: (0.0, -0.2214, self.medium_size), 25: (0.0, -0.2214, self.small_size),
            13: (0.0, -0.1107, self.medium_size), 14: (0.0, -0.1107, self.small_size),
            1: (0.0, 0.0, self.medium_size), 2: (0.0, 0.0, self.small_size),
            15: (0.0, 0.1107, self.medium_size), 16: (0.0, 0.1107, self.small_size),
            26: (0.0, 0.2214, self.medium_size), 27: (0.0, 0.2214, self.small_size),
            
            28: (-0.1378, -0.2214, self.medium_size), 29: (-0.1378, -0.2214, self.small_size),
            7: (-0.1378, -0.1107, self.medium_size), 8: (-0.1378, -0.1107, self.small_size),
            17: (-0.1378, 0.0, self.medium_size), 18: (-0.1378, 0.0, self.small_size),
            9: (-0.1378, 0.1107, self.medium_size), 19: (-0.1378, 0.1107, self.small_size),
            30: (-0.1378, 0.2214, self.medium_size), 31: (-0.1378, 0.2214, self.small_size)
        }
        
        # Mathematically corrected rotation matrix for 35 degree tilt
        self.R_body_cam = np.array([
            [ 0.0,  0.8192,  0.5736], # Corrected sign: +0.8192
            [ 1.0,  0.0,     0.0],    # Keeps Y rock-solid
            [ 0.0,  0.5736, -0.8192]  # Corrected sign: -0.8192
        ])
        
        # Physical mounting offset of the camera relative to Pixhawk center (in meters)
        self.t_body_cam = np.array([
             0.09175,  # 9.1 cm forward
            -0.02347,  # 2.3 cm left
            -0.11550   # 11.5 cm below
        ])
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        # SUPER ROBUST DETECTOR TUNING:
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        try:
            self.aruco_params.cornerRefinementMethod = 1
        except AttributeError:
            self.aruco_params.doCornerRefinement = True
            
        self.aruco_params.minMarkerPerimeterRate = 0.015
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 33
        self.aruco_params.adaptiveThreshWinSizeStep = 5
        self.aruco_params.adaptiveThreshConstant = 7
        try:
            self.aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.2
        except AttributeError:
            pass
        
        self.cam_matrix = None
        self.dist_coeffs = None
        
        # Low-Pass Filter States
        self.filtered_x = None
        self.filtered_y = None
        self.filter_alpha = 0.15
        
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.info_callback)
        rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        
        self.last_print = rospy.Time.now()
        rospy.loginfo("Corrected robust XY Offset Checker started.")

    def info_callback(self, msg):
        if self.cam_matrix is None:
            self.cam_matrix = np.array(msg.K).reshape((3, 3))
            self.dist_coeffs = np.array(msg.D)

    def image_callback(self, msg):
        now = rospy.Time.now()
        if (now - self.last_print).to_sec() < 0.5:
            return
        self.last_print = now
        
        if self.cam_matrix is None:
            return
            
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError:
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError:
                return
            
        if len(cv_img.shape) == 3:
            gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        else:
            gray = cv_img
            
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        print("\n" + "="*60)
        print("LANDING PAD XY OFFSET CHECKER (MATHEMATICALLY CORRECTED):")
        print("="*60)
        
        calculated_offsets = []
        if ids is not None:
            flat_ids = ids.flatten()
            for idx, marker_id in enumerate(flat_ids):
                if marker_id in self.marker_offsets:
                    ox, oy, size = self.marker_offsets[marker_id]
                    
                    res = cv2.aruco.estimatePoseSingleMarkers(
                        np.array([corners[idx]]), size, self.cam_matrix, self.dist_coeffs
                    )
                    rvec = res[0][0][0]
                    tvec = res[1][0][0]
                    
                    R, _ = cv2.Rodrigues(rvec)
                    local_offset = np.array([oy, -ox, 0.0])
                    offset_cam = R.dot(local_offset)
                    center_cam = tvec - offset_cam
                    
                    # Project to body frame using corrected matrix & translation offset
                    center_body = self.R_body_cam.dot(center_cam) + self.t_body_cam
                    
                    calculated_offsets.append((center_body[0], center_body[1], marker_id))
                    
        if len(calculated_offsets) > 0:
            raw_x = np.mean([x[0] for x in calculated_offsets])
            raw_y = np.mean([x[1] for x in calculated_offsets])
            visible_ids = [x[2] for x in calculated_offsets]
            
            # Apply low-pass filter to smooth out pixel noise
            if self.filtered_x is None:
                self.filtered_x = raw_x
                self.filtered_y = raw_y
            else:
                self.filtered_x = self.filter_alpha * raw_x + (1.0 - self.filter_alpha) * self.filtered_x
                self.filtered_y = self.filter_alpha * raw_y + (1.0 - self.filter_alpha) * self.filtered_y
            
            print("  Visible Markers: %s" % str(visible_ids))
            print("  Offset X (Forward/Backward): %+.1f cm" % (self.filtered_x * 100.0))
            print("  Offset Y (Left/Right):       %+.1f cm" % (self.filtered_y * 100.0))
            
            error = np.sqrt(self.filtered_x**2 + self.filtered_y**2) * 100.0
            print("  Total Horizontal Error:     %.1f cm" % error)
            
            if error < 10.0:
                print("  STATUS: ALIGNED (Perfect for landing!)")
            else:
                dir_x = "BACK" if self.filtered_x > 0 else "FORWARD"
                dir_y = "LEFT" if self.filtered_y > 0 else "RIGHT"
                print("  STATUS: MISALIGNED -> Move drone %s by %.0f cm and %s by %.0f cm" % (
                    dir_x, abs(self.filtered_x*100), dir_y, abs(self.filtered_y*100)
                ))
        else:
            print("  No markers detected in camera view.")
        print("="*60)

if __name__ == '__main__':
    try:
        node = XYOffsetChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

