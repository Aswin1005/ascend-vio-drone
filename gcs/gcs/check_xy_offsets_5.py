#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32

# ── cv_bridge-free image conversion ──────────────────────────────
# Avoids loading the Python-2.7-compiled cv_bridge_boost .so, which
# fails with 'PyInit_cv_bridge_boost' when run under python3 after
# sourcing ROS Melodic (which prepends the Python 2.7 dist-packages
# to PYTHONPATH). We decode the raw bytes directly using numpy.
def imgmsg_to_cv2(msg, desired_encoding="passthrough"):
    """Lightweight cv_bridge replacement for mono8 / bgr8 / rgb8 / mono16."""
    enc = msg.encoding.lower()
    if enc in ("mono8", "8uc1"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
    elif enc in ("mono16", "16uc1"):
        img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
    elif enc in ("bgr8", "8uc3"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    elif enc in ("rgb8",):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        img = img[:, :, ::-1]   # RGB → BGR
    elif enc in ("bayer_rggb8", "bayer_bggr8", "bayer_gbrg8", "bayer_grbg8"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        bayer_map = {
            "bayer_rggb8": cv2.COLOR_BAYER_RG2GRAY,
            "bayer_bggr8": cv2.COLOR_BAYER_BG2GRAY,
            "bayer_gbrg8": cv2.COLOR_BAYER_GB2GRAY,
            "bayer_grbg8": cv2.COLOR_BAYER_GR2GRAY,
        }
        img = cv2.cvtColor(img, bayer_map[enc])
    else:
        raise ValueError(f"Unsupported encoding: {msg.encoding}")

    # Convert to desired encoding
    if desired_encoding == "mono8":
        if len(img.shape) == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img.dtype != np.uint8:
            img = (img // 256).astype(np.uint8)
    elif desired_encoding == "bgr8" and len(img.shape) == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img

class XYOffsetChecker(object):
    def __init__(self):
        rospy.init_node('check_xy_offsets', anonymous=True)
        
        # Topic parameters (using Rectified Infrared 1 feed)
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/infra1/image_rect_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/infra1/camera_info')
        
        # ----------------------------------------------------------------
        # 9-MARKER A0 BOARD OFFSETS (confirmed from physical A0 printout)
        # Convention: ox = forward (north), oy = right (east), size in metres
        # ----------------------------------------------------------------
        self.marker_offsets = {
            # Center - 30 cm
            0:  ( 0.0,     0.0,    0.30),
            # Four corners - 20 cm  (+-45 cm left/right, +-25 cm forward/back)
            10: ( 0.25,  -0.45,   0.20),   # Top-Left
            20: ( 0.25,   0.45,   0.20),   # Top-Right
            30: (-0.25,  -0.45,   0.20),   # Bottom-Left
            40: (-0.25,   0.45,   0.20),   # Bottom-Right
            # Left/Right edges - 20 cm  (0 forward/back, +-45 cm left/right)
            13: ( 0.0,   -0.45,   0.20),   # Left-Edge
            14: ( 0.0,    0.45,   0.20),   # Right-Edge
            # Top/Bottom center - 15 cm  (+-29.55 cm forward/back, 0 left/right)
            11: ( 0.296,  0.0,    0.15),   # Top-Center
            12: (-0.296,  0.0,    0.15),   # Bottom-Center
        }
        
        # Factory-calibrated camera to body (IMU) rotation matrix
        self.R_body_cam = np.array([
            [ 0.1054220329945270,  0.8128383360258432,  0.5728699978581873],
            [ 0.9944182947436805, -0.0836821167198117, -0.0642616403491435],
            [-0.0042953507856418,  0.5764469991491242, -0.8171232508829975]
        ])
        
        # Factory-calibrated translation offset (in meters)
        self.t_body_cam = np.array([
             0.0917517043091625,
            -0.0234705563751033,
            -0.1155101841902607
        ])
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        # ----------------------------------------------------------------
        # CREATE ARUCO BOARD FOR AMBIGUITY-FREE POSE ESTIMATION
        # ----------------------------------------------------------------
        obj_points = []
        ids_list = []
        for marker_id, (ox, oy, size) in self.marker_offsets.items():
            s = size / 2.0
            # Marker corners in the Board frame (X=East/Right, Y=North/Forward, Z=Up)
            # OpenCV expects corners in order: Top-Left, Top-Right, Bottom-Right, Bottom-Left
            # OpenCV 3.2 strictly expects CV_32FC3 (3-channel float). 
            # Reshaping to (1, 4, 3) creates a 3-channel array in Python.
            c = np.array([[
                [oy - s, ox + s, 0.0],
                [oy + s, ox + s, 0.0],
                [oy + s, ox - s, 0.0],
                [oy - s, ox - s, 0.0]
            ]], dtype=np.float32)
            obj_points.append(c)
            ids_list.append([marker_id])
            
        try:
            self.board = cv2.aruco.Board_create(obj_points, self.aruco_dict, np.array(ids_list, dtype=np.int32))
        except AttributeError:
            self.board = cv2.aruco.Board(np.array(obj_points), self.aruco_dict, np.array(ids_list, dtype=np.int32))
        
        # SUPER ROBUST DETECTOR TUNING (Compatible with all OpenCV versions):
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        try:
            self.aruco_params.cornerRefinementMethod = 1  # 1 = Subpixel refinement (OpenCV 3.3+)
        except AttributeError:
            self.aruco_params.doCornerRefinement = True   # Fallback (OpenCV 3.2)
            
        self.aruco_params.minMarkerPerimeterRate = 0.015  # Detect small markers far away/near edges
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
        
        # EMA low-pass filter (alpha=0.35 -> responsive but smoothed)
        self.filtered_x = None
        self.filtered_y = None
        self.alpha = 0.35
        
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.info_callback)
        rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        
        # --- ROS Publishers for arena nav integration ---
        self.pub_status   = rospy.Publisher('/aruco/status',   Bool,    queue_size=5)
        self.pub_x_offset = rospy.Publisher('/aruco/x_offset', Float32, queue_size=5)
        self.pub_y_offset = rospy.Publisher('/aruco/y_offset', Float32, queue_size=5)
        
        self.last_print = rospy.Time.now()
        rospy.loginfo("A0 9-Marker XY Offset Checker (INFRARED) started.")
        rospy.loginfo("Publishing: /aruco/status, /aruco/x_offset, /aruco/y_offset")

    def info_callback(self, msg):
        if self.cam_matrix is None:
            self.cam_matrix = np.array(msg.K).reshape((3, 3))
            self.dist_coeffs = np.array(msg.D)

    def image_callback(self, msg):
        now = rospy.Time.now()
        if (now - self.last_print).to_sec() < 0.1:   # 10 updates/sec
            return
        self.last_print = now
        
        if self.cam_matrix is None:
            return
            
        try:
            cv_img = imgmsg_to_cv2(msg, "mono8")
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Image decode error: {e}")
            return
            
        if len(cv_img.shape) == 3:
            gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        else:
            gray = cv_img
            
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        print("\n" + "="*60)
        print("LANDING PAD XY OFFSET CHECKER (A0 9-MARKER):")
        print("="*60)
        
        raw_x, raw_y = None, None
        used_ids = []
        
        if ids is not None and len(ids) > 0:
            # --- USE FULL BOARD ESTIMATION ---
            # This completely eliminates "Pose Ambiguity" which causes X fluctuations
            # when approaching from the side. It optimally fuses ALL visible markers.
            try:
                retval, rvec, tvec = cv2.aruco.estimatePoseBoard(
                    corners, ids, self.board, self.cam_matrix, self.dist_coeffs, None, None
                )
            except TypeError:
                retval, rvec, tvec = cv2.aruco.estimatePoseBoard(
                    corners, ids, self.board, self.cam_matrix, self.dist_coeffs
                )
                
            if retval > 0:
                # Project camera to drone body coordinates using exact calibration
                center_body = self.R_body_cam.dot(tvec.flatten()) + self.t_body_cam
                raw_x, raw_y = center_body[0], center_body[1]
                used_ids = ids.flatten().tolist()
        
        # --- EMA Filter ---
        if raw_x is not None:
            if self.filtered_x is None:
                self.filtered_x, self.filtered_y = raw_x, raw_y
            else:
                self.filtered_x = self.alpha * raw_x + (1.0 - self.alpha) * self.filtered_x
                self.filtered_y = self.alpha * raw_y + (1.0 - self.alpha) * self.filtered_y
            
            fx_cm = self.filtered_x * 100.0
            fy_cm = self.filtered_y * 100.0
            error = np.sqrt(self.filtered_x**2 + self.filtered_y**2) * 100.0
            
            # Publish to ROS topics (offsets in cm)
            self.pub_status.publish(Bool(data=True))
            self.pub_x_offset.publish(Float32(data=fx_cm))
            self.pub_y_offset.publish(Float32(data=fy_cm))
            
            mode = "CENTER(precise)" if 0 in used_ids else "FALLBACK(coarse)"
            print("  Mode:          %s" % mode)
            print("  Visible IDs:   %s" % str(used_ids))
            print("  Offset X (Fwd/Back): %+.1f cm  [raw: %+.1f cm]" % (fx_cm, raw_x*100))
            print("  Offset Y (Left/Rgt): %+.1f cm  [raw: %+.1f cm]" % (fy_cm, raw_y*100))
            print("  Total Error:         %.1f cm" % error)
            
            if error < 10.0:
                print("  STATUS: [OK] ALIGNED (Perfect for landing!)")
            else:
                dir_x = "BACK" if self.filtered_x > 0 else "FORWARD"
                dir_y = "LEFT" if self.filtered_y > 0 else "RIGHT"
                print("  STATUS: [!!] Move drone %s by %.0f cm and %s by %.0f cm" % (
                    dir_x, abs(fx_cm), dir_y, abs(fy_cm)
                ))
        else:
            # No markers: publish status=False, reset filter
            self.pub_status.publish(Bool(data=False))
            self.filtered_x = None
            self.filtered_y = None
            print("  No markers detected in camera view.")
        print("="*60)

if __name__ == '__main__':
    try:
        node = XYOffsetChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
