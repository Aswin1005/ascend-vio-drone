#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
VIO Bridge Node (ROS 1 Version)
===============================
Transforms OpenVINS odometry output and publishes it as
geometry_msgs/PoseStamped to /mavros/vision_pose/pose for
ArduPilot's EKF3 ExternalNav fusion.

Compatible with Python 2.7 (ROS Melodic).
"""

import math
import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import State


class VioBridgeNode(object):
    """Bridge between OpenVINS odometry and MAVROS vision pose."""

    def __init__(self):
        rospy.init_node('vio_bridge', anonymous=False)

        # ── Parameters ───────────────────────────────────────────────
        self.publish_rate = rospy.get_param('~publish_rate', 30.0)
        camera_pitch_deg = rospy.get_param('~camera_pitch_deg', -40.0)
        camera_yaw_deg   = rospy.get_param('~camera_yaw_deg',    0.0)
        self.auto_set_origin = rospy.get_param('~auto_set_ekf_origin', True)
        self.origin_lat = rospy.get_param('~origin_lat', 10.5276)
        self.origin_lon = rospy.get_param('~origin_lon', 76.2144)
        self.origin_alt = rospy.get_param('~origin_alt', 0.0)

        # ── Camera-to-body rotation matrix (pitch then yaw around Z) ──
        # camera_yaw_deg: try -90 or +90 to swap X/Y if forward = Y instead of X
        pitch_rad = math.radians(camera_pitch_deg)
        yaw_rad   = math.radians(camera_yaw_deg)

        R_pitch = np.array([
            [ math.cos(pitch_rad), 0, math.sin(pitch_rad)],
            [ 0,                   1, 0                   ],
            [-math.sin(pitch_rad), 0, math.cos(pitch_rad) ],
        ])
        R_yaw = np.array([
            [ math.cos(yaw_rad), -math.sin(yaw_rad), 0],
            [ math.sin(yaw_rad),  math.cos(yaw_rad), 0],
            [ 0,                  0,                  1],
        ])
        self.R_body_cam = R_yaw.dot(R_pitch)


        # ── State ────────────────────────────────────────────────────
        self.origin_set = False
        self.mavros_connected = False
        self.last_pose_time = None
        self.pose_count = 0
        self.last_log_time = rospy.Time.now()

        # ── Rate limiting timer ──────────────────────────────────────
        self.min_period = 1.0 / self.publish_rate
        self.last_publish_time = 0.0

        # ── Publishers ───────────────────────────────────────────────
        self.vision_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        self.origin_pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=10)

        # ── Subscribers ──────────────────────────────────────────────
        rospy.Subscriber('/ov_msckf/odomimu', Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber('/mavros/state', State, self.state_callback, queue_size=10)

        # ── Health monitoring timer (every 5 seconds) ────────────────
        rospy.Timer(rospy.Duration(5.0), self.health_check)

        rospy.loginfo('='*50)
        rospy.loginfo('VIO Bridge Node Started (ROS 1)')
        rospy.loginfo('  Publish rate limit: %s Hz', self.publish_rate)
        rospy.loginfo('  Camera pitch: %s degrees', camera_pitch_deg)
        rospy.loginfo('  Auto-set EKF origin: %s', self.auto_set_origin)
        rospy.loginfo('='*50)

    def state_callback(self, msg):
        """Track MAVROS connection state."""
        if not self.mavros_connected and msg.connected:
            rospy.loginfo('MAVROS connected to Pixhawk!')
        elif self.mavros_connected and not msg.connected:
            rospy.logwarn('MAVROS disconnected from Pixhawk!')
        self.mavros_connected = msg.connected

    def set_ekf_origin(self):
        """Set the EKF origin (required for GPS-denied ArduPilot flight)."""
        if self.origin_set:
            return

        origin_msg = GeoPointStamped()
        origin_msg.header.stamp = rospy.Time.now()
        origin_msg.header.frame_id = 'map'
        origin_msg.position.latitude = self.origin_lat
        origin_msg.position.longitude = self.origin_lon
        origin_msg.position.altitude = self.origin_alt

        self.origin_pub.publish(origin_msg)
        self.origin_set = True
        rospy.loginfo('EKF origin set: lat=%s, lon=%s, alt=%s', 
                      self.origin_lat, self.origin_lon, self.origin_alt)

    def odom_callback(self, msg):
        """Process OpenVINS odometry and publish as vision pose."""
        now = rospy.Time.now().to_sec()
        if (now - self.last_publish_time) < self.min_period:
            return
        self.last_publish_time = now

        if self.auto_set_origin and not self.origin_set and self.mavros_connected:
            self.set_ekf_origin()

        try:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation

            # Check for NaN/Inf in input data to avoid sending corrupted numbers to EKF
            coords = [pos.x, pos.y, pos.z]
            quats = [ori.x, ori.y, ori.z, ori.w]
            if np.isnan(coords).any() or np.isinf(coords).any():
                rospy.logerr_throttle(5.0, 'VIO Bridge received NaN/Inf position from OpenVINS!')
                return
            if np.isnan(quats).any() or np.isinf(quats).any():
                rospy.logerr_throttle(5.0, 'VIO Bridge received NaN/Inf orientation from OpenVINS!')
                return

            # Apply camera-to-body rotation to position
            pos_cam = np.array([pos.x, pos.y, pos.z])
            pos_body = self.R_body_cam.dot(pos_cam)

            # Apply camera-to-body rotation to orientation
            q = ori
            R_world_cam = self.quat_to_rotmat(q.w, q.x, q.y, q.z)
            R_world_body = R_world_cam.dot(self.R_body_cam.T)
            quat_body = self.rotmat_to_quat(R_world_body)

            # Build PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = msg.header.stamp  # Use OpenVINS timestamp
            pose_msg.header.frame_id = 'map'

            pose_msg.pose.position.x = pos_body[0]
            pose_msg.pose.position.y = pos_body[1]
            pose_msg.pose.position.z = pos_body[2]

            pose_msg.pose.orientation.w = quat_body[0]
            pose_msg.pose.orientation.x = quat_body[1]
            pose_msg.pose.orientation.y = quat_body[2]
            pose_msg.pose.orientation.z = quat_body[3]

            self.vision_pub.publish(pose_msg)
            self.pose_count += 1
            self.last_pose_time = rospy.Time.now()
        except Exception as e:
            rospy.logerr_throttle(5.0, 'Error processing OpenVINS odometry in VIO Bridge: %s', str(e))

    def health_check(self, event):
        """Periodic health monitoring."""
        now = rospy.Time.now()
        elapsed = (now - self.last_log_time).to_sec()

        if elapsed > 0:
            rate = self.pose_count / elapsed
            connected_str = "connected" if self.mavros_connected else "DISCONNECTED"
            origin_str = "set" if self.origin_set else "NOT SET"
            rospy.loginfo('VIO Bridge: %.1f Hz | MAVROS: %s | Origin: %s | Total poses: %d',
                          rate, connected_str, origin_str, self.pose_count)

        if self.last_pose_time is not None:
            stale = (now - self.last_pose_time).to_sec()
            if stale > 2.0:
                rospy.logwarn('No VIO data for %.1fs! Check OpenVINS.', stale)
        else:
            rospy.logwarn_throttle(5.0, 'No VIO data received yet! Check if OpenVINS is publishing to /ov_msckf/odomimu.')

        self.pose_count = 0
        self.last_log_time = now

    @staticmethod
    def quat_to_rotmat(w, x, y, z):
        """Convert quaternion (w,x,y,z) to 3x3 rotation matrix."""
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
            [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
        ])

    @staticmethod
    def rotmat_to_quat(R):
        """Convert 3x3 rotation matrix to quaternion (w,x,y,z)."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        return [w/norm, x/norm, y/norm, z/norm]


if __name__ == '__main__':
    try:
        VioBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
