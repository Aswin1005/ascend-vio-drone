#!/usr/bin/env python3
"""
RealSense D435i ROS Publisher (Enhanced)
Publishes: color, IR stereo, depth, camera_info, IMU
"""

import rospy
import pyrealsense2 as rs
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, Imu
import threading


def numpy_to_image(arr, encoding):
    msg = Image()
    if len(arr.shape) == 2:
        msg.height, msg.width = arr.shape
        channels = 1
    else:
        msg.height, msg.width, channels = arr.shape
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = msg.width * channels * arr.itemsize
    msg.data = arr.tobytes()
    return msg


def make_camera_info(intrinsics, frame_id):
    """Build CameraInfo message from RealSense intrinsics"""
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width = intrinsics.width
    info.height = intrinsics.height
    info.distortion_model = "plumb_bob"
    info.D = list(intrinsics.coeffs) + [0.0] * (5 - len(intrinsics.coeffs))
    info.K = [intrinsics.fx, 0, intrinsics.ppx,
              0, intrinsics.fy, intrinsics.ppy,
              0, 0, 1]
    info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    info.P = [intrinsics.fx, 0, intrinsics.ppx, 0,
              0, intrinsics.fy, intrinsics.ppy, 0,
              0, 0, 1, 0]
    return info


class RealSensePublisher:
    def __init__(self):
        rospy.init_node('realsense_publisher')

        # Parameters
        self.color_width = rospy.get_param('~color_width', 640)
        self.color_height = rospy.get_param('~color_height', 480)
        self.color_fps = rospy.get_param('~color_fps', 30)
        self.ir_width = rospy.get_param('~ir_width', 640)
        self.ir_height = rospy.get_param('~ir_height', 480)
        self.ir_fps = rospy.get_param('~ir_fps', 30)
        self.enable_color = rospy.get_param('~enable_color', True)
        self.enable_ir = rospy.get_param('~enable_ir', True)
        self.enable_depth = rospy.get_param('~enable_depth', True)
        self.enable_imu = rospy.get_param('~enable_imu', True)
        self.emitter_enabled = rospy.get_param('~emitter_enabled', False)

        # Image publishers
        self.color_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)
        self.color_info_pub = rospy.Publisher('/camera/color/camera_info', CameraInfo, queue_size=1)
        self.infra1_pub = rospy.Publisher('/camera/infra1/image_raw', Image, queue_size=1)
        self.infra1_info_pub = rospy.Publisher('/camera/infra1/camera_info', CameraInfo, queue_size=1)
        self.infra2_pub = rospy.Publisher('/camera/infra2/image_raw', Image, queue_size=1)
        self.infra2_info_pub = rospy.Publisher('/camera/infra2/camera_info', CameraInfo, queue_size=1)
        self.depth_pub = rospy.Publisher('/camera/depth/image_rect_raw', Image, queue_size=1)
        self.depth_info_pub = rospy.Publisher('/camera/depth/camera_info', CameraInfo, queue_size=1)

        # IMU publisher (combined gyro + accel)
        self.imu_pub = rospy.Publisher('/camera/imu', Imu, queue_size=100)

        # Setup
        self.setup_camera_pipeline()
        if self.enable_imu:
            threading.Thread(target=self.imu_loop, daemon=True).start()

        # Pre-compute camera_info messages
        self.compute_camera_infos()

    def setup_camera_pipeline(self):
        self.pipeline = rs.pipeline()
        config = rs.config()

        if self.enable_color:
            config.enable_stream(rs.stream.color, self.color_width, self.color_height,
                                 rs.format.bgr8, self.color_fps)
        if self.enable_ir:
            config.enable_stream(rs.stream.infrared, 1, self.ir_width, self.ir_height,
                                 rs.format.y8, self.ir_fps)
            config.enable_stream(rs.stream.infrared, 2, self.ir_width, self.ir_height,
                                 rs.format.y8, self.ir_fps)
        if self.enable_depth:
            config.enable_stream(rs.stream.depth, self.ir_width, self.ir_height,
                                 rs.format.z16, self.ir_fps)

        self.profile = self.pipeline.start(config)

        # Disable IR emitter
        device = self.profile.get_device()
        depth_sensor = device.first_depth_sensor()
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled,
                                    1 if self.emitter_enabled else 0)
            rospy.loginfo("IR emitter: {}".format("ON" if self.emitter_enabled else "OFF"))

    def compute_camera_infos(self):
        """Pre-compute CameraInfo messages from camera intrinsics"""
        self.color_info = None
        self.infra1_info = None
        self.infra2_info = None
        self.depth_info = None

        if self.enable_color:
            stream = self.profile.get_stream(rs.stream.color)
            intr = stream.as_video_stream_profile().get_intrinsics()
            self.color_info = make_camera_info(intr, "camera_color_optical_frame")

        if self.enable_ir:
            stream = self.profile.get_stream(rs.stream.infrared, 1)
            intr = stream.as_video_stream_profile().get_intrinsics()
            self.infra1_info = make_camera_info(intr, "camera_infra1_optical_frame")

            stream = self.profile.get_stream(rs.stream.infrared, 2)
            intr = stream.as_video_stream_profile().get_intrinsics()
            self.infra2_info = make_camera_info(intr, "camera_infra2_optical_frame")

        if self.enable_depth:
            stream = self.profile.get_stream(rs.stream.depth)
            intr = stream.as_video_stream_profile().get_intrinsics()
            self.depth_info = make_camera_info(intr, "camera_depth_optical_frame")

    def imu_loop(self):
        """Separate pipeline for IMU (different fps)"""
        try:
            pipeline_imu = rs.pipeline()
            config_imu = rs.config()
            config_imu.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
            config_imu.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
            pipeline_imu.start(config_imu)
            rospy.loginfo("IMU pipeline started")

            last_accel = None
            while not rospy.is_shutdown():
                frames = pipeline_imu.wait_for_frames(timeout_ms=1000)
                for frame in frames:
                    if not frame.is_motion_frame():
                        continue
                    motion = frame.as_motion_frame()
                    data = motion.get_motion_data()
                    stype = frame.get_profile().stream_type()

                    if stype == rs.stream.accel:
                        last_accel = (data.x, data.y, data.z)
                    elif stype == rs.stream.gyro and last_accel is not None:
                        msg = Imu()
                        msg.header.stamp = rospy.Time.now()
                        msg.header.frame_id = "camera_imu_optical_frame"
                        msg.linear_acceleration.x = last_accel[0]
                        msg.linear_acceleration.y = last_accel[1]
                        msg.linear_acceleration.z = last_accel[2]
                        msg.angular_velocity.x = data.x
                        msg.angular_velocity.y = data.y
                        msg.angular_velocity.z = data.z
                        msg.orientation_covariance[0] = -1
                        self.imu_pub.publish(msg)
        except Exception as e:
            rospy.logwarn("IMU thread error: {}".format(e))

    def run(self):
        rospy.loginfo("RealSense publisher started")
        while not rospy.is_shutdown():
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                stamp = rospy.Time.now()

                if self.enable_color:
                    cf = frames.get_color_frame()
                    if cf:
                        img = np.asanyarray(cf.get_data())
                        msg = numpy_to_image(img, "bgr8")
                        msg.header.stamp = stamp
                        msg.header.frame_id = "camera_color_optical_frame"
                        self.color_pub.publish(msg)
                        if self.color_info:
                            self.color_info.header.stamp = stamp
                            self.color_info_pub.publish(self.color_info)

                if self.enable_ir:
                    ir1 = frames.get_infrared_frame(1)
                    if ir1:
                        img = np.asanyarray(ir1.get_data())
                        msg = numpy_to_image(img, "mono8")
                        msg.header.stamp = stamp
                        msg.header.frame_id = "camera_infra1_optical_frame"
                        self.infra1_pub.publish(msg)
                        if self.infra1_info:
                            self.infra1_info.header.stamp = stamp
                            self.infra1_info_pub.publish(self.infra1_info)

                    ir2 = frames.get_infrared_frame(2)
                    if ir2:
                        img = np.asanyarray(ir2.get_data())
                        msg = numpy_to_image(img, "mono8")
                        msg.header.stamp = stamp
                        msg.header.frame_id = "camera_infra2_optical_frame"
                        self.infra2_pub.publish(msg)
                        if self.infra2_info:
                            self.infra2_info.header.stamp = stamp
                            self.infra2_info_pub.publish(self.infra2_info)

                if self.enable_depth:
                    df = frames.get_depth_frame()
                    if df:
                        img = np.asanyarray(df.get_data())
                        msg = numpy_to_image(img, "16UC1")
                        msg.header.stamp = stamp
                        msg.header.frame_id = "camera_depth_optical_frame"
                        self.depth_pub.publish(msg)
                        if self.depth_info:
                            self.depth_info.header.stamp = stamp
                            self.depth_info_pub.publish(self.depth_info)

            except Exception as e:
                rospy.logwarn_throttle(5, "Frame error: {}".format(e))

        self.pipeline.stop()


if __name__ == '__main__':
    try:
        node = RealSensePublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass#!/usr/bin/env python3
"""
RealSense D435i ROS Publisher
Uses pyrealsense2 directly (bypasses broken realsense2_camera ROS wrapper)
"""

import rospy
import pyrealsense2 as rs
import numpy as np
from sensor_msgs.msg import Image


def numpy_to_image(arr, encoding):
    """Convert numpy array to sensor_msgs/Image without cv_bridge"""
    msg = Image()
    if len(arr.shape) == 2:
        msg.height, msg.width = arr.shape
        channels = 1
    else:
        msg.height, msg.width, channels = arr.shape
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = msg.width * channels * arr.itemsize
    msg.data = arr.tobytes()
    return msg


class RealSensePublisher:
    def __init__(self):
        rospy.init_node('realsense_publisher')

        # Parameters
        self.color_width = rospy.get_param('~color_width', 640)
        self.color_height = rospy.get_param('~color_height', 480)
        self.color_fps = rospy.get_param('~color_fps', 30)
        self.ir_width = rospy.get_param('~ir_width', 640)
        self.ir_height = rospy.get_param('~ir_height', 480)
        self.ir_fps = rospy.get_param('~ir_fps', 30)
        self.enable_color = rospy.get_param('~enable_color', True)
        self.enable_ir = rospy.get_param('~enable_ir', True)
        self.emitter_enabled = rospy.get_param('~emitter_enabled', False)

        # Publishers
        self.color_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)
        self.infra1_pub = rospy.Publisher('/camera/infra1/image_raw', Image, queue_size=1)
        self.infra2_pub = rospy.Publisher('/camera/infra2/image_raw', Image, queue_size=1)

        self.setup_pipeline()

    def setup_pipeline(self):
        self.pipeline = rs.pipeline()
        config = rs.config()

        if self.enable_color:
            config.enable_stream(rs.stream.color, self.color_width, self.color_height,
                                 rs.format.bgr8, self.color_fps)
            rospy.loginfo("Color: {}x{}@{}".format(
                self.color_width, self.color_height, self.color_fps))

        if self.enable_ir:
            config.enable_stream(rs.stream.infrared, 1, self.ir_width, self.ir_height,
                                 rs.format.y8, self.ir_fps)
            config.enable_stream(rs.stream.infrared, 2, self.ir_width, self.ir_height,
                                 rs.format.y8, self.ir_fps)
            rospy.loginfo("IR Stereo: {}x{}@{}".format(
                self.ir_width, self.ir_height, self.ir_fps))

        self.pipeline.start(config)

        # Disable IR emitter (important for Kalibr — dot pattern interferes)
        device = self.pipeline.get_active_profile().get_device()
        depth_sensor = device.first_depth_sensor()
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled,
                                    1 if self.emitter_enabled else 0)
            rospy.loginfo("IR emitter: {}".format("ON" if self.emitter_enabled else "OFF"))

    def run(self):
        rospy.loginfo("RealSense publisher started")
        while not rospy.is_shutdown():
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                stamp = rospy.Time.now()

                if self.enable_color:
                    cf = frames.get_color_frame()
                    if cf:
                        img = np.asanyarray(cf.get_data())
                        msg = numpy_to_image(img, "bgr8")
                        msg.header.stamp = stamp
                        msg.header.frame_id = "camera_color_optical_frame"
                        self.color_pub.publish(msg)

                if self.enable_ir:
                    ir1 = frames.get_infrared_frame(1)
                    if ir1:
                        img = np.asanyarray(ir1.get_data())
                        msg = numpy_to_image(img, "mono8")
                        msg.header.stamp = stamp
                        msg.header.frame_id = "camera_infra1_optical_frame"
                        self.infra1_pub.publish(msg)

                    ir2 = frames.get_infrared_frame(2)
                    if ir2:
                        img = np.asanyarray(ir2.get_data())
                        msg = numpy_to_image(img, "mono8")
                        msg.header.stamp = stamp
                        msg.header.frame_id = "camera_infra2_optical_frame"
                        self.infra2_pub.publish(msg)

            except Exception as e:
                rospy.logwarn_throttle(5, "Frame error: {}".format(e))

        self.pipeline.stop()


if __name__ == '__main__':
    try:
        node = RealSensePublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
