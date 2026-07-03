import cv2
import numpy as np
import threading
import time


class FramePacket:
    def __init__(self, color, depth=None):
        self.color = color
        self.depth = depth


class OpenCVCapture:
    def __init__(self, camera_index=None, video_path=None, camera_gst=None, width=640, height=360):
        if video_path:
            self.cap = cv2.VideoCapture(video_path)
        elif camera_gst:
            self.cap = cv2.VideoCapture(camera_gst, cv2.CAP_GSTREAMER)
        else:
            index = 0 if camera_index is None else camera_index
            self.cap = cv2.VideoCapture(index)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if not self.cap.isOpened():
            raise RuntimeError("cant open camera bro")

    def read(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None
        return FramePacket(color=frame)

    def release(self):
        self.cap.release()


class RealSenseCapture:
    def __init__(self, width=1280, height=720, fps=6):
        try:
            import pyrealsense2 as rs
        except ImportError:
            raise ImportError(
                "pyrealsense2 library not found. Please install it using:\n"
                "  pip install pyrealsense2\n"
                "Or use standard camera capture via: --camera <index>"
            )
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.profile = self.pipeline.start(self.config)
        print(f"[realsense] started streaming at {width}x{height} @ {fps}fps")

    def read(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                return None
            color_image = np.asanyarray(color_frame.get_data())
            return FramePacket(color=color_image)
        except Exception as e:
            print(f"[realsense] warning: failed to read frame: {e}")
            return None

    def release(self):
        try:
            self.pipeline.stop()
            print("[realsense] stopped pipeline")
        except Exception:
            pass


class ROSCapture:
    """Reads frames from a ROS image topic.

    Use this when the realsense2_camera ROS node already owns the camera.
    Pass the topic via --ros-topic (e.g. /camera/color/image_raw).
    Supported encodings: mono8 (infra), bgr8, rgb8, 8UC3.
    mono8 frames are auto-converted to BGR so the rest of the pipeline is unchanged.

    read() blocks until a new frame arrives — it does NOT busy-spin, so it
    will not starve OpenVINS or other nodes of CPU time on the Jetson Nano.
    """

    def __init__(self, topic):
        try:
            import rospy
        except ImportError:
            raise ImportError(
                "rospy not found.\n"
                "Make sure ROS is sourced: source /opt/ros/melodic/setup.bash"
            )

        self._frame = None
        self._lock = threading.Lock()
        # Event is set every time a genuinely new frame arrives.
        self._new_frame_event = threading.Event()

        rospy.init_node("survey_ros_capture", anonymous=True, disable_signals=True)

        from sensor_msgs.msg import Image as Img
        self._sub = rospy.Subscriber(
            topic, Img, self._callback,
            queue_size=1,
            buff_size=262144,  # 256 KB — keeps buffer small so old frames aren't queued
        )
        print(f"[ros_capture] subscribed to: {topic}")

        # Wait up to 5s for the first frame
        if not self._new_frame_event.wait(timeout=5.0):
            raise RuntimeError(
                f"[ros_capture] no frames on '{topic}' after 5s. "
                "Is the ROS node running? Is the topic correct?"
            )
        with self._lock:
            shape = self._frame.shape
        print(f"[ros_capture] receiving frames OK  shape={shape}")

    def _callback(self, msg):
        try:
            enc = msg.encoding
            data = np.frombuffer(msg.data, dtype=np.uint8)

            if enc == "mono8":
                gray = data.reshape(msg.height, msg.width)
                frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            elif enc == "rgb8":
                frame = data.reshape(msg.height, msg.width, 3)
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif enc in ("bgr8", "8UC3"):
                frame = data.reshape(msg.height, msg.width, 3).copy()
            else:
                channels = len(data) // (msg.height * msg.width)
                print(f"[ros_capture] WARNING: unknown encoding '{enc}', channels={channels}")
                frame = data.reshape(msg.height, msg.width, channels).copy()
                if channels == 1:
                    frame = cv2.cvtColor(frame[:, :, 0], cv2.COLOR_GRAY2BGR)

            with self._lock:
                self._frame = frame
            self._new_frame_event.set()

        except Exception as e:
            print(f"[ros_capture] callback error: {e}")

    def read(self, timeout=2.0):
        """Block until a new frame arrives, then return it."""
        self._new_frame_event.clear()
        arrived = self._new_frame_event.wait(timeout=timeout)
        if not arrived:
            return None
        with self._lock:
            if self._frame is None:
                return None
            return FramePacket(color=self._frame.copy())

    def release(self):
        try:
            self._sub.unregister()
            print("[ros_capture] unsubscribed from topic")
        except Exception:
            pass
