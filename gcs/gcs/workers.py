"""
workers.py  —  QThread workers for streaming SSH commands to the GUI
"""

import re
import select
import time

from PyQt5.QtCore import QThread, pyqtSignal


class StreamWorker(QThread):
    """
    Runs a remote SSH command and emits its output line-by-line.
    Signals:
      line_received(tag, line)  — a new line of output
      finished(tag, exit_code)  — process exited
      error(tag, msg)           — SSH/IO error
    """

    line_received = pyqtSignal(str, str)   # (tag, line)
    finished = pyqtSignal(str, int)        # (tag, exit_code)
    error = pyqtSignal(str, str)           # (tag, error_msg)

    def __init__(self, ssh_mgr, cmd: str, tag: str, use_pty: bool = True, parent=None):
        super().__init__(parent)
        self.ssh_mgr = ssh_mgr
        self.cmd = cmd
        self.tag = tag
        self.use_pty = use_pty
        self._stop_flag = False
        self._channel = None
        self._stdin = None   # for non-pty mode

    def run(self):
        try:
            if self.use_pty:
                channel = self.ssh_mgr.open_streaming_channel(self.cmd)
                self._channel = channel
                channel.setblocking(False)
                buf = b""
                while not self._stop_flag:
                    if channel.exit_status_ready():
                        # Drain remaining output
                        try:
                            remaining = channel.recv(65536)
                            if remaining:
                                buf += remaining
                        except Exception:
                            pass
                        break
                    try:
                        ready = select.select([channel], [], [], 0.1)[0]
                        if ready:
                            data = channel.recv(4096)
                            if not data:
                                break
                            buf += data
                            # emit complete lines
                            while b"\n" in buf:
                                idx = buf.index(b"\n")
                                line = buf[:idx].decode(errors="replace").rstrip("\r")
                                buf = buf[idx + 1:]
                                if line:
                                    self.line_received.emit(self.tag, line)
                    except Exception:
                        time.sleep(0.05)
                # emit any remaining
                if buf:
                    line = buf.decode(errors="replace").rstrip("\r\n")
                    if line:
                        self.line_received.emit(self.tag, line)
                exit_code = channel.recv_exit_status() if not self._stop_flag else -1
                self.finished.emit(self.tag, exit_code)
            else:
                stdin, stdout, stderr = self.ssh_mgr.open_raw_channel(self.cmd)
                self._stdin = stdin
                stdout.channel.setblocking(False)
                for line in iter(lambda: stdout.readline(), ""):
                    if self._stop_flag:
                        break
                    line = line.rstrip("\n\r")
                    if line:
                        self.line_received.emit(self.tag, line)
                exit_code = stdout.channel.recv_exit_status()
                self.finished.emit(self.tag, exit_code)
        except Exception as e:
            self.error.emit(self.tag, str(e))

    def stop(self):
        self._stop_flag = True
        if self._channel:
            try:
                self._channel.close()
            except Exception:
                pass
        if self._stdin:
            try:
                self._stdin.close()
            except Exception:
                pass

    def send_stdin(self, text: str):
        """Send text to the process stdin (only works with PTY channel)."""
        if self._channel:
            try:
                self._channel.send(text.encode())
            except Exception:
                pass


class AllTopicsHzWorker(QThread):
    """
    Polls the Hz rate of all monitored topics in a single SSH session.

    Instead of opening 4 separate SSH channels (one per rostopic hz), this
    worker runs a single bash script that executes all four rostopic hz
    commands concurrently as background subshells and collects the results.
    This uses only 1 SSH channel for 4 topics, solving the channel-limit issue.

    Signals:
      rate_updated(topic, hz_float)  — a topic has a valid rate
      no_messages(topic)             — a topic has no data
      error_signal(msg)              — SSH or parse error
    """

    rate_updated = pyqtSignal(str, float)   # (topic, hz)
    no_messages = pyqtSignal(str)           # topic with no data
    error_signal = pyqtSignal(str)          # error message

    # Poll duration per cycle. Each rostopic hz subprocess times out after this
    # many seconds. A window of 10 messages is enough for a stable average and
    # converges quickly for all topics (0.07s for 150Hz IMU, 0.33s for 30fps cams).
    POLL_DURATION = 5
    HZ_WINDOW = 10

    def __init__(self, ssh_mgr, topics: list, parent=None):
        super().__init__(parent)
        self.ssh_mgr = ssh_mgr
        self.topics = topics
        self._stop_flag = False

    def run(self):
        ros_source = f"{self.ssh_mgr.ros_env_prefix}source /opt/ros/melodic/setup.bash && source ~/catkin_ws/devel/setup.bash"

        # Build a single bash script that runs each rostopic hz in the background,
        # collecting output into separate temp files, then cats them with headers.
        # All subshells share one SSH session — only 1 channel used for 4 topics.
        #
        # IMPORTANT redirect order: '> file 2>&1' puts BOTH stdout and stderr into
        # the temp file. The wrong order '2>&1 > file' sends stderr to SSH stdout,
        # which poisons the cat output we parse below.
        topic_cmds = []
        for topic in self.topics:
            safe = topic.replace("/", "_").lstrip("_")
            topic_cmds.append(
                f"( timeout {self.POLL_DURATION} rostopic hz {topic} -w {self.HZ_WINDOW} "
                f"> /tmp/_hz_{safe} 2>&1 ) &"
            )

        parallel_block = " ".join(topic_cmds)
        # Print each file with a unique header so the Python parser can split them.
        cat_block = " ".join(
            f"echo '===TOPIC:{topic}==='; cat /tmp/_hz_{topic.replace('/', '_').lstrip('_')} 2>/dev/null;"
            for topic in self.topics
        )
        cmd = (
            f"bash -c '{ros_source} && "
            f"{parallel_block} "
            f"wait; "
            f"{cat_block}'"
        )

        while not self._stop_flag:
            try:
                total_timeout = self.POLL_DURATION + 5
                _, out, _ = self.ssh_mgr.exec(cmd, timeout=total_timeout)
                if self._stop_flag:
                    break

                # Parse the combined output — split by ===TOPIC:<name>=== headers
                topic_outputs: dict[str, list[str]] = {}
                current_topic = None
                for line in out.splitlines():
                    m = re.match(r"===TOPIC:(.+?)===", line)
                    if m:
                        current_topic = m.group(1)
                        topic_outputs[current_topic] = []
                    elif current_topic is not None:
                        topic_outputs[current_topic].append(line)

                for topic in self.topics:
                    lines = topic_outputs.get(topic, [])
                    block = "\n".join(lines)
                    hz = self._parse_hz(block)
                    if hz is not None:
                        self.rate_updated.emit(topic, hz)
                    else:
                        self.no_messages.emit(topic)

            except Exception as e:
                if not self._stop_flag:
                    self.error_signal.emit(str(e))
                time.sleep(2)

    def _parse_hz(self, output: str) -> float | None:
        """
        Return the LAST 'average rate: X.XXX' found in the output.
        rostopic hz prints multiple readings over its run time;
        the last one is the most settled (window fully populated).
        Skip warning/error lines.
        """
        last_hz = None
        for line in output.splitlines():
            if "warning" in line.lower() or "error" in line.lower():
                continue
            m = re.search(r"average rate:\s*([\d.]+)", line)
            if m:
                try:
                    last_hz = float(m.group(1))
                except ValueError:
                    pass
        return last_hz

    def stop(self):
        self._stop_flag = True


class TopicEchoWorker(QThread):
    """
    Continuously runs `rostopic echo <topic>` and emits parsed output.

    Signals:
      data_received(topic, raw_text)
      error_signal(topic, msg)
    """

    data_received = pyqtSignal(str, str)
    error_signal = pyqtSignal(str, str)

    def __init__(self, ssh_mgr, topic: str, parent=None):
        super().__init__(parent)
        self.ssh_mgr = ssh_mgr
        self.topic = topic
        self._stop_flag = False
        self._channel = None

    def run(self):
        ros_source = f"{self.ssh_mgr.ros_env_prefix}source /opt/ros/melodic/setup.bash && source ~/catkin_ws/devel/setup.bash"
        cmd = f"bash -c '{ros_source} && rostopic echo {self.topic}'"
        try:
            channel = self.ssh_mgr.open_streaming_channel(cmd)
            self._channel = channel
            channel.setblocking(False)
            buf = b""
            block_lines = []
            in_block = False

            while not self._stop_flag:
                if channel.exit_status_ready():
                    break
                try:
                    ready = select.select([channel], [], [], 0.1)[0]
                    if ready:
                        data = channel.recv(4096)
                        if not data:
                            break
                        buf += data
                        while b"\n" in buf:
                            idx = buf.index(b"\n")
                            line = buf[:idx].decode(errors="replace").rstrip("\r")
                            buf = buf[idx + 1:]
                            if line == "---":
                                if block_lines:
                                    self.data_received.emit(self.topic, "\n".join(block_lines))
                                block_lines = []
                            else:
                                block_lines.append(line)
                except Exception:
                    time.sleep(0.05)
        except Exception as e:
            if not self._stop_flag:
                self.error_signal.emit(self.topic, str(e))

    def stop(self):
        self._stop_flag = True
        if self._channel:
            try:
                self._channel.close()
            except Exception:
                pass


class OpenVinsWatcher(QThread):
    """
    Polls for VIO initialization by checking if /mavros/vision_pose/pose
    is publishing.

    The moment MAVROS receives ANY message on this topic, VIO is initialized
    and feeding poses. We do NOT require non-zero x/y because the drone may
    be sitting exactly at its arm position (x=0, y=0 relative to origin).

    Signals:
      initialized()    — vision_pose/pose is actively publishing
      status_update(msg: str)
    """

    initialized = pyqtSignal()
    status_update = pyqtSignal(str)

    TOPIC = "/mavros/vision_pose/pose"
    POLL_INTERVAL = 3.0

    def __init__(self, ssh_mgr, parent=None):
        super().__init__(parent)
        self.ssh_mgr = ssh_mgr
        self._stop_flag = False

    def run(self):
        ros_source = f"{self.ssh_mgr.ros_env_prefix}source /opt/ros/melodic/setup.bash && source ~/catkin_ws/devel/setup.bash"
        self.status_update.emit("Waiting for VIO initialization (vision_pose/pose)...")
        while not self._stop_flag:
            try:
                # Grab one message — if ANY message arrives, VIO is live.
                cmd = (
                    f"bash -c '{ros_source} && timeout 4 rostopic echo {self.TOPIC} "
                    f"-n 1 2>&1'"
                )
                code, out, _ = self.ssh_mgr.exec(cmd, timeout=6)
                if self._stop_flag:
                    return
                
                out_stripped = out.strip()
                if out_stripped:
                    # Check if there is an error in the output
                    if any(err_kw in out_stripped.lower() for err_kw in ["unable to communicate", "error", "cannot connect", "does not exist", "roscore"]):
                        self.status_update.emit(f"⚠️ ROS Error: {out_stripped.splitlines()[0]}")
                    else:
                        # Topic is publishing — VIO is initialized.
                        # Parse x/y just to show in the status message.
                        x, y = self._extract_xy(out_stripped)
                        if x is not None and y is not None:
                            self.status_update.emit(
                                f"\u2705 VIO initialized!  vision_pose/pose live  "
                                f"x={x:.3f}  y={y:.3f}"
                            )
                        else:
                            self.status_update.emit(
                                "\u2705 VIO initialized!  vision_pose/pose is publishing."
                            )
                        self.initialized.emit()
                        return
                else:
                    self.status_update.emit(
                        f"\u23f3 Waiting for VIO... ({self.TOPIC} not publishing yet)"
                    )
            except Exception as e:
                self.status_update.emit(f"⚠️ Watcher Exception: {str(e)}")
            time.sleep(self.POLL_INTERVAL)

    def _extract_xy(self, text: str):
        """Extract position x and y from a rostopic echo PoseStamped block."""
        x = y = None
        in_pose = False
        in_position = False
        for line in text.splitlines():
            s = line.strip()
            if s == "pose:":
                in_pose = True
                continue
            if in_pose and s == "position:":
                in_position = True
                continue
            if in_position:
                m = re.match(r"x:\s*([-\d.eE+]+)", s)
                if m:
                    try:
                        x = float(m.group(1))
                    except ValueError:
                        pass
                m = re.match(r"y:\s*([-\d.eE+]+)", s)
                if m:
                    try:
                        y = float(m.group(1))
                    except ValueError:
                        pass
                if x is not None and y is not None:
                    break
                if s.startswith("z:") and x is not None:
                    break
        return x, y

    def stop(self):
        self._stop_flag = True


class MavrosTelemetryWorker(QThread):
    """
    Streams live MAVROS telemetry at ~20 Hz via a persistent SSH channel.

    Runs a compact inline Python ROS subscriber on the Jetson that prints
    structured TELEM lines continuously. This is identical in architecture
    to the vision_pose streaming approach and avoids the 6-second polling
    lag of the old rostopic-echo-n1 method.

    Output format per line:
        TELEM battery_v=<v> battery_pct=<pct> pos_x=<x> pos_y=<y> pos_z=<z> vel_x=<vx> vel_y=<vy>

    Signals:
      telemetry_updated(data: dict)
      error_signal(msg: str)
    """

    telemetry_updated = pyqtSignal(dict)
    error_signal = pyqtSignal(str)

    # Compact inline Python subscriber that runs on the Jetson
    _REMOTE_SCRIPT = (
        "import rospy, sys, time\n"
        "from sensor_msgs.msg import BatteryState\n"
        "from geometry_msgs.msg import PoseStamped, TwistStamped\n"
        "d={}\n"
        "def cb_b(m): d.update(battery_v=f\\\"{m.voltage:.2f}\\\", battery_pct=f\\\"{m.percentage:.2f}\\\")\n"
        "def cb_p(m): d.update(pos_x=f\\\"{m.pose.position.x:.3f}\\\", pos_y=f\\\"{m.pose.position.y:.3f}\\\", pos_z=f\\\"{m.pose.position.z:.3f}\\\")\n"
        "def cb_v(m): d.update(vel_x=f\\\"{m.twist.linear.x:.3f}\\\", vel_y=f\\\"{m.twist.linear.y:.3f}\\\")\n"
        "rospy.init_node(\\\"gcs_telem_stream\\\",anonymous=True)\n"
        "rospy.Subscriber(\\\"/mavros/battery\\\",BatteryState,cb_b)\n"
        "rospy.Subscriber(\\\"/mavros/local_position/pose\\\",PoseStamped,cb_p)\n"
        "rospy.Subscriber(\\\"/mavros/local_position/velocity_local\\\",TwistStamped,cb_v)\n"
        "r=rospy.Rate(20)\n"
        "while not rospy.is_shutdown():\n"
        "  if d: print(\\\"TELEM \\\"+\\\" \\\".join(f\\\"{k}={v}\\\" for k,v in d.items()),flush=True)\n"
        "  r.sleep()\n"
    )

    def __init__(self, ssh_mgr, parent=None):
        super().__init__(parent)
        self.ssh_mgr = ssh_mgr
        self._stop_flag = False
        self._channel = None

    def run(self):
        ros_source = (
            f"{self.ssh_mgr.ros_env_prefix}"
            f"source /opt/ros/melodic/setup.bash && source ~/catkin_ws/devel/setup.bash"
        )
        cmd = f"bash -c '{ros_source} && python3 -c \"{self._REMOTE_SCRIPT}\"'"

        try:
            self._channel = self.ssh_mgr.open_streaming_channel(cmd)
            self._channel.setblocking(False)
            buf = b""

            while not self._stop_flag:
                try:
                    if self._channel.recv_ready():
                        chunk = self._channel.recv(4096)
                        if not chunk:
                            break
                        buf += chunk
                        while b"\n" in buf:
                            idx = buf.index(b"\n")
                            raw = buf[:idx].decode(errors="replace").strip()
                            buf = buf[idx + 1:]
                            if raw.startswith("TELEM "):
                                data = {}
                                for pair in raw[6:].split():
                                    if "=" in pair:
                                        k, v = pair.split("=", 1)
                                        data[k] = v
                                if data and not self._stop_flag:
                                    self.telemetry_updated.emit(data)
                            else:
                                if raw.strip():
                                    print(f"DEBUG MavrosTelemetryWorker: {raw}")
                    else:
                        time.sleep(0.05)
                        
                    if self._channel.exit_status_ready():
                        break
                except Exception as e:
                    print(f"DEBUG MavrosTelemetryWorker loop exception: {e}")
                    time.sleep(0.05)

        except Exception as e:
            if not self._stop_flag:
                self.error_signal.emit(str(e))

    def stop(self):
        self._stop_flag = True
        if self._channel:
            try:
                self._channel.close()
            except Exception:
                pass


class ChargingWorker(QThread):
    """
    Listens locally for UDP broadcast logs on port 12345 from the charging microcontroller.
    Parses live metrics (voltage, rise, elapsed time, status changes) and emits them.
    """
    log_received = pyqtSignal(str)
    status_changed = pyqtSignal(str)
    voltage_updated = pyqtSignal(float)
    metrics_updated = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._stop_flag = False
        self._sock = None

    def run(self):
        import socket
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._sock.bind(("", 12345))
        except Exception as e:
            self.log_received.emit(f"[SYSTEM ERROR] Failed to bind to UDP port 12345: {e}")
            return

        self.log_received.emit("[SYSTEM] Listening for ESP32 Charger logs on UDP port 12345...")
        self.status_changed.emit("Waiting for Charger...")

        while not self._stop_flag:
            try:
                self._sock.settimeout(0.5)
                data, addr = self._sock.recvfrom(4096)
                msg = data.decode("utf-8", errors="replace")
                
                # Split multiple lines if any
                for line in msg.splitlines():
                    line_str = line.strip()
                    if line_str:
                        self.log_received.emit(line_str)
                        self._parse_line(line_str)
            except socket.timeout:
                continue
            except Exception as e:
                if not self._stop_flag:
                    self.log_received.emit(f"[SYSTEM ERROR] {e}")
                break

        if self._sock:
            self._sock.close()

    def _parse_line(self, line: str):
        line_up = line.upper()
        if "EMERGENCY STOP" in line_up or "FATAL" in line_up or "FAULT" in line_up or "SSR CUT" in line_up:
            self.status_changed.emit("🚨 Emergency Stop")
        elif "STARTING CHARGE" in line_up or "CHARGING_START" in line_up or "CHARGING STARTED" in line_up:
            self.status_changed.emit("⚡ Charging...")
        elif "STOPPING CHARGE" in line_up or "CHARGING_STOP" in line_up or "TARGET CONFIRMED" in line_up:
            self.status_changed.emit("✅ Charging Complete")
        elif "WAITING FOR BATTERY" in line_up:
            self.status_changed.emit("⏳ Waiting for Battery...")
        elif "BATTERY CONNECTED" in line_up:
            self.status_changed.emit("🔋 Battery Connected")

        metrics = {}
        
        # Check for [LIVE] or [AVG] lines, e.g.:
        # [LIVE] 14.200V | Target: 15.500V | Rise: +0.200V | tHits: 0/1 | Time: 1m 30s
        # [AVG] 14.150V | Rise: +0.150V | tHits: 0/1
        m_live = re.search(r"\[(LIVE|AVG)\]\s*([\d.]+)\s*V", line)
        if m_live:
            v_val = float(m_live.group(2))
            metrics["voltage"] = v_val
            self.voltage_updated.emit(v_val)

        # Target voltage
        m_target = re.search(r"Target:\s*([\d.]+)\s*V", line, re.IGNORECASE)
        if m_target:
            metrics["target_voltage"] = float(m_target.group(1))
        elif "TARGET:" in line_up:
            m_target_2 = re.search(r"TARGET:\s*([\d.]+)\s*V", line_up)
            if m_target_2:
                metrics["target_voltage"] = float(m_target_2.group(1))

        # Start voltage
        m_start = re.search(r"(?:Start|Started)\s*:\s*([\d.]+)\s*V", line, re.IGNORECASE)
        if m_start:
            metrics["start_voltage"] = float(m_start.group(1))
        else:
            m_start_2 = re.search(r"Start:\s*([\d.]+)\s*V", line, re.IGNORECASE)
            if m_start_2:
                metrics["start_voltage"] = float(m_start_2.group(1))

        # Rise
        m_rise = re.search(r"Rise:\s*\+?([-\d.]+)\s*V", line, re.IGNORECASE)
        if m_rise:
            metrics["rise"] = float(m_rise.group(1))

        # Time
        m_time = re.search(r"Time:\s*([\w\s\d]+)$", line, re.IGNORECASE)
        if m_time:
            metrics["time_str"] = m_time.group(1).strip()

        if metrics:
            self.metrics_updated.emit(metrics)

    def stop(self):
        self._stop_flag = True




class SeedTransferWorker(QThread):
    finished = pyqtSignal(bool, str)  # (success, msg)
    progress = pyqtSignal(str)

    def __init__(self, ssh_mgr, remote_dir: str, local_dir: str, parent=None):
        super().__init__(parent)
        self.ssh_mgr = ssh_mgr
        self.remote_dir = remote_dir
        self.local_dir = local_dir

    def run(self):
        import os
        try:
            self.progress.emit("Starting seed image transfer via SFTP...")
            # Automatically download the original seeds if not already downloaded locally
            local_base = os.path.dirname(self.local_dir)
            local_seeds_dir = os.path.join(local_base, "seeds")
            
            # Recreate seeds directory
            if not os.path.exists(local_seeds_dir) or not os.listdir(local_seeds_dir):
                self.progress.emit("Downloading seed templates from Jetson...")
                remote_seeds = f"/home/{self.ssh_mgr.username}/seed_tracker/seeds/ascend_seeds"
                self.ssh_mgr.sftp_download_folder(remote_seeds, local_seeds_dir, lambda m: self.progress.emit(m))
            
            # Download captured detections folder
            self.progress.emit("Downloading captured seeds from Jetson...")
            self.ssh_mgr.sftp_download_folder(self.remote_dir, self.local_dir, lambda m: self.progress.emit(m))
            self.finished.emit(True, "Transfer complete")
        except Exception as e:
            self.finished.emit(False, str(e))


class BatchVerifyWorker(QThread):
    finished = pyqtSignal(bool, str)  # (success, stdout/report)
    progress = pyqtSignal(str)

    def __init__(self, data_dir: str, parent=None):
        super().__init__(parent)
        self.data_dir = data_dir

    def run(self):
        import os
        import sys
        import subprocess
        try:
            self.progress.emit("Running batch verification pipeline...")
            gcs_dir = os.path.dirname(os.path.abspath(__file__))
            script_path = os.path.join(gcs_dir, "batch_verify.py")
            
            cmd = [sys.executable, script_path, self.data_dir]
            
            # Run process and capture stdout/stderr. Set Cwd to gcs_dir so seeds/ and match_logs/ are resolved correctly.
            proc = subprocess.Popen(
                cmd,
                cwd=gcs_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            stdout_data, stderr_data = proc.communicate()
            
            if proc.returncode == 0:
                self.finished.emit(True, stdout_data)
            else:
                self.finished.emit(False, stderr_data or stdout_data)
        except Exception as e:
            self.finished.emit(False, str(e))
