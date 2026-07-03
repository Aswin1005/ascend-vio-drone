from pymavlink import mavutil
import time
import keyboard
import signal
import threading
import paramiko # Add to imports at the top
from scp import SCPClient # pip install scp

#connection_string = 'COM15'
connection_string = '/dev/ttyACM0'  # Replace with your port
baud_rate = 57600
rangefinder_distance = 0.0

# Connect to Pixhawk
print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
master.wait_heartbeat()
print("Heartbeat received! Pixhawk is connected.")

def request_rangefinder_data():
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,  # Request rangefinder data
        10,  # Frequency (Hz)
        1
    )
request_rangefinder_data()

def read_rangefinder_data():
    global rangefinder_distance
    while True:
        msg = master.recv_match(type="RANGEFINDER", blocking=False)  # Non-blocking
        if msg:
            rangefinder_distance = msg.distance
        time.sleep(0.1)  # Prevent CPU overuse

# Start Rangefinder Data Retrieval in a Separate Thread
rangefinder_thread = threading.Thread(target=read_rangefinder_data)
rangefinder_thread.daemon = True
rangefinder_thread.start()

def get_mode():
    try:
        mode = master.flightmode
        print(f" - Current mode: {mode}")
        return mode
    except Exception as e:
        print(f"Error getting mode: {e}")
        return None

def set_mode(mode):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode set to {mode}")
    time.sleep(2)

def clear_rc_override():
    """Release RC override so autopilot has full control"""
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        0, 0, 0, 0,  # 0 = release override on all channels
        0, 0, 0, 0
    )
    time.sleep(0.1)

def land_drone():
    print("Landing initiated...")
    clear_rc_override()        # ← Release throttle override first
    time.sleep(0.3)            # ← Brief pause to let autopilot resume control
    set_mode("LAND")

def disarm_drone():
    print("Disarming drone...")
    master.arducopter_disarm()
    print("Drone disarmed.")

def send_rc_override(throttle_pwm):
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        1500, 1500, throttle_pwm, 1500,
        0, 0, 0, 0
    )

def takeoff():
    print("Taking off...")
    count = 1
    for pwm in range(1400, 1850, 10):
        curr_alt = rangefinder_distance  # Real-time updated altitude
        print(f"Current Altitude: {curr_alt:.2f}m")
        
        if curr_alt > 0.40:
            pwm -= 10 * count
            count += 1
        if curr_alt >= 0.6:
            set_mode("FLOWHOLD")
            send_rc_override(1550)
            break
        if get_mode() != 'FLOWHOLD':
            set_mode("LAND")
            break
        if keyboard.is_pressed("l"):
            break
        send_rc_override(pwm)
        print(f"Throttle PWM: {pwm}")
        time.sleep(0.3)

def emergency_shutdown():
    print("\n[CTRL+C DETECTED] Emergency landing and disarm initiated!")
    land_drone()
    disarm_drone()
    print("Exiting safely.")
    exit(0)

keyboard.add_hotkey("l", land_drone)
keyboard.add_hotkey("q", disarm_drone)
signal.signal(signal.SIGINT, lambda sig, frame: emergency_shutdown())

set_mode("FLOWHOLD")

print("Disabling safety switch...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER,
    0, 220, 0, 0, 0, 0, 0, 0
)
time.sleep(2)
print("Safety switch disabled.")

print("Arming drone...")
master.arducopter_arm()
master.motors_armed_wait()
print("Motors armed.")

takeoff()
# if rangefinder_distance >= 1.2:
#             set_mode("FLOWHOLD")
#             send_rc_override(1550)
time.sleep(10)
land_drone()
time.sleep(10)

print("Disarming...")
master.arducopter_disarm()
print("Drone disarmed.")

def transfer_file_to_base(local_path, remote_path, server_ip, username, password):
    print(f"Initiating data transfer to {server_ip}...")
    try:
        # Create SSH client
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(server_ip, port=22, username=username, password=password)

        # SCP the file
        with SCPClient(ssh.get_transport()) as scp:
            scp.put(local_path, remote_path)
        
        print("Transfer Complete: File moved to Base Station.")
        ssh.close()
    except Exception as e:
        print(f"Transfer failed: {e}")

# MISSION EXTENSION:
# Replace with your actual laptop IP and credentials
BASE_STATION_IP = "10.195.189.133" 
BASE_USER = "shehin"
BASE_PWD = "lmlm"
FILE_TO_SEND = "/home/isro/mission_data.txt"
DESTINATION = "/home/shehin/Desktop/isro/" # Or Linux path if laptop is Linux

transfer_file_to_base(FILE_TO_SEND, DESTINATION, BASE_STATION_IP, BASE_USER, BASE_PWD)

print("System Shutdown.")


"""
ADDITION TO YOUR EXISTING MISSION CODE
=======================================
Add these two blocks to your drone_mission.py — nothing else changes.

BLOCK 1 — paste at the top with your other imports + config
BLOCK 2 — paste after master.wait_heartbeat(), before set_mode("FLOWHOLD")
BLOCK 3 — replace your existing transfer_file_to_base() with this wrapped version

That's all.  The original mission logic is untouched.
"""

# ── BLOCK 1 ─ add to imports / config section ─────────────────────────────────

import socket as _socket
import json as _json
import threading as _threading

BASE_STATION_IP = "172.16.37.43"   # already in your code — reuse this
STATUS_PORT     = 14560            # UDP port GCS listens on (pick any free port)

_status = {
    "mission_phase":     "PRE-FLIGHT",
    "last_msg":          "Starting up",
    "transfer_state":    "WAITING",
    "transfer_file":     "---",
    "transfer_progress": "---",
}
_status_lock = _threading.Lock()
_status_sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)


def _send_status():
    """Broadcast current status to GCS every second."""
    while True:
        try:
            with _status_lock:
                pkt = _json.dumps(_status).encode()
            _status_sock.sendto(pkt, (BASE_STATION_IP, STATUS_PORT))
        except Exception:
            pass
        import time as _t; _t.sleep(1)


def _upd(phase=None, msg=None, xstate=None, xfile=None, xprog=None):
    """Update status fields (call from anywhere in the mission)."""
    with _status_lock:
        if phase  is not None: _status["mission_phase"]     = phase
        if msg    is not None: _status["last_msg"]           = msg
        if xstate is not None: _status["transfer_state"]    = xstate
        if xfile  is not None: _status["transfer_file"]     = xfile
        if xprog  is not None: _status["transfer_progress"] = xprog


# ── BLOCK 2 ─ paste after master.wait_heartbeat() ─────────────────────────────

_threading.Thread(target=_send_status, daemon=True).start()


# ── BLOCK 3 ─ replace your existing transfer_file_to_base() ───────────────────
# (keep all the paramiko/SCP logic, just add the _upd() calls around it)

def transfer_file_to_base(local_path, remote_path, server_ip, username, password):
    import paramiko
    from scp import SCPClient

    _upd(phase="DATA TRANSFER", msg="Initiating SCP transfer",
         xstate="CONNECTING", xfile=local_path.split("/")[-1])

    print(f"Initiating data transfer to {server_ip}...")
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(server_ip, port=22, username=username, password=password)

        _upd(xstate="TRANSFERRING", msg="SSH connected, sending file")

        def _progress(filename, size, sent):
            pct = int(sent * 100 / size) if size else 0
            _upd(xprog=f"{pct} %", msg=f"Sending {filename.decode() if isinstance(filename, bytes) else filename}")

        with SCPClient(ssh.get_transport(), progress=_progress) as scp:
            scp.put(local_path, remote_path)

        _upd(xstate="DONE", xprog="100 %", msg="Transfer complete")
        print("Transfer Complete: File moved to Base Station.")
        ssh.close()

    except Exception as e:
        _upd(xstate="FAILED", msg=f"Transfer error: {e}")
        print(f"Transfer failed: {e}")


# ── OPTIONAL: sprinkle _upd() calls in existing functions for phase tracking ──
# Example — add these one-liners to your existing functions:
#
#   set_mode("FLOWHOLD")       →  after this:  _upd(phase="PRE-FLIGHT", msg="FLOWHOLD set")
#   master.arducopter_arm()    →  after this:  _upd(phase="ARMING",     msg="Motors arming")
#   motors_armed_wait()        →  after this:  _upd(phase="ARMED",      msg="Motors armed")
#   inside takeoff():          →  add:         _upd(phase="TAKEOFF",    msg=f"PWM {pwm}, alt {curr_alt:.2f}m")
#   time.sleep(300)            →  before this: _upd(phase="AIRBORNE",   msg="Holding position")
#   land_drone()               →  add inside:  _upd(phase="LANDING",    msg="Land mode set")
#   arducopter_disarm()        →  after this:  _upd(phase="DISARMED",   msg="Disarmed OK")
