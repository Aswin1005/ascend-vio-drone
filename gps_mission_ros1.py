#!/usr/bin/env python3
"""
gps_mission_ros1.py  —  ROS1 Melodic / MAVROS  (ArduCopter + GPS)
==================================================================
Same missions as vio_mission_ros1.py but intended for GPS-based
comparison testing.

Differences from VIO version:
  • NO VIO divergence guard
  • NO /mavros/vision_pose/pose subscriber
  • Mission 2 uses local_position/pose from GPS-EKF directly
  • Takeoff can also optionally use GPS-based LOITER (no FLOWHOLD needed)

Run:  rosrun vio_bridge gps_mission_ros1.py
"""

import rospy
import threading
import math
import sys
import time
import signal
import termios
import tty
import select

from mavros_msgs.msg import State, OverrideRCIn, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range

# ──────────────────────────────────────────────────────────────
#  TUNABLE PARAMETERS
# ──────────────────────────────────────────────────────────────

MAVROS_NS           = "/mavros"
LOOP_RATE_HZ        = 10

# Takeoff
TAKEOFF_PWM_START   = 1400
TAKEOFF_PWM_MAX     = 1850
TAKEOFF_PWM_STEP    = 10
TAKEOFF_TICK_S      = 0.3
GROUND_ALT_OFFSET   = 0.10
TAKEOFF_TARGET_OFF  = 0.35


# Mission 1
M1_POST_TAKEOFF_HOVER_S = 10.0
M1_PITCH_FWD_PWM    = 1550
M1_FWD_TIME_S       = 5.0
M1_NEUTRAL_TIME_S   = 10.0
M1_PITCH_BWD_PWM    = 1450
M1_BWD_TIME_S       = 5.0
M1_POST_BWD_NEUTRAL_S = 10.0
M1_LAND_WAIT_S      = 20.0

# Mission 2
M2_ALT              = 1.5
M2_FWD_M            = 2.0
M2_SIDE_M           = 1.0
M2_WP_TOLERANCE     = 0.25
M2_WP_TIMEOUT_S     = 15.0

# ──────────────────────────────────────────────────────────────
#  GLOBALS
# ──────────────────────────────────────────────────────────────
state        = State()
local_pose   = PoseStamped()
rf_range     = 0.0
abort_flag   = threading.Event()
land_flag    = threading.Event()

rc_hb_active = threading.Event()
rc_hb_lock   = threading.Lock()
rc_hb_vals   = [1500, 1500, 1500, 1500]

rc_pub  = None
sp_pub  = None
arm_srv = None
mode_srv= None

# ──────────────────────────────────────────────────────────────
#  CALLBACKS
# ──────────────────────────────────────────────────────────────
def state_cb(msg):     global state;     state = msg
def local_pos_cb(msg): global local_pose; local_pose = msg
def rf_cb(msg):        global rf_range;  rf_range = msg.range

# ──────────────────────────────────────────────────────────────
#  LOGGING
# ──────────────────────────────────────────────────────────────
def log(msg):  rospy.loginfo(f"[MISSION] {msg}")
def warn(msg): rospy.logwarn(f"[WARN]    {msg}")
def err(msg):  rospy.logerr(f"[ERROR]   {msg}")

# ──────────────────────────────────────────────────────────────
#  MODE / ARM
# ──────────────────────────────────────────────────────────────
def set_mode(mode):
    try:
        res = mode_srv(custom_mode=mode)
        log(f"Mode → {mode}" if res.mode_sent else f"Mode {mode} FAILED")
        return res.mode_sent
    except Exception as e:
        err(f"set_mode: {e}"); return False

def arm(do_arm):
    if state.armed == do_arm:
        log("Drone is already " + ("armed." if do_arm else "disarmed."))
        return True
    try:
        res = arm_srv(value=do_arm)
        if res.success:
            log("Arming command ACCEPTED" if do_arm else "Disarming command ACCEPTED")
        else:
            warn(f"Arming command REJECTED (result code: {res.result})" if do_arm else f"Disarming command REJECTED (result code: {res.result})")
        return res.success
    except Exception as e:
        err(f"arm exception: {e}"); return False

def wait_mode(target, timeout=8.0):
    t0 = time.time(); r = rospy.Rate(10)
    while not rospy.is_shutdown() and not abort_flag.is_set():
        if state.mode == target: return True
        if time.time() - t0 > timeout: return False
        r.sleep()
    return False

def wait_armed(do_arm=True, timeout=8.0):
    t0 = time.time(); r = rospy.Rate(10)
    while not rospy.is_shutdown() and not abort_flag.is_set():
        if state.armed == do_arm: return True
        if time.time() - t0 > timeout: return False
        r.sleep()
    return False

def sleep_check(secs):
    t0 = time.time(); r = rospy.Rate(10)
    while not rospy.is_shutdown() and not abort_flag.is_set() and not land_flag.is_set():
        if time.time() - t0 >= secs: return True
        r.sleep()
    return False


# ──────────────────────────────────────────────────────────────
#  RC OVERRIDE
# ──────────────────────────────────────────────────────────────
def send_rc(roll=1500, pitch=1500, throttle=1500, yaw=1500):
    msg = OverrideRCIn(); msg.channels = [0]*18
    msg.channels[0] = roll;  msg.channels[1] = pitch
    msg.channels[2] = throttle; msg.channels[3] = yaw
    rc_pub.publish(msg)

def clear_rc():
    rc_hb_active.clear()
    msg = OverrideRCIn(); msg.channels = [0]*18
    rc_pub.publish(msg)

def set_rc_hb(roll=1500, pitch=1500, throttle=1500, yaw=1500):
    with rc_hb_lock: rc_hb_vals[:] = [roll, pitch, throttle, yaw]

def start_rc_heartbeat(throttle=1500):
    set_rc_hb(throttle=throttle)
    if not rc_hb_active.is_set():
        rc_hb_active.set()
        threading.Thread(target=_rc_hb_thread, daemon=True).start()
    log("RC heartbeat started.")

def _rc_hb_thread():
    while rc_hb_active.is_set() and not rospy.is_shutdown():
        with rc_hb_lock: r, p, t, y = rc_hb_vals
        send_rc(r, p, t, y)
        time.sleep(0.2)

# ──────────────────────────────────────────────────────────────
#  LAND
# ──────────────────────────────────────────────────────────────
def do_land():
    log("LAND — clearing RC override.")
    clear_rc(); time.sleep(0.1); set_mode("LAND")

def abort_land(reason=""):
    err(f"ABORT: {reason}"); abort_flag.set(); do_land()

# ──────────────────────────────────────────────────────────────
#  KEYBOARD ('l' → land)
# ──────────────────────────────────────────────────────────────
_orig_tc = None
def _enable_raw_kb():
    global _orig_tc
    try:
        fd = sys.stdin.fileno()
        _orig_tc = termios.tcgetattr(fd)
        tty.setraw(fd)
    except Exception: pass

def _restore_kb():
    try:
        if _orig_tc: termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, _orig_tc)
    except Exception: pass

def _kb_thread():
    _enable_raw_kb()
    while not abort_flag.is_set() and not rospy.is_shutdown():
        try:
            dr, _, _ = select.select([sys.stdin], [], [], 0.1)
            if dr:
                ch = sys.stdin.read(1)
                if ch.lower() == 'l':
                    log("'l' pressed → landing."); land_flag.set()
        except Exception: break
    _restore_kb()

# ──────────────────────────────────────────────────────────────
#  POSITION HELPERS
# ──────────────────────────────────────────────────────────────
def publish_ned(north, east, alt_up, yaw_rad=0.0):
    pt = PositionTarget()
    pt.header.stamp = rospy.Time.now()
    pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    pt.type_mask = (
        PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
        PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
        PositionTarget.IGNORE_YAW_RATE
    )
    pt.position.x = north   # ENU y → NED x (North)
    pt.position.y = east    # ENU x → NED y (East)
    pt.position.z = alt_up
    pt.yaw = yaw_rad
    sp_pub.publish(pt)

def get_pos():
    """Return (north, east, alt) from ENU /mavros/local_position/pose."""
    p = local_pose.pose.position
    return p.y, p.x, p.z

def get_yaw():
    q = local_pose.pose.orientation
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))

# ──────────────────────────────────────────────────────────────
#  RC OVERRIDE TAKEOFF
# ──────────────────────────────────────────────────────────────
def rc_takeoff(arm_mode, wait_str):
    log(f"Switch to {arm_mode}")
    if not set_mode(arm_mode) or not wait_mode(wait_str): return False
    log("Arming..."); 
    if not arm(True) or not wait_armed(True): return False

    log("Takeoff ramp...")
    ground     = rf_range
    detect_alt = ground + GROUND_ALT_OFFSET
    target_alt = ground + TAKEOFF_TARGET_OFF
    rospy.loginfo(f"[TAKEOFF] ground={ground:.2f} detect={detect_alt:.2f} target={target_alt:.2f}")

    has_liftoff = False; frozen_pwm = TAKEOFF_PWM_START
    for pwm in range(TAKEOFF_PWM_START, TAKEOFF_PWM_MAX + 1, TAKEOFF_PWM_STEP):
        if abort_flag.is_set() or land_flag.is_set(): return False
        curr = rf_range
        if not has_liftoff and curr > detect_alt:
            has_liftoff = True; frozen_pwm = pwm
            log(f"[TAKEOFF] Liftoff! Freezing at {frozen_pwm}")
        active = frozen_pwm if has_liftoff else pwm
        rospy.loginfo(f"[TAKEOFF] alt={curr:.2f} loop={pwm} active={active}")
        if curr >= target_alt:
            log("[TAKEOFF] Target altitude reached!"); send_rc(throttle=1500); return True
        if state.mode not in (wait_str, arm_mode):
            warn("Mode changed during takeoff."); return False
        send_rc(throttle=active); time.sleep(TAKEOFF_TICK_S)
    warn("Takeoff ramp exhausted without reaching target altitude. Continuing anyway."); send_rc(throttle=1500); return True

# ──────────────────────────────────────────────────────────────
#  MISSION 1  – GPS version (LOITER instead of FLOWHOLD option)
# ──────────────────────────────────────────────────────────────
def run_mission1():
    log("=" * 40); log("MISSION 1 (GPS) START"); log("=" * 40)

    while True:
        choice = input("\nTakeoff mode:\n  1 - FLOWHOLD (optical flow)\n  2 - LOITER   (GPS)\nEnter (1 or 2): ").strip()
        if choice == "1": arm_mode, wait_str = "22", "CMODE(22)"; break
        elif choice == "2": arm_mode, wait_str = "LOITER", "LOITER"; break

    threading.Thread(target=_kb_thread, daemon=True).start()

    if not rc_takeoff(arm_mode, wait_str):
        abort_land("Takeoff failed"); return

    start_rc_heartbeat(1500)

    # Hover post-takeoff before pitch forward
    log(f"Hovering for {M1_POST_TAKEOFF_HOVER_S}s before pitch forward...")
    if not sleep_check(M1_POST_TAKEOFF_HOVER_S): do_land(); return

    # Switch to LOITER for pitch sweep
    if state.mode != "LOITER":
        log("Switching to LOITER...")
        if not set_mode("LOITER") or not wait_mode("LOITER"):
            abort_land("LOITER failed"); return

    # Pitch forward
    log(f"Pitch forward ({M1_PITCH_FWD_PWM}) for {M1_FWD_TIME_S}s...")
    set_rc_hb(pitch=M1_PITCH_FWD_PWM, throttle=1500)
    if not sleep_check(M1_FWD_TIME_S): do_land(); return

    # Neutral hold
    log(f"Neutral hold for {M1_NEUTRAL_TIME_S}s...")
    set_rc_hb(1500, 1500, 1500, 1500)
    if not sleep_check(M1_NEUTRAL_TIME_S): do_land(); return

    # Pitch backward
    log(f"Pitch backward ({M1_PITCH_BWD_PWM}) for {M1_BWD_TIME_S}s...")
    set_rc_hb(pitch=M1_PITCH_BWD_PWM, throttle=1500)
    if not sleep_check(M1_BWD_TIME_S): do_land(); return

    # Neutral hold after backward
    log(f"Neutral hold for {M1_POST_BWD_NEUTRAL_S}s...")
    set_rc_hb(1500, 1500, 1500, 1500)
    if not sleep_check(M1_POST_BWD_NEUTRAL_S): do_land(); return

    # Land
    log("Landing..."); do_land()
    sleep_check(M1_LAND_WAIT_S); arm(False)
    log("=" * 40); log("MISSION 1 (GPS) COMPLETE"); log("=" * 40)

# ──────────────────────────────────────────────────────────────
#  MISSION 2  – GPS GUIDED zigzag
# ──────────────────────────────────────────────────────────────
def run_mission2():
    log("=" * 40); log("MISSION 2 (GPS) START"); log("=" * 40)

    while True:
        choice = input("\nTakeoff mode for Mission 2:\n  1 - FLOWHOLD (optical flow)\n  2 - LOITER   (GPS)\nEnter (1 or 2): ").strip()
        if choice == "1": arm_mode, wait_str = "22", "CMODE(22)"; break
        elif choice == "2": arm_mode, wait_str = "LOITER", "LOITER"; break

    threading.Thread(target=_kb_thread, daemon=True).start()

    if not rc_takeoff(arm_mode, wait_str):
        abort_land("Takeoff failed"); return

    start_rc_heartbeat(1500)

    # Hover post-takeoff before GUIDED transition
    log(f"Hovering for {M1_POST_TAKEOFF_HOVER_S}s before GUIDED switch...")
    if not sleep_check(M1_POST_TAKEOFF_HOVER_S):
        do_land(); return

    rospy.sleep(1.0)

    home_n, home_e, _ = get_pos()
    home_alt = M2_ALT
    home_yaw = get_yaw()
    rospy.loginfo(f"[M2-GPS] Home: N={home_n:.3f} E={home_e:.3f} Alt={home_alt:.2f}")

    # Pre-stream before GUIDED
    log("Pre-streaming 3s..."); t0 = time.time(); r = rospy.Rate(10)
    while time.time() - t0 < 3.0 and not abort_flag.is_set() and not land_flag.is_set():
        publish_ned(home_n, home_e, home_alt, home_yaw); r.sleep()

    log("Switching to GUIDED...")
    if not set_mode("GUIDED") or not wait_mode("GUIDED"):
        abort_land("GUIDED failed"); return

    # Anchor 2s
    t0 = time.time()
    while time.time() - t0 < 2.0 and not abort_flag.is_set() and not land_flag.is_set():
        publish_ned(home_n, home_e, home_alt, home_yaw); r.sleep()

    # Waypoints
    wps = [
        (home_n + M2_FWD_M, home_e,             home_alt, "Forward"),
        (home_n + M2_FWD_M, home_e + M2_SIDE_M, home_alt, "Right"),
        (home_n,            home_e + M2_SIDE_M,  home_alt, "Back"),
    ]

    def goto_wp(n, e, alt, label):
        log(f"→ {label}: N={n:.2f} E={e:.2f}")
        t0 = time.time()
        while not abort_flag.is_set() and not land_flag.is_set() and not rospy.is_shutdown():
            publish_ned(n, e, alt, home_yaw)
            cn, ce, _ = get_pos()
            if math.sqrt((cn-n)**2 + (ce-e)**2) < M2_WP_TOLERANCE:
                log(f"Reached {label}"); return True
            if time.time() - t0 > M2_WP_TIMEOUT_S:
                warn(f"Timeout: {label}"); return False
            r.sleep()
        return False

    for wn, we, wa, wl in wps:
        if not goto_wp(wn, we, wa, wl):
            warn(f"Failed at {wl}, continuing to next waypoint...")

    # Return to LOITER then land
    log("Switching to LOITER before land..."); set_mode("LOITER")
    sleep_check(5.0); do_land()
    sleep_check(M1_LAND_WAIT_S); arm(False)
    log("=" * 40); log("MISSION 2 (GPS) COMPLETE"); log("=" * 40)

# ──────────────────────────────────────────────────────────────
#  SIGINT
# ──────────────────────────────────────────────────────────────
def sigint_handler(sig, frame):
    warn("Ctrl+C → Emergency land!")
    abort_flag.set(); do_land(); time.sleep(3.0); arm(False)
    _restore_kb(); sys.exit(0)

# ──────────────────────────────────────────────────────────────
#  MAIN
# ──────────────────────────────────────────────────────────────
def main():
    global rc_pub, sp_pub, arm_srv, mode_srv

    rospy.init_node("gps_mission", anonymous=False)
    signal.signal(signal.SIGINT, sigint_handler)

    rospy.Subscriber(f"{MAVROS_NS}/state",                   State,       state_cb)
    rospy.Subscriber(f"{MAVROS_NS}/local_position/pose",     PoseStamped, local_pos_cb)
    rospy.Subscriber(f"{MAVROS_NS}/rangefinder/rangefinder", Range,       rf_cb)

    rc_pub  = rospy.Publisher(f"{MAVROS_NS}/rc/override",        OverrideRCIn,   queue_size=5)
    sp_pub  = rospy.Publisher(f"{MAVROS_NS}/setpoint_raw/local", PositionTarget, queue_size=5)

    log("Waiting for services...")
    rospy.wait_for_service(f"{MAVROS_NS}/cmd/arming")
    rospy.wait_for_service(f"{MAVROS_NS}/set_mode")
    arm_srv  = rospy.ServiceProxy(f"{MAVROS_NS}/cmd/arming", CommandBool)
    mode_srv = rospy.ServiceProxy(f"{MAVROS_NS}/set_mode",   SetMode)

    log("Waiting for MAVROS heartbeat...")
    r = rospy.Rate(2)
    while not rospy.is_shutdown() and not state.connected: r.sleep()
    log("Connected.")
    rospy.sleep(1.0)

    mission = 0
    while mission not in (1, 2) and not rospy.is_shutdown():
        try:
            mission = int(input("\nSelect mission:\n  1 - RC Takeoff + LOITER pitch sweep (GPS)\n  2 - GUIDED zigzag (GPS)\nEnter (1 or 2): ").strip())
        except ValueError: pass

    if mission == 1: run_mission1()
    elif mission == 2: run_mission2()

    _restore_kb()
    rospy.signal_shutdown("Done.")

if __name__ == "__main__":
    main()
