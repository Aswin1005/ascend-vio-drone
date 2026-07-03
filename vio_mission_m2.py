#!/usr/bin/env python3
"""
vio_mission_m2.py  —  ROS1 Melodic / MAVROS  (ArduCopter)
===========================================================
Mission 2: GUIDED takeoff to 1.5m then waypoint navigation
  - GUIDED takeoff to M2_TAKEOFF_ALT
  - User selects hover mode: GUIDED hold or FLOWHOLD
  - Waypoints: North 2m (wait 5s) -> East 1m -> North 2m
  - Timeout WARNING (not abort) if waypoint takes > 10s
  - Land with 20s timeout then disarm

Safety:
  - Press 'l'  -> immediate LAND
  - Ctrl+C     -> LAND + disarm
  - Exception  -> LAND + disarm
  - VIO divergence guard
"""

import rospy, threading, math, sys, time, signal, termios, tty, select
from mavros_msgs.msg import State, OverrideRCIn, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range

# ──────────────────────────────────────────────────────────────
#  PARAMETERS
# ──────────────────────────────────────────────────────────────
MAVROS_NS           = "/mavros"

# GUIDED takeoff
M2_TAKEOFF_ALT      = 1.5    # target altitude (m)
M2_TAKEOFF_TIMEOUT  = 30.0   # max time to reach altitude (s)
M2_TAKEOFF_THRESH   = 0.90   # fraction of target alt to consider reached

# Hover after takeoff
M2_GUIDED_HOVER_S   = 2.0    # hover in GUIDED after takeoff (if chosen)

# Waypoints
M2_NORTH_1_M        = 2.0    # first north leg (m)
M2_EAST_M           = 1.0    # east leg (m)
M2_NORTH_2_M        = 2.0    # second north leg (m)
M2_WP_WAIT_S        = 5.0    # hold time at each waypoint (s)
M2_WP_RADIUS        = 0.30   # waypoint reached radius (m)
M2_WP_TIMEOUT_WARN  = 10.0   # warn if waypoint takes longer than this (s)

# Land
M2_LAND_WAIT_S      = 20.0   # wait after LAND before disarm

# VIO divergence guard
VIO_XY_LIMIT_M      = 400.0
VIO_CHECK_INTV_S    = 0.5

# ──────────────────────────────────────────────────────────────
#  GLOBALS
# ──────────────────────────────────────────────────────────────
state       = State()
local_pose  = PoseStamped()
vision_pose = PoseStamped()
rf_range    = 0.0
abort_flag  = threading.Event()
land_flag   = threading.Event()

rc_hb_active = threading.Event()
rc_hb_lock   = threading.Lock()
rc_hb_vals   = [1500, 1500, 1500, 1500]

rc_pub = sp_pub = arm_srv = mode_srv = None

# ──────────────────────────────────────────────────────────────
#  CALLBACKS
# ──────────────────────────────────────────────────────────────
def state_cb(msg):      global state;       state = msg
def local_pos_cb(msg):  global local_pose;  local_pose = msg
def vision_cb(msg):     global vision_pose; vision_pose = msg
def rf_cb(msg):         global rf_range;    rf_range = msg.range

# ──────────────────────────────────────────────────────────────
#  LOGGING
# ──────────────────────────────────────────────────────────────
def log(m):  rospy.loginfo(f"[M2] {m}")
def warn(m): rospy.logwarn(f"[WARN] {m}")
def err(m):  rospy.logerr(f"[ERROR] {m}")

# ──────────────────────────────────────────────────────────────
#  MAVROS HELPERS
# ──────────────────────────────────────────────────────────────
def set_mode(mode):
    try:
        res = mode_srv(custom_mode=mode)
        log(f"Mode -> {mode}" if res.mode_sent else f"Mode {mode} FAILED")
        return res.mode_sent
    except Exception as e:
        err(f"set_mode: {e}"); return False

def wait_mode(target, timeout=8.0):
    t0 = time.time(); r = rospy.Rate(10)
    while not rospy.is_shutdown() and not abort_flag.is_set():
        if state.mode == target: return True
        if time.time()-t0 > timeout: return False
        r.sleep()
    return False

def arm(do_arm):
    if state.armed == do_arm:
        log("Already " + ("armed." if do_arm else "disarmed.")); return True
    try:
        res = arm_srv(value=do_arm)
        if res.success: log("Arming ACCEPTED" if do_arm else "Disarming ACCEPTED")
        else:           warn(f"Arming REJECTED (code:{res.result})")
        return res.success
    except Exception as e:
        err(f"arm: {e}"); return False

def wait_armed(target=True, timeout=8.0):
    t0 = time.time(); r = rospy.Rate(10)
    while not rospy.is_shutdown() and not abort_flag.is_set():
        if state.armed == target: return True
        if time.time()-t0 > timeout: return False
        r.sleep()
    return False

def sleep_check(secs):
    t0 = time.time(); r = rospy.Rate(10)
    while not rospy.is_shutdown() and not abort_flag.is_set() and not land_flag.is_set():
        if time.time()-t0 >= secs: return True
        r.sleep()
    return False

def get_pos():
    p = local_pose.pose.position
    return p.y, p.x, p.z   # ENU: y=North, x=East, z=Up

def get_yaw():
    q = local_pose.pose.orientation
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2+q.z**2))

def publish_ned(north, east, alt_up, yaw_rad=0.0):
    pt = PositionTarget()
    pt.header.stamp = rospy.Time.now()
    pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    pt.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                    PositionTarget.IGNORE_VZ  | PositionTarget.IGNORE_AFX |
                    PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                    PositionTarget.IGNORE_YAW_RATE)
    pt.position.x = north  # FRAME_LOCAL_NED: x=North, y=East
    pt.position.y = east
    pt.position.z = alt_up
    pt.yaw = yaw_rad
    sp_pub.publish(pt)

# ──────────────────────────────────────────────────────────────
#  RC OVERRIDE (used only for clearing on land)
# ──────────────────────────────────────────────────────────────
def send_rc(roll=1500, pitch=1500, throttle=1500, yaw=1500):
    msg = OverrideRCIn(); msg.channels = [0]*18
    msg.channels[0]=roll; msg.channels[1]=pitch
    msg.channels[2]=throttle; msg.channels[3]=yaw
    rc_pub.publish(msg)

def clear_rc():
    rc_hb_active.clear()
    msg = OverrideRCIn(); msg.channels = [0]*18
    rc_pub.publish(msg)

# ──────────────────────────────────────────────────────────────
#  LAND
# ──────────────────────────────────────────────────────────────
def do_land():
    log("LAND mode — clearing RC override.")
    clear_rc(); time.sleep(0.1); set_mode("LAND")

def abort_land(reason=""):
    err(f"ABORT: {reason}"); abort_flag.set(); do_land()

def land_and_disarm():
    do_land()
    log(f"Waiting up to {M2_LAND_WAIT_S}s for auto-disarm...")
    t0 = time.time()
    while time.time()-t0 < M2_LAND_WAIT_S:
        if not state.armed: log("Auto-disarmed."); return
        time.sleep(0.5)
    log("Manual disarm after timeout."); arm(False)

# ──────────────────────────────────────────────────────────────
#  KEYBOARD 'l' -> LAND
# ──────────────────────────────────────────────────────────────
_orig_tc = None
def _enable_raw_kb():
    global _orig_tc
    try: fd=sys.stdin.fileno(); _orig_tc=termios.tcgetattr(fd); tty.setraw(fd)
    except: pass

def _restore_kb():
    try:
        if _orig_tc: termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, _orig_tc)
    except: pass

def _kb_thread():
    _enable_raw_kb()
    while not abort_flag.is_set() and not rospy.is_shutdown():
        try:
            dr,_,_ = select.select([sys.stdin],[],[],0.1)
            if dr:
                ch = sys.stdin.read(1)
                if ch.lower()=='l': log("'l' pressed -> land."); land_flag.set()
        except: break
    _restore_kb()

# ──────────────────────────────────────────────────────────────
#  VIO DIVERGENCE GUARD
# ──────────────────────────────────────────────────────────────
def _vio_guard_loop():
    r = rospy.Rate(1.0/VIO_CHECK_INTV_S)
    while not abort_flag.is_set() and not land_flag.is_set() and not rospy.is_shutdown():
        vx = vision_pose.pose.position.x
        vy = vision_pose.pose.position.y
        if abs(vx)>VIO_XY_LIMIT_M or abs(vy)>VIO_XY_LIMIT_M:
            abort_land(f"VIO divergence! x={vx:.2f} y={vy:.2f}"); break
        r.sleep()

def start_vio_guard():
    threading.Thread(target=_vio_guard_loop, daemon=True).start()
    log(f"VIO divergence guard started (limit +/-{VIO_XY_LIMIT_M}m).")

# ──────────────────────────────────────────────────────────────
#  GUIDED TAKEOFF
# ──────────────────────────────────────────────────────────────
def guided_takeoff():
    """Switch to GUIDED, arm, climb to M2_TAKEOFF_ALT. Returns True on success."""
    log("Step: Switching to GUIDED...")
    if not set_mode("GUIDED"): return False
    if not wait_mode("GUIDED"): return False

    log("Step: Arming...")
    if not arm(True): return False
    if not wait_armed(True): return False

    n, e, _ = get_pos()
    yaw = get_yaw()
    log(f"Step: Climbing to {M2_TAKEOFF_ALT}m...")

    rate = rospy.Rate(10)
    t0 = time.time()
    while time.time()-t0 < M2_TAKEOFF_TIMEOUT:
        if abort_flag.is_set() or land_flag.is_set(): return False
        publish_ned(n, e, M2_TAKEOFF_ALT, yaw)
        _, _, curr_alt = get_pos()
        if curr_alt >= M2_TAKEOFF_ALT * M2_TAKEOFF_THRESH:
            log(f"Target altitude reached ({curr_alt:.2f}m).")
            return True
        rate.sleep()

    warn(f"GUIDED takeoff timed out after {M2_TAKEOFF_TIMEOUT}s (at {get_pos()[2]:.2f}m). Continuing.")
    return True

# ──────────────────────────────────────────────────────────────
#  GOTO WAYPOINT
# ──────────────────────────────────────────────────────────────
def goto_wp(target_n, target_e, target_a, home_yaw, label="WP"):
    """
    Stream GUIDED setpoint to target. Warns if timeout exceeded.
    Returns True if reached, False if aborted.
    """
    log(f"GoTo {label}: N={target_n:.2f} E={target_e:.2f} A={target_a:.2f}")
    warned = False
    t0 = time.time()
    rate = rospy.Rate(10)

    while not abort_flag.is_set() and not land_flag.is_set() and not rospy.is_shutdown():
        publish_ned(target_n, target_e, target_a, home_yaw)
        n, e, a = get_pos()
        dist = math.sqrt((n-target_n)**2 + (e-target_e)**2 + (a-target_a)**2)
        elapsed = time.time()-t0

        if not warned and elapsed > M2_WP_TIMEOUT_WARN:
            warn(f"Waypoint {label} taking >{M2_WP_TIMEOUT_WARN}s (dist={dist:.2f}m). Continuing...")
            warned = True

        if dist < M2_WP_RADIUS:
            log(f"Reached {label} (dist={dist:.2f}m, t={elapsed:.1f}s).")
            return True

        rate.sleep()

    return False

# ──────────────────────────────────────────────────────────────
#  MISSION 2
# ──────────────────────────────────────────────────────────────
def run_mission2():
    log("="*40); log("MISSION 2 START"); log("="*40)

    # Hover mode after takeoff
    while True:
        c = input("\nHover mode after takeoff:\n  1 - GUIDED hold (2s)\n  2 - FLOWHOLD\nEnter (1 or 2): ").strip()
        if c=="1": hover_mode="guided"; break
        elif c=="2": hover_mode="flowhold"; break
        print("Invalid.")

    threading.Thread(target=_kb_thread, daemon=True).start()
    start_vio_guard()

    # GUIDED takeoff to M2_TAKEOFF_ALT
    if not guided_takeoff():
        abort_land("GUIDED takeoff failed"); return

    # Capture home position and yaw
    home_n, home_e, home_alt = get_pos()
    home_yaw = get_yaw()
    rospy.loginfo(f"[M2] Home: N={home_n:.3f} E={home_e:.3f} Alt={home_alt:.2f} Yaw={math.degrees(home_yaw):.1f}deg")

    # Hover after takeoff
    if hover_mode == "guided":
        log(f"GUIDED hover for {M2_GUIDED_HOVER_S}s...")
        rate = rospy.Rate(10); t0 = time.time()
        while time.time()-t0 < M2_GUIDED_HOVER_S:
            if abort_flag.is_set() or land_flag.is_set(): do_land(); return
            publish_ned(home_n, home_e, home_alt, home_yaw)
            rate.sleep()
    else:
        log("Switching to FLOWHOLD for hover...")
        set_mode("22"); wait_mode("CMODE(22)", timeout=5)
        if not sleep_check(M2_GUIDED_HOVER_S): do_land(); return
        # Switch back to GUIDED for waypoints
        log("Switching back to GUIDED for waypoints...")
        if not set_mode("GUIDED") or not wait_mode("GUIDED"):
            abort_land("GUIDED switch failed after hover"); return

    # Waypoint 1: North 2m
    wp1_n = home_n + M2_NORTH_1_M
    wp1_e = home_e
    if not goto_wp(wp1_n, wp1_e, home_alt, home_yaw, label="North1"):
        do_land(); return
    log(f"Holding at North1 for {M2_WP_WAIT_S}s...")
    t0 = time.time(); rate = rospy.Rate(10)
    while time.time()-t0 < M2_WP_WAIT_S:
        if abort_flag.is_set() or land_flag.is_set(): do_land(); return
        publish_ned(wp1_n, wp1_e, home_alt, home_yaw); rate.sleep()

    # Waypoint 2: East 1m (from wp1)
    wp2_n = wp1_n
    wp2_e = home_e + M2_EAST_M
    if not goto_wp(wp2_n, wp2_e, home_alt, home_yaw, label="East"):
        do_land(); return

    # Waypoint 3: North 2m more (from wp2)
    wp3_n = wp2_n + M2_NORTH_2_M
    wp3_e = wp2_e
    if not goto_wp(wp3_n, wp3_e, home_alt, home_yaw, label="North2"):
        do_land(); return
    log(f"Holding at North2 for {M2_WP_WAIT_S}s...")
    t0 = time.time()
    while time.time()-t0 < M2_WP_WAIT_S:
        if abort_flag.is_set() or land_flag.is_set(): do_land(); return
        publish_ned(wp3_n, wp3_e, home_alt, home_yaw); rate.sleep()

    land_and_disarm()
    log("="*40); log("MISSION 2 COMPLETE"); log("="*40)

# ──────────────────────────────────────────────────────────────
#  SIGINT HANDLER
# ──────────────────────────────────────────────────────────────
def sigint_handler(sig, frame):
    warn("Ctrl+C -> Emergency land!")
    abort_flag.set(); do_land(); time.sleep(3.0); arm(False); _restore_kb(); sys.exit(0)

# ──────────────────────────────────────────────────────────────
#  MAIN
# ──────────────────────────────────────────────────────────────
def main():
    global rc_pub, sp_pub, arm_srv, mode_srv
    rospy.init_node("vio_mission_m2", anonymous=False)
    signal.signal(signal.SIGINT, sigint_handler)

    rospy.Subscriber(f"{MAVROS_NS}/state",                   State,        state_cb)
    rospy.Subscriber(f"{MAVROS_NS}/local_position/pose",     PoseStamped,  local_pos_cb)
    rospy.Subscriber(f"{MAVROS_NS}/vision_pose/pose",        PoseStamped,  vision_cb)
    rospy.Subscriber(f"{MAVROS_NS}/rangefinder/rangefinder", Range,        rf_cb)

    rc_pub = rospy.Publisher(f"{MAVROS_NS}/rc/override",        OverrideRCIn,   queue_size=5)
    sp_pub = rospy.Publisher(f"{MAVROS_NS}/setpoint_raw/local", PositionTarget, queue_size=5)

    log("Waiting for MAVROS services...")
    rospy.wait_for_service(f"{MAVROS_NS}/cmd/arming")
    rospy.wait_for_service(f"{MAVROS_NS}/set_mode")
    arm_srv  = rospy.ServiceProxy(f"{MAVROS_NS}/cmd/arming", CommandBool)
    mode_srv = rospy.ServiceProxy(f"{MAVROS_NS}/set_mode",   SetMode)

    log("Waiting for MAVROS heartbeat...")
    r = rospy.Rate(2)
    while not rospy.is_shutdown() and not state.connected: r.sleep()
    log("MAVROS connected.")
    rospy.sleep(1.0)

    try:
        run_mission2()
    except Exception as e:
        err(f"Unhandled exception: {e}")
        abort_flag.set(); do_land(); time.sleep(3.0); arm(False)
    finally:
        _restore_kb()
        rospy.signal_shutdown("Mission complete.")

if __name__ == "__main__":
    main()
