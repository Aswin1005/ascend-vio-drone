#!/usr/bin/env python3
"""
arena_nav_guided.py  —  GUIDED-based arena sweep with yellow border detection
===============================================================================
Mission: RC takeoff → GUIDED mode → waypoint sweep → return to (0,0) → land

Loop (×N_LOOPS):
  GUIDED sideways SIDE_M  (yellow check: stop if border detected) → hover →
  GUIDED forward  FWD_M   (yellow check) → hover →
  GUIDED opposite SIDE_M  (yellow check) → hover →
  [between loops: GUIDED shift FWD_SHIFT_M]

After loops:
  GUIDED goto (0, 0, alt) → hover 5s → LAND (20s)

Requires: yellow_border_node.py running in a separate terminal.
Uses the same working GUIDED takeoff method as vio_mission_ros1.py M2.
"""

import rospy, threading, math, sys, time, signal, termios, tty, select
from mavros_msgs.msg import State, OverrideRCIn, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String

# ══════════════════════════════════════════════════════════════
#  PARAMETERS  (edit here)
# ══════════════════════════════════════════════════════════════

# Takeoff ramp (RC override phase)
TAKEOFF_PWM_START  = 1400
TAKEOFF_PWM_MAX    = 1850
TAKEOFF_PWM_STEP   = 10
TAKEOFF_TICK_S     = 0.3
GROUND_ALT_OFFSET  = 0.10
TAKEOFF_TARGET_OFF = 1.5

# GUIDED altitude
GUIDED_ALT = 2.0     # target hover altitude (m)

# Sweep dimensions — use +/- to control direction
#   East = positive, West = negative
#   North = positive, South = negative
SIDE_M      =  2.0    # sideways distance per leg (+ = East/right, - = West/left)
FWD_M       =  3.0    # forward distance per leg (+ = North, - = South)
FWD_SHIFT_M =  1.0    # shift between loops (+ = North)

# Number of loops
N_LOOPS = 1

# Waypoint navigation
WP_TOLERANCE  = 0.30    # reached radius (m)
WP_TIMEOUT_S  = 15.0    # warn if slow — continues anyway
WP_RATE_HZ    = 10

# Hovers
HOVER_S             = 3.0    # general hover between waypoints (s)
YELLOW_STOP_HOVER_S = 5.0    # hover after yellow detection (s)

# Return to origin
ORIGIN_HOVER_S  = 5.0    # hover at (0,0) before land
ORIGIN_TIMEOUT_S = 30.0

# Yaw 180° after loops (in LOITER mode, RC timer-based)
YAW_180_PWM    = 1560   # yaw channel PWM for turning
YAW_180_TIME_S = 8.0    # timer for 180° at this PWM

# Land
LAND_WAIT_S = 20.0

# VIO guard
VIO_XY_LIMIT_M   = 4.0
VIO_CHECK_INTV_S = 0.5

# Pre-GUIDED
PRE_GUIDED_STREAM_S  = 3.0    # seconds to pre-stream setpoints
POST_GUIDED_ANCHOR_S = 2.0    # anchor at home after GUIDED switch

MAVROS_NS = "/mavros"

# ══════════════════════════════════════════════════════════════
#  GLOBALS
# ══════════════════════════════════════════════════════════════
state       = State()
local_pose  = PoseStamped()
vision_pose = PoseStamped()
rf_range    = 0.0
abort_flag  = threading.Event()
land_flag   = threading.Event()

rc_hb_active = threading.Event()
rc_hb_lock   = threading.Lock()
rc_hb_vals   = [1500, 1500, 1500, 1500]

yellow_status      = "none"
yellow_status_lock = threading.Lock()
yellow_connected   = False

rc_pub = sp_pub = arm_srv = mode_srv = None

# ══════════════════════════════════════════════════════════════
#  CALLBACKS
# ══════════════════════════════════════════════════════════════
def state_cb(msg):      global state;       state = msg
def local_pos_cb(msg):  global local_pose;  local_pose = msg
def vision_cb(msg):     global vision_pose; vision_pose = msg
def rf_cb(msg):         global rf_range;    rf_range = msg.range

def yellow_cb(msg):
    global yellow_status, yellow_connected
    with yellow_status_lock:
        yellow_status = msg.data
    yellow_connected = True

# ══════════════════════════════════════════════════════════════
#  LOGGING
# ══════════════════════════════════════════════════════════════
def log(m):  rospy.loginfo(f"[ARENA-G] {m}")
def warn(m): rospy.logwarn(f"[WARN]    {m}")
def err(m):  rospy.logerr(f"[ERROR]   {m}")

# ══════════════════════════════════════════════════════════════
#  YELLOW HELPERS
# ══════════════════════════════════════════════════════════════
def yellow_active(region):
    with yellow_status_lock:
        return region in yellow_status

def get_yellow():
    with yellow_status_lock:
        return yellow_status

def yellow_any():
    """True if any border is detected."""
    with yellow_status_lock:
        return yellow_status != "none"

# ══════════════════════════════════════════════════════════════
#  MAVROS HELPERS
# ══════════════════════════════════════════════════════════════
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
        log("Arming ACCEPTED" if (do_arm and res.success) else
            "Disarming ACCEPTED" if res.success else "Arm REJECTED")
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
    pt.position.x = north
    pt.position.y = east
    pt.position.z = alt_up
    pt.yaw = yaw_rad
    sp_pub.publish(pt)

# ══════════════════════════════════════════════════════════════
#  RC OVERRIDE (for takeoff phase only)
# ══════════════════════════════════════════════════════════════
def send_rc(roll=1500, pitch=1500, throttle=1500, yaw=1500):
    msg = OverrideRCIn(); msg.channels = [0]*18
    msg.channels[0]=roll; msg.channels[1]=pitch
    msg.channels[2]=throttle; msg.channels[3]=yaw
    rc_pub.publish(msg)

def clear_rc():
    rc_hb_active.clear()
    msg = OverrideRCIn(); msg.channels = [0]*18
    rc_pub.publish(msg)

def set_rc_hb(roll=1500, pitch=1500, throttle=1500, yaw=1500):
    with rc_hb_lock: rc_hb_vals[:] = [roll, pitch, throttle, yaw]

def _rc_hb_loop():
    while rc_hb_active.is_set() and not rospy.is_shutdown():
        with rc_hb_lock: r,p,t,y = rc_hb_vals
        send_rc(r,p,t,y); time.sleep(0.2)

def start_rc_heartbeat():
    set_rc_hb(1500,1500,1500,1500)
    if not rc_hb_active.is_set():
        rc_hb_active.set()
        threading.Thread(target=_rc_hb_loop, daemon=True).start()
    log("RC heartbeat started.")

# ══════════════════════════════════════════════════════════════
#  LAND
# ══════════════════════════════════════════════════════════════
def do_land():
    log("LAND mode — clearing RC.")
    clear_rc(); time.sleep(0.1); set_mode("LAND")

def abort_land(reason=""):
    err(f"ABORT: {reason}"); abort_flag.set(); do_land()

def land_and_disarm():
    do_land()
    log(f"Waiting up to {LAND_WAIT_S}s for disarm...")
    t0 = time.time()
    while time.time()-t0 < LAND_WAIT_S:
        if not state.armed: log("Disarmed."); return
        time.sleep(0.5)
    log("Timeout — forcing disarm."); arm(False)

# ══════════════════════════════════════════════════════════════
#  KEYBOARD 'l'
# ══════════════════════════════════════════════════════════════
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
                if ch.lower()=='l': log("'l' pressed -> landing."); land_flag.set()
        except: break
    _restore_kb()

# ══════════════════════════════════════════════════════════════
#  VIO GUARD
# ══════════════════════════════════════════════════════════════
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
    log(f"VIO guard started (limit ±{VIO_XY_LIMIT_M}m).")

# ══════════════════════════════════════════════════════════════
#  RC TAKEOFF (same as vio_mission_ros1.py)
# ══════════════════════════════════════════════════════════════
def rc_takeoff(arm_mode, wait_str):
    log(f"Switch to {arm_mode}")
    if not set_mode(arm_mode): return False
    if not wait_mode(wait_str): return False
    log("Arming...")
    if not arm(True): return False
    if not wait_armed(True): return False

    ground=rf_range; detect_alt=ground+GROUND_ALT_OFFSET; target_alt=ground+TAKEOFF_TARGET_OFF
    log(f"Takeoff: ground={ground:.2f}m target={target_alt:.2f}m")

    has_liftoff=False; frozen_pwm=TAKEOFF_PWM_START
    for pwm in range(TAKEOFF_PWM_START, TAKEOFF_PWM_MAX+1, TAKEOFF_PWM_STEP):
        if abort_flag.is_set() or land_flag.is_set() or rospy.is_shutdown(): return False
        curr = rf_range
        if not has_liftoff and curr > detect_alt:
            has_liftoff=True; frozen_pwm=1500
            log(f"Liftoff at {curr:.2f}m")
        active = frozen_pwm if has_liftoff else pwm
        if curr >= target_alt:
            log("Target alt reached!"); send_rc(throttle=1500); return True
        send_rc(throttle=active); time.sleep(TAKEOFF_TICK_S)

    warn("Ramp exhausted."); send_rc(throttle=1500); return True

# ══════════════════════════════════════════════════════════════
#  HOVER HELPER (GUIDED or FLOWHOLD)
# ══════════════════════════════════════════════════════════════
def _ok():
    return not abort_flag.is_set() and not land_flag.is_set() and not rospy.is_shutdown()

def guided_hover(duration_s, n, e, alt, yaw):
    """Hold position in GUIDED for duration_s."""
    rate = rospy.Rate(WP_RATE_HZ); t0 = time.time()
    while _ok() and time.time()-t0 < duration_s:
        publish_ned(n, e, alt, yaw); rate.sleep()
    return _ok()

def do_hover_guided_or_fh(duration_s, hover_mode, n, e, alt, yaw):
    """Hover using either GUIDED hold or FLOWHOLD."""
    if hover_mode == "flowhold":
        set_mode("22"); wait_mode("CMODE(22)", timeout=5)
        set_rc_hb(1500,1500,1500,1500)
        ok = sleep_check(duration_s)
        # Switch back to GUIDED
        set_mode("GUIDED"); wait_mode("GUIDED", timeout=5)
        return ok
    else:
        # GUIDED hold
        return guided_hover(duration_s, n, e, alt, yaw)

# ══════════════════════════════════════════════════════════════
#  GUIDED NAVIGATION WITH YELLOW CHECK
# ══════════════════════════════════════════════════════════════
def goto_wp(target_n, target_e, alt, yaw, label, hover_mode):
    """Navigate to (target_n, target_e) in GUIDED. Stops if any yellow detected."""
    log(f"GOTO {label}: N={target_n:.2f} E={target_e:.2f}")
    rate = rospy.Rate(WP_RATE_HZ)
    t0 = time.time(); warned = False

    while _ok():
        publish_ned(target_n, target_e, alt, yaw)
        cn, ce, _ = get_pos()
        dist = math.sqrt((cn-target_n)**2 + (ce-target_e)**2)

        if dist < WP_TOLERANCE:
            log(f"Reached {label} (dist={dist:.2f}m)")
            return do_hover_guided_or_fh(HOVER_S, hover_mode, target_n, target_e, alt, yaw)

        # Yellow border check
        if yellow_any():
            ys = get_yellow()
            log(f"Yellow '{ys}' detected during {label}! Stopping at current position.")
            cn, ce, _ = get_pos()
            return do_hover_guided_or_fh(YELLOW_STOP_HOVER_S, hover_mode, cn, ce, alt, yaw)

        elapsed = time.time()-t0
        if elapsed > WP_TIMEOUT_S:
            warn(f"Timeout on {label} after {WP_TIMEOUT_S}s. Proceeding to next waypoint.")
            cn, ce, _ = get_pos()
            return do_hover_guided_or_fh(HOVER_S, hover_mode, cn, ce, alt, yaw)

        rate.sleep()

    return False

# ══════════════════════════════════════════════════════════════
#  MISSION
# ══════════════════════════════════════════════════════════════
def run_mission():
    log("="*50); log("ARENA NAV — GUIDED SWEEP"); log("="*50)
    log(f"Loops: {N_LOOPS}  Alt: {GUIDED_ALT}m")
    log(f"Side: {SIDE_M}m  Fwd: {FWD_M}m  Shift: {FWD_SHIFT_M}m")
    log(f"WP tolerance: {WP_TOLERANCE}m  timeout: {WP_TIMEOUT_S}s")

    if not yellow_connected:
        warn("yellow_border_node NOT connected!")
        c = input("Continue without yellow? (y/n): ").strip().lower()
        if c != 'y': log("Aborted."); return

    # Hover mode (between waypoints)
    while True:
        c = input("\nHover mode between waypoints:\n  1 - GUIDED hold\n  2 - FLOWHOLD\nEnter: ").strip()
        if c=="1": hover_mode="guided"; break
        elif c=="2": hover_mode="flowhold"; break
        print("Invalid.")

    threading.Thread(target=_kb_thread, daemon=True).start()
    start_vio_guard()

    # ── RC TAKEOFF (Hardcoded to LOITER like nav_loiter) ──
    if not rc_takeoff("LOITER", "LOITER"):
        abort_land("Takeoff failed"); return

    start_rc_heartbeat()
    set_rc_hb(1500,1500,1500,1500)

    # Ensure LOITER mode
    if state.mode != "LOITER":
        log("Switching to LOITER...")
        if not set_mode("LOITER") or not wait_mode("LOITER"):
            abort_land("LOITER switch failed"); return

    # Pre-sequence hover in FLOWHOLD (like nav_loiter)
    log("Switching to FLOWHOLD (mode 22) for pre-sequence hover...")
    if not set_mode("22") or not wait_mode("CMODE(22)", timeout=5):
        abort_land("FLOWHOLD switch failed"); return
    
    log("Hovering in FLOWHOLD for 10s...")
    if not sleep_check(10.0): land_and_disarm(); return

    log("Switching back to LOITER before GUIDED transition...")
    if not set_mode("LOITER") or not wait_mode("LOITER", timeout=5):
        abort_land("LOITER switch back failed"); return

    # Capture home
    rospy.sleep(1.0)
    home_n, home_e, _ = get_pos()
    home_alt = GUIDED_ALT
    home_yaw = get_yaw()
    log(f"Home: N={home_n:.3f} E={home_e:.3f} Alt={home_alt:.2f} Yaw={math.degrees(home_yaw):.1f}°")

    # Pre-stream setpoints before GUIDED switch
    log(f"Pre-streaming setpoints {PRE_GUIDED_STREAM_S}s...")
    rate = rospy.Rate(WP_RATE_HZ); t0 = time.time()
    while time.time()-t0 < PRE_GUIDED_STREAM_S and _ok():
        publish_ned(home_n, home_e, home_alt, home_yaw); rate.sleep()

    # Switch to GUIDED
    log("Switching to GUIDED...")
    if not set_mode("GUIDED") or not wait_mode("GUIDED"):
        abort_land("GUIDED switch failed"); return

    # Anchor at home
    log(f"Anchoring home {POST_GUIDED_ANCHOR_S}s...")
    t0 = time.time()
    while time.time()-t0 < POST_GUIDED_ANCHOR_S and _ok():
        publish_ned(home_n, home_e, home_alt, home_yaw); rate.sleep()

    if not _ok(): land_and_disarm(); return

    # Stop RC heartbeat — GUIDED uses setpoints now
    clear_rc()

    # ── CLIMB TO GUIDED_ALT ──
    log(f"Climbing to {GUIDED_ALT}m...")
    if not goto_wp(home_n, home_e, home_alt, home_yaw, "Climb", hover_mode):
        land_and_disarm(); return

    # ── MAIN LOOP ──
    # Track cumulative position for the sweep pattern
    cur_n = home_n
    cur_e = home_e

    for loop in range(1, N_LOOPS+1):
        log(f"═══ Loop {loop}/{N_LOOPS} ═══")

        if not _ok(): land_and_disarm(); return

        # 1. Sideways (SIDE_M)
        target_e = cur_e + SIDE_M
        if not goto_wp(cur_n, target_e, home_alt, home_yaw, f"Side-1 (E={target_e:.2f})", hover_mode):
            land_and_disarm(); return
        cur_e = target_e

        if not _ok(): land_and_disarm(); return

        # 2. Forward (FWD_M)
        target_n = cur_n + FWD_M
        if not goto_wp(target_n, cur_e, home_alt, home_yaw, f"Fwd (N={target_n:.2f})", hover_mode):
            land_and_disarm(); return
        cur_n = target_n

        if not _ok(): land_and_disarm(); return

        # 3. Opposite side (-SIDE_M)
        target_e = cur_e - SIDE_M
        if not goto_wp(cur_n, target_e, home_alt, home_yaw, f"Side-2 (E={target_e:.2f})", hover_mode):
            land_and_disarm(); return
        cur_e = target_e

        if not _ok(): land_and_disarm(); return

        # Shift between loops
        if loop < N_LOOPS:
            target_n = cur_n + FWD_SHIFT_M
            if not goto_wp(target_n, cur_e, home_alt, home_yaw, f"Shift (N={target_n:.2f})", hover_mode):
                land_and_disarm(); return
            cur_n = target_n

    log("All loops complete.")

    if not _ok(): land_and_disarm(); return

    # ── YAW 180° (in LOITER, timer-based) ──
    log(f"Yaw 180° — switching to LOITER...")
    start_rc_heartbeat()
    if not set_mode("LOITER") or not wait_mode("LOITER"):
        abort_land("LOITER switch for yaw failed"); return

    log(f"Yawing 180° ({YAW_180_PWM}) for {YAW_180_TIME_S}s...")
    set_rc_hb(roll=1500, pitch=1500, throttle=1500, yaw=YAW_180_PWM)
    if not sleep_check(YAW_180_TIME_S):
        land_and_disarm(); return
    set_rc_hb(1500, 1500, 1500, 1500)
    log(f"Yaw complete. Hovering {HOVER_S}s...")
    if not sleep_check(HOVER_S):
        land_and_disarm(); return

    if not _ok(): land_and_disarm(); return

    # Pre-stream setpoints at current pos before switching back to GUIDED
    cn, ce, _ = get_pos()
    log(f"Pre-streaming at current pos N={cn:.2f} E={ce:.2f} before GUIDED switch...")
    t_pre = time.time()
    while time.time()-t_pre < PRE_GUIDED_STREAM_S and _ok():
        publish_ned(cn, ce, home_alt, home_yaw); rate.sleep()

    clear_rc()
    log("Switching back to GUIDED for return...")
    if not set_mode("GUIDED") or not wait_mode("GUIDED"):
        abort_land("GUIDED switch failed after yaw"); return

    if not _ok(): land_and_disarm(); return

    # ── RETURN TO ORIGIN ──
    log("Returning to origin (0, 0)...")
    t0 = time.time(); warned = False
    while _ok():
        publish_ned(home_n, home_e, home_alt, home_yaw)
        cn, ce, _ = get_pos()
        dist = math.sqrt((cn-home_n)**2 + (ce-home_e)**2)
        if dist < WP_TOLERANCE:
            log(f"Reached origin (dist={dist:.2f}m)"); break
        if not warned and time.time()-t0 > ORIGIN_TIMEOUT_S:
            warn(f"Still {dist:.2f}m from origin. Continuing..."); warned=True
        rate.sleep()

    if not _ok(): land_and_disarm(); return

    # Hover at origin
    log(f"Hovering at origin for {ORIGIN_HOVER_S}s...")
    guided_hover(ORIGIN_HOVER_S, home_n, home_e, home_alt, home_yaw)

    # ── LAND ──
    land_and_disarm()
    log("="*50); log("ARENA NAV GUIDED — COMPLETE"); log("="*50)

# ══════════════════════════════════════════════════════════════
#  SIGINT
# ══════════════════════════════════════════════════════════════
def sigint_handler(sig, frame):
    warn("Ctrl+C -> Emergency land!")
    abort_flag.set(); do_land(); time.sleep(3.0); arm(False); _restore_kb(); sys.exit(0)

# ══════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════
def main():
    global rc_pub, sp_pub, arm_srv, mode_srv
    rospy.init_node("arena_nav_guided", anonymous=False)
    signal.signal(signal.SIGINT, sigint_handler)

    rospy.Subscriber(f"{MAVROS_NS}/state",                   State,       state_cb)
    rospy.Subscriber(f"{MAVROS_NS}/local_position/pose",     PoseStamped, local_pos_cb)
    rospy.Subscriber(f"{MAVROS_NS}/vision_pose/pose",        PoseStamped, vision_cb)
    rospy.Subscriber(f"{MAVROS_NS}/rangefinder/rangefinder", Range,       rf_cb)
    rospy.Subscriber("/yellow_border/status",                 String,      yellow_cb)

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

    rospy.sleep(2.0)
    if yellow_connected:
        log(f"Yellow border node connected. Status: {get_yellow()}")
    else:
        warn("Yellow border node not detected.")

    try:
        run_mission()
    except Exception as e:
        err(f"Unhandled exception: {e}")
        abort_flag.set(); do_land(); time.sleep(3.0); arm(False)
    finally:
        _restore_kb()
        rospy.signal_shutdown("Done.")

if __name__ == "__main__":
    main()
