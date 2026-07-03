#!/usr/bin/env python3
"""
loiter_sweep_m2.py  —  ROS1 Melodic / MAVROS
=============================================
Mission 2: Identical sweep to M1, but after loops → GUIDED → go to 0,0 → hover 5s → LAND

Loop sequence (repeated N_LOOPS times):
  Roll right T1s → Pitch forward T2s → Roll left T3s
  [between loops only]: Shift pitch T4s

After all loops:
  Switch to GUIDED → wait 5s → navigate to (0,0) → hover 5s → LAND (20s)

Safety: 'l' key, Ctrl+C, VIO divergence guard — all same as M1
"""

import rospy, threading, math, sys, time, signal, termios, tty, select
from mavros_msgs.msg import State, OverrideRCIn, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range

# ──────────────────────────────────────────────────────────────
#  PARAMETERS  (edit here)
# ──────────────────────────────────────────────────────────────

# Takeoff ramp
TAKEOFF_PWM_START  = 1400
TAKEOFF_PWM_MAX    = 1750
TAKEOFF_PWM_STEP   = 10
TAKEOFF_TICK_S     = 0.3
GROUND_ALT_OFFSET  = 0.10
TAKEOFF_TARGET_OFF = 0.35   # target alt above ground (m)

# Number of sweep loops
N_LOOPS = 1

# Per-loop motion
ROLL_R_PWM   = 1530
T1           = 5.0    # roll right duration (s)

PITCH_F_PWM  = 1470
T2           = 5.0    # pitch forward duration (s)

ROLL_L_PWM   = 1470
T3           = 5.0    # roll left duration (s)

# Shift pitch (between loops only, not after last)
SHIFT_PITCH_PWM = 1470
T4              = 3.0

# Hover between commands
HOVER_S = 3.0

# M2-specific: GUIDED return-to-origin
GUIDED_SETTLE_S  = 5.0    # wait after switching to GUIDED before moving
ORIGIN_HOVER_S   = 5.0    # hover at (0,0) before landing
ORIGIN_TOLERANCE = 0.30   # metres — close enough to origin
ORIGIN_TIMEOUT_S = 30.0   # warn if not reached in time (continues anyway)

# Land
LAND_WAIT_S = 20.0

# VIO guard
VIO_XY_LIMIT_M   = 4.0
VIO_CHECK_INTV_S = 0.5

MAVROS_NS = "/mavros"

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
def log(m):  rospy.loginfo(f"[SWEEP2] {m}")
def warn(m): rospy.logwarn(f"[WARN]   {m}")
def err(m):  rospy.logerr(f"[ERROR]  {m}")

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
        log("Arming ACCEPTED" if (do_arm and res.success) else
            "Disarming ACCEPTED" if res.success else f"Arm REJECTED (code:{res.result})")
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

# ──────────────────────────────────────────────────────────────
#  RC OVERRIDE
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
    log(f"Waiting up to {LAND_WAIT_S}s for auto-disarm...")
    t0 = time.time()
    while time.time()-t0 < LAND_WAIT_S:
        if not state.armed: log("Auto-disarmed."); return
        time.sleep(0.5)
    log("Timeout — forcing disarm."); arm(False)

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
                if ch.lower()=='l': log("'l' pressed -> landing."); land_flag.set()
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
#  RC TAKEOFF
# ──────────────────────────────────────────────────────────────
def rc_takeoff(arm_mode, wait_str):
    log(f"Switch to {arm_mode}")
    if not set_mode(arm_mode): return False
    if not wait_mode(wait_str): return False
    log("Arming...")
    if not arm(True): return False
    if not wait_armed(True): return False

    ground=rf_range; detect_alt=ground+GROUND_ALT_OFFSET; target_alt=ground+TAKEOFF_TARGET_OFF
    log(f"Takeoff ramp: ground={ground:.2f}m target={target_alt:.2f}m")

    has_liftoff=False; frozen_pwm=TAKEOFF_PWM_START
    for pwm in range(TAKEOFF_PWM_START, TAKEOFF_PWM_MAX+1, TAKEOFF_PWM_STEP):
        if abort_flag.is_set() or land_flag.is_set() or rospy.is_shutdown(): return False
        curr = rf_range
        if not has_liftoff and curr > detect_alt:
            has_liftoff=True; frozen_pwm=1500
            log(f"Liftoff detected at {curr:.2f}m")
        active = frozen_pwm if has_liftoff else pwm
        if curr >= target_alt:
            log("Target altitude reached!"); send_rc(throttle=1500); return True
        send_rc(throttle=active); time.sleep(TAKEOFF_TICK_S)

    warn("Ramp exhausted. Continuing."); send_rc(throttle=1500); return True

# ──────────────────────────────────────────────────────────────
#  HOVER HELPER
# ──────────────────────────────────────────────────────────────
def do_hover(duration_s, hover_mode):
    if hover_mode == "flowhold":
        set_mode("22"); wait_mode("CMODE(22)", timeout=5)
        set_rc_hb(1500,1500,1500,1500)
        ok = sleep_check(duration_s)
        set_mode("LOITER"); wait_mode("LOITER", timeout=5)
        return ok
    else:
        set_rc_hb(1500,1500,1500,1500)
        return sleep_check(duration_s)

# ──────────────────────────────────────────────────────────────
#  SWEEP LOOP  (shared logic)
# ──────────────────────────────────────────────────────────────
def run_sweep_loop(hover_mode):
    """Returns False if aborted/landed during sweep."""
    for loop in range(1, N_LOOPS+1):
        log(f"--- Loop {loop}/{N_LOOPS} ---")

        # Roll right
        if abort_flag.is_set() or land_flag.is_set(): return False
        log(f"Roll RIGHT ({ROLL_R_PWM}) for {T1}s")
        set_rc_hb(roll=ROLL_R_PWM, pitch=1500, throttle=1500, yaw=1500)
        if not sleep_check(T1): return False
        if not do_hover(HOVER_S, hover_mode): return False

        # Pitch forward
        if abort_flag.is_set() or land_flag.is_set(): return False
        log(f"Pitch FORWARD ({PITCH_F_PWM}) for {T2}s")
        set_rc_hb(roll=1500, pitch=PITCH_F_PWM, throttle=1500, yaw=1500)
        if not sleep_check(T2): return False
        if not do_hover(HOVER_S, hover_mode): return False

        # Roll left
        if abort_flag.is_set() or land_flag.is_set(): return False
        log(f"Roll LEFT ({ROLL_L_PWM}) for {T3}s")
        set_rc_hb(roll=ROLL_L_PWM, pitch=1500, throttle=1500, yaw=1500)
        if not sleep_check(T3): return False
        if not do_hover(HOVER_S, hover_mode): return False

        # Shift between loops
        if loop < N_LOOPS:
            log(f"Shift pitch ({SHIFT_PITCH_PWM}) for {T4}s")
            set_rc_hb(roll=1500, pitch=SHIFT_PITCH_PWM, throttle=1500, yaw=1500)
            if not sleep_check(T4): return False
            if not do_hover(HOVER_S, hover_mode): return False

    return True

# ──────────────────────────────────────────────────────────────
#  MISSION 2
# ──────────────────────────────────────────────────────────────
def run_mission2():
    log("="*40); log("LOITER SWEEP MISSION 2 (GUIDED RETURN)"); log("="*40)
    log(f"Loops: {N_LOOPS} | Roll-R: {ROLL_R_PWM}/{T1}s | Pitch-F: {PITCH_F_PWM}/{T2}s | Roll-L: {ROLL_L_PWM}/{T3}s")
    log(f"Shift: {SHIFT_PITCH_PWM}/{T4}s | Hover: {HOVER_S}s")
    log(f"After loops: GUIDED settle {GUIDED_SETTLE_S}s -> goto (0,0) -> hover {ORIGIN_HOVER_S}s -> LAND")

    # Takeoff mode
    while True:
        c = input("\nTakeoff mode:\n  1 - FLOWHOLD\n  2 - LOITER\nEnter (1 or 2): ").strip()
        if c=="1": arm_mode,wait_str="22","CMODE(22)"; break
        elif c=="2": arm_mode,wait_str="LOITER","LOITER"; break
        print("Invalid.")

    # Hover mode
    while True:
        c = input("\nHover mode between commands:\n  1 - NEUTRAL (1500 in LOITER)\n  2 - FLOWHOLD\nEnter (1 or 2): ").strip()
        if c=="1": hover_mode="neutral"; break
        elif c=="2": hover_mode="flowhold"; break
        print("Invalid.")

    threading.Thread(target=_kb_thread, daemon=True).start()
    start_vio_guard()

    if not rc_takeoff(arm_mode, wait_str):
        abort_land("Takeoff failed"); return

    start_rc_heartbeat()
    set_rc_hb(1500,1500,1500,1500)

    # Ensure LOITER
    if state.mode != "LOITER":
        log("Switching to LOITER...")
        if not set_mode("LOITER") or not wait_mode("LOITER"):
            abort_land("LOITER switch failed"); return

    # Pre-sequence hover
    log(f"Pre-sequence hover ({hover_mode}) for {HOVER_S}s...")
    if not do_hover(HOVER_S, hover_mode): do_land(); return

    # ── SWEEP LOOP ──
    if not run_sweep_loop(hover_mode):
        do_land(); return

    log("All loops complete. Starting GUIDED return.")

    # ── GUIDED RETURN TO ORIGIN ──
    if abort_flag.is_set() or land_flag.is_set(): do_land(); return

    # Capture current altitude and yaw to maintain during return
    _, _, curr_alt = get_pos()
    home_yaw = get_yaw()
    log(f"Captured return alt={curr_alt:.2f}m yaw={math.degrees(home_yaw):.1f}deg")

    # Pre-stream setpoints 3s before switching to GUIDED
    log("Pre-streaming setpoints before GUIDED switch...")
    rate = rospy.Rate(10); t0 = time.time()
    while time.time()-t0 < 3.0 and not abort_flag.is_set() and not land_flag.is_set():
        publish_ned(0.0, 0.0, curr_alt, home_yaw); rate.sleep()

    # Switch to GUIDED
    log("Switching to GUIDED...")
    if not set_mode("GUIDED") or not wait_mode("GUIDED"):
        abort_land("GUIDED switch failed"); return

    # Settle in GUIDED for GUIDED_SETTLE_S while streaming origin setpoint
    log(f"Settling in GUIDED for {GUIDED_SETTLE_S}s...")
    t0 = time.time()
    while time.time()-t0 < GUIDED_SETTLE_S:
        if abort_flag.is_set() or land_flag.is_set(): do_land(); return
        publish_ned(0.0, 0.0, curr_alt, home_yaw); rate.sleep()

    # Navigate to (0,0)
    log("Navigating to origin (0, 0)...")
    warned = False; t0 = time.time()
    while not abort_flag.is_set() and not land_flag.is_set() and not rospy.is_shutdown():
        publish_ned(0.0, 0.0, curr_alt, home_yaw)
        n, e, _ = get_pos()
        dist = math.sqrt(n**2 + e**2)
        elapsed = time.time()-t0

        if not warned and elapsed > ORIGIN_TIMEOUT_S:
            warn(f"Still {dist:.2f}m from origin after {ORIGIN_TIMEOUT_S}s. Continuing...")
            warned = True

        if dist < ORIGIN_TOLERANCE:
            log(f"Reached origin (dist={dist:.2f}m, t={elapsed:.1f}s)")
            break

        rate.sleep()

    if abort_flag.is_set() or land_flag.is_set(): do_land(); return

    # Hover at origin
    log(f"Hovering at origin for {ORIGIN_HOVER_S}s...")
    t0 = time.time()
    while time.time()-t0 < ORIGIN_HOVER_S:
        if abort_flag.is_set() or land_flag.is_set(): do_land(); return
        publish_ned(0.0, 0.0, curr_alt, home_yaw); rate.sleep()

    # ── LAND ──
    land_and_disarm()
    log("="*40); log("MISSION 2 COMPLETE"); log("="*40)

# ──────────────────────────────────────────────────────────────
#  SIGINT
# ──────────────────────────────────────────────────────────────
def sigint_handler(sig, frame):
    warn("Ctrl+C -> Emergency land!")
    abort_flag.set(); do_land(); time.sleep(3.0); arm(False); _restore_kb(); sys.exit(0)

# ──────────────────────────────────────────────────────────────
#  MAIN
# ──────────────────────────────────────────────────────────────
def main():
    global rc_pub, sp_pub, arm_srv, mode_srv
    rospy.init_node("loiter_sweep_m2", anonymous=False)
    signal.signal(signal.SIGINT, sigint_handler)

    rospy.Subscriber(f"{MAVROS_NS}/state",                   State,       state_cb)
    rospy.Subscriber(f"{MAVROS_NS}/local_position/pose",     PoseStamped, local_pos_cb)
    rospy.Subscriber(f"{MAVROS_NS}/vision_pose/pose",        PoseStamped, vision_cb)
    rospy.Subscriber(f"{MAVROS_NS}/rangefinder/rangefinder", Range,       rf_cb)

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
