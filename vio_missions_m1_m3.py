#!/usr/bin/env python3
"""
vio_missions_m1_m3.py  —  ROS1 Melodic / MAVROS  (ArduCopter)
==============================================================
Mission 1: RC-override takeoff (FLOWHOLD or LOITER) + roll/pitch sweep in LOITER
Mission 3: Same as Mission 1 + yaw hold throughout

Safety:
  • Press 'l'  → immediate LAND
  • Ctrl+C     → LAND + disarm
  • Exception  → LAND + disarm
  • VIO divergence guard (background thread)
  • RC heartbeat (throttle=1500) until LAND
"""

import rospy, threading, math, sys, time, signal, termios, tty, select
from mavros_msgs.msg import State, OverrideRCIn, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range

# ──────────────────────────────────────────────────────────────
#  PARAMETERS  (edit here)
# ──────────────────────────────────────────────────────────────
MAVROS_NS           = "/mavros"

# Takeoff ramp
TAKEOFF_PWM_START   = 1400
TAKEOFF_PWM_MAX     = 1750
TAKEOFF_PWM_STEP    = 10
TAKEOFF_TICK_S      = 0.3
GROUND_ALT_OFFSET   = 0.10
TAKEOFF_TARGET_OFF  = 0.45

# Sequence timings & PWMs  (Mission 1 & 3)
M1_LOOPS            = 1       # number of sweep loops
M1_PRE_HOVER_S      = 10.0    # hover after takeoff before starting sequence
M1_ROLL_R_PWM       = 1530   # roll right
M1_ROLL_L_PWM       = 1470   # roll left
M1_PITCH_F_PWM      = 1470   # pitch forward
M1_PITCH_B_PWM      = 1470   # pitch backward
M1_CMD_S            = 5.0    # each roll/pitch command duration
M1_HOVER_S          = 5.0    # hover between commands
M1_POST_HOVER_S     = 5.0    # hover after last command before land
M1_LAND_WAIT_S      = 20.0   # wait after LAND before disarm

# VIO divergence
VIO_XY_LIMIT_M      = 4.0
VIO_CHECK_INTV_S    = 0.5

# Yaw hold  (Mission 3 only)
M3_YAW_DEADBAND_DEG = 5.0
M3_YAW_GAIN         = 8      # PWM per degree of error
M3_YAW_MAX_CORR     = 50     # max PWM deviation from 1500

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
rc_hb_vals   = [1500, 1500, 1500, 1500]  # roll, pitch, throttle, yaw

rc_pub = sp_pub = arm_srv = mode_srv = None

_yaw_hold_on     = False
_yaw_hold_target = 0.0

# ──────────────────────────────────────────────────────────────
#  CALLBACKS
# ──────────────────────────────────────────────────────────────
def state_cb(msg):      global state;      state = msg
def local_pos_cb(msg):  global local_pose; local_pose = msg
def vision_cb(msg):     global vision_pose; vision_pose = msg
def rf_cb(msg):         global rf_range;   rf_range = msg.range

# ──────────────────────────────────────────────────────────────
#  LOGGING
# ──────────────────────────────────────────────────────────────
def log(m):  rospy.loginfo(f"[MISSION] {m}")
def warn(m): rospy.logwarn(f"[WARN]    {m}")
def err(m):  rospy.logerr(f"[ERROR]   {m}")

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
        send_rc(r, p, t, y)
        time.sleep(0.2)

def start_rc_heartbeat(throttle=1500):
    set_rc_hb(throttle=throttle)
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
    log(f"Waiting up to {M1_LAND_WAIT_S}s for auto-disarm...")
    t0 = time.time()
    while time.time()-t0 < M1_LAND_WAIT_S:
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
#  YAW HOLD THREAD  (Mission 3)
# ──────────────────────────────────────────────────────────────
def _yaw_hold_loop():
    r = rospy.Rate(5)
    while _yaw_hold_on and not abort_flag.is_set() and not land_flag.is_set() and not rospy.is_shutdown():
        err_deg = math.degrees(_yaw_hold_target - get_yaw())
        err_deg = (err_deg+180)%360-180  # wrap to -180..180
        if abs(err_deg) > M3_YAW_DEADBAND_DEG:
            corr = int(min(M3_YAW_MAX_CORR, abs(err_deg)*M3_YAW_GAIN))
            yaw_pwm = 1500 + (corr if err_deg>0 else -corr)
        else:
            yaw_pwm = 1500
        with rc_hb_lock: rc_hb_vals[3] = yaw_pwm
        r.sleep()

def start_yaw_hold(target_rad):
    global _yaw_hold_on, _yaw_hold_target
    _yaw_hold_target = target_rad
    _yaw_hold_on = True
    threading.Thread(target=_yaw_hold_loop, daemon=True).start()
    log(f"Yaw hold started at {math.degrees(target_rad):.1f} deg.")

def stop_yaw_hold():
    global _yaw_hold_on
    _yaw_hold_on = False

# ──────────────────────────────────────────────────────────────
#  RC TAKEOFF
# ──────────────────────────────────────────────────────────────
def rc_takeoff(arm_mode_str, wait_mode_str):
    log(f"Step: Switch to {arm_mode_str}")
    if not set_mode(arm_mode_str): return False
    if not wait_mode(wait_mode_str): return False
    log("Step: Arming...")
    if not arm(True): return False
    if not wait_armed(True): return False

    log("Step: RC Override Takeoff ramp...")
    ground=rf_range; detect_alt=ground+GROUND_ALT_OFFSET; target_alt=ground+TAKEOFF_TARGET_OFF
    rospy.loginfo(f"[TAKEOFF] Ground={ground:.2f}m detect={detect_alt:.2f}m target={target_alt:.2f}m")

    has_liftoff=False; frozen_pwm=TAKEOFF_PWM_START
    for pwm in range(TAKEOFF_PWM_START, TAKEOFF_PWM_MAX+1, TAKEOFF_PWM_STEP):
        if abort_flag.is_set() or land_flag.is_set() or rospy.is_shutdown(): return False
        curr=rf_range
        if not has_liftoff and curr>detect_alt:
            has_liftoff=True; frozen_pwm=1500
            log(f"[TAKEOFF] Liftoff! Freezing PWM at {frozen_pwm}")
        active=frozen_pwm if has_liftoff else pwm
        rospy.loginfo(f"[TAKEOFF] alt={curr:.2f}m pwm={pwm} active={active}")
        if curr>=target_alt:
            log("[TAKEOFF] Target altitude reached!"); send_rc(throttle=1500); return True
        send_rc(throttle=active); time.sleep(TAKEOFF_TICK_S)

    warn("Ramp exhausted without reaching target. Continuing anyway.")
    send_rc(throttle=1500); return True

# ──────────────────────────────────────────────────────────────
#  HOVER HELPER
# ──────────────────────────────────────────────────────────────
def do_hover(duration_s, hover_mode, yaw_pwm=1500):
    """Hover for duration_s. hover_mode: 'neutral' or 'flowhold'."""
    if hover_mode == "flowhold":
        set_mode("22"); wait_mode("CMODE(22)", timeout=5)
        set_rc_hb(1500, 1500, 1500, yaw_pwm)
        ok = sleep_check(duration_s)
        set_mode("LOITER"); wait_mode("LOITER", timeout=5)
        return ok
    else:
        set_rc_hb(1500, 1500, 1500, yaw_pwm)
        return sleep_check(duration_s)

# ──────────────────────────────────────────────────────────────
#  SWEEP SEQUENCE  (shared by M1 & M3)
# ──────────────────────────────────────────────────────────────
def run_sweep_sequence(hover_mode, use_yaw_hold=False):
    """
    Runs M1_LOOPS of: roll-R -> hover -> pitch-F -> hover -> roll-L -> hover -> pitch-B -> hover
    Returns False if interrupted.
    """
    for loop in range(1, M1_LOOPS+1):
        log(f"--- Loop {loop}/{M1_LOOPS} ---")

        yaw_pwm = 1500  # updated live by yaw hold thread if use_yaw_hold

        # Roll right
        if land_flag.is_set() or abort_flag.is_set(): return False
        log(f"Roll RIGHT ({M1_ROLL_R_PWM}) for {M1_CMD_S}s...")
        set_rc_hb(roll=M1_ROLL_R_PWM, pitch=1500, throttle=1500, yaw=yaw_pwm)
        if not sleep_check(M1_CMD_S): return False

        # Hover
        log(f"Hover ({hover_mode}) for {M1_HOVER_S}s...")
        if not do_hover(M1_HOVER_S, hover_mode, yaw_pwm): return False

        # Pitch forward
        if land_flag.is_set() or abort_flag.is_set(): return False
        log(f"Pitch FORWARD ({M1_PITCH_F_PWM}) for {M1_CMD_S}s...")
        set_rc_hb(roll=1500, pitch=M1_PITCH_F_PWM, throttle=1500, yaw=yaw_pwm)
        if not sleep_check(M1_CMD_S): return False

        # Hover
        log(f"Hover ({hover_mode}) for {M1_HOVER_S}s...")
        if not do_hover(M1_HOVER_S, hover_mode, yaw_pwm): return False

        # Roll left
        if land_flag.is_set() or abort_flag.is_set(): return False
        log(f"Roll LEFT ({M1_ROLL_L_PWM}) for {M1_CMD_S}s...")
        set_rc_hb(roll=M1_ROLL_L_PWM, pitch=1500, throttle=1500, yaw=yaw_pwm)
        if not sleep_check(M1_CMD_S): return False

        # Hover
        log(f"Hover ({hover_mode}) for {M1_HOVER_S}s...")
        if not do_hover(M1_HOVER_S, hover_mode, yaw_pwm): return False

        # Pitch backward
        if land_flag.is_set() or abort_flag.is_set(): return False
        log(f"Pitch BACKWARD ({M1_PITCH_B_PWM}) for {M1_CMD_S}s...")
        set_rc_hb(roll=1500, pitch=M1_PITCH_B_PWM, throttle=1500, yaw=yaw_pwm)
        if not sleep_check(M1_CMD_S): return False

        # Hover after last command in loop
        log(f"Hover ({hover_mode}) for {M1_HOVER_S}s...")
        if not do_hover(M1_HOVER_S, hover_mode, yaw_pwm): return False

    return True

# ──────────────────────────────────────────────────────────────
#  MISSION 1
# ──────────────────────────────────────────────────────────────
def run_mission1():
    log("="*40); log("MISSION 1 START"); log("="*40)

    # Takeoff mode
    while True:
        c = input("\nTakeoff mode:\n  1 - FLOWHOLD\n  2 - LOITER\nEnter (1 or 2): ").strip()
        if c=="1": arm_mode,wait_str="22","CMODE(22)"; break
        elif c=="2": arm_mode,wait_str="LOITER","LOITER"; break
        print("Invalid.")

    # Hover mode between commands
    while True:
        c = input("\nHover mode between commands:\n  1 - NEUTRAL (1500 in LOITER)\n  2 - FLOWHOLD\nEnter (1 or 2): ").strip()
        if c=="1": hover_mode="neutral"; break
        elif c=="2": hover_mode="flowhold"; break
        print("Invalid.")

    threading.Thread(target=_kb_thread, daemon=True).start()
    start_vio_guard()

    if not rc_takeoff(arm_mode, wait_str):
        abort_land("Takeoff failed"); return

    start_rc_heartbeat(1500)
    set_rc_hb(1500, 1500, 1500, 1500)  # clear any residual from ramp

    # Switch to LOITER
    if state.mode != "LOITER":
        log("Switching to LOITER...")
        if not set_mode("LOITER") or not wait_mode("LOITER"):
            abort_land("LOITER switch failed"); return

    # Pre-sequence hover
    log(f"Pre-sequence hover ({hover_mode}) for {M1_PRE_HOVER_S}s...")
    if not do_hover(M1_PRE_HOVER_S, hover_mode): do_land(); return

    # Run sweep
    if not run_sweep_sequence(hover_mode, use_yaw_hold=False): do_land(); return

    # Post-sequence hover
    log(f"Post-sequence hover for {M1_POST_HOVER_S}s...")
    set_rc_hb(1500, 1500, 1500, 1500)
    sleep_check(M1_POST_HOVER_S)

    land_and_disarm()
    log("="*40); log("MISSION 1 COMPLETE"); log("="*40)

# ──────────────────────────────────────────────────────────────
#  MISSION 3  (Mission 1 + Yaw Hold)
# ──────────────────────────────────────────────────────────────
def run_mission3():
    log("="*40); log("MISSION 3 START (with YAW HOLD)"); log("="*40)

    # Takeoff mode
    while True:
        c = input("\nTakeoff mode:\n  1 - FLOWHOLD\n  2 - LOITER\nEnter (1 or 2): ").strip()
        if c=="1": arm_mode,wait_str="22","CMODE(22)"; break
        elif c=="2": arm_mode,wait_str="LOITER","LOITER"; break
        print("Invalid.")

    # Hover mode between commands
    while True:
        c = input("\nHover mode between commands:\n  1 - NEUTRAL (1500 in LOITER)\n  2 - FLOWHOLD\nEnter (1 or 2): ").strip()
        if c=="1": hover_mode="neutral"; break
        elif c=="2": hover_mode="flowhold"; break
        print("Invalid.")

    threading.Thread(target=_kb_thread, daemon=True).start()
    start_vio_guard()

    if not rc_takeoff(arm_mode, wait_str):
        abort_land("Takeoff failed"); return

    start_rc_heartbeat(1500)
    set_rc_hb(1500, 1500, 1500, 1500)

    # Capture yaw immediately after takeoff
    captured_yaw = get_yaw()
    log(f"Yaw captured at takeoff: {math.degrees(captured_yaw):.1f} deg")
    start_yaw_hold(captured_yaw)

    # Switch to LOITER
    if state.mode != "LOITER":
        log("Switching to LOITER...")
        if not set_mode("LOITER") or not wait_mode("LOITER"):
            abort_land("LOITER switch failed"); return

    # Pre-sequence hover
    log(f"Pre-sequence hover ({hover_mode}) for {M1_PRE_HOVER_S}s...")
    if not do_hover(M1_PRE_HOVER_S, hover_mode): do_land(); return

    # Run sweep (yaw hold thread updates rc_hb_vals[3] continuously)
    if not run_sweep_sequence(hover_mode, use_yaw_hold=True): do_land(); return

    # Post-sequence hover
    log(f"Post-sequence hover for {M1_POST_HOVER_S}s...")
    set_rc_hb(1500, 1500, 1500, 1500)
    sleep_check(M1_POST_HOVER_S)

    stop_yaw_hold()
    land_and_disarm()
    log("="*40); log("MISSION 3 COMPLETE"); log("="*40)

# ──────────────────────────────────────────────────────────────
#  SIGINT + EXCEPTION HANDLER
# ──────────────────────────────────────────────────────────────
def sigint_handler(sig, frame):
    warn("Ctrl+C -> Emergency land!")
    abort_flag.set(); do_land(); time.sleep(3.0); arm(False); _restore_kb(); sys.exit(0)

# ──────────────────────────────────────────────────────────────
#  MAIN
# ──────────────────────────────────────────────────────────────
def main():
    global rc_pub, sp_pub, arm_srv, mode_srv
    rospy.init_node("vio_mission_m1_m3", anonymous=False)
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

    mission = 0
    while mission not in (1, 3) and not rospy.is_shutdown():
        try:
            mission = int(input("\nSelect mission:\n  1 - RC Roll/Pitch sweep (LOITER)\n  3 - Same + Yaw Hold\nEnter (1 or 3): ").strip())
        except ValueError: pass

    try:
        if mission == 1: run_mission1()
        elif mission == 3: run_mission3()
    except Exception as e:
        err(f"Unhandled exception: {e}")
        abort_flag.set(); do_land(); time.sleep(3.0); arm(False)
    finally:
        _restore_kb()
        rospy.signal_shutdown("Mission complete.")

if __name__ == "__main__":
    main()
