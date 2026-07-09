#!/usr/bin/env python3
"""
corner_VerticalTakeoff_arena_loiter_rtl.py
==========================================
Mission:
  1) LOITER takeoff → FLOWHOLD 10s → FLOWHOLD climb → FLOWHOLD hover →
     LOITER 5s neutral
  2) Corner detection: Yaw180 → pitch fwd until yellow FRONT → roll right
     until yellow RIGHT → CORNER DETECTED → Yaw180 back
  3) Arena loiter sweep: roll right until yellow → pitch fwd timer →
     roll left until yellow → [loops] → Yaw180
  4) FLOWHOLD step-descend → hover → LAND

Safety: 'l' key, Ctrl+C, VIO divergence → LAND.
Requires: yellow_border_node.py running.
"""

import rospy, threading, sys, time, signal, termios, tty, select
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String

# ══════════════════════════════════════════════════════════════
#  PARAMETERS  (edit here)
# ══════════════════════════════════════════════════════════════

# Takeoff ramp
TAKEOFF_PWM_START  = 1400
TAKEOFF_PWM_MAX    = 1750
TAKEOFF_PWM_STEP   = 10
TAKEOFF_TICK_S     = 0.3
GROUND_ALT_OFFSET  = 0.10
TAKEOFF_TARGET_OFF = 1.0    # initial takeoff height above ground (m)

# FLOWHOLD climb
CLIMB_ALT_M        = 1.75   # climb target in FLOWHOLD (m above ground)
CLIMB_PWM_STEP     = 1      # throttle increment per tick
CLIMB_PWM_MAX      = 1800   # max throttle safety limit

# FLOWHOLD descent (landing)
DESCEND_ALT_M      = 1.0    # descend target in FLOWHOLD (m above ground)
DESCEND_PWM_STEP   = 1      # throttle decrement per tick
DESCEND_PWM_MIN    = 1300   # min throttle safety floor
LOOP_RATE_HZ       = 10

# Hover durations
HOVER_FH_INITIAL_S     = 10.0  # FLOWHOLD hover after takeoff (s)
HOVER_FH_POST_CLIMB_S  = 10.0  # FLOWHOLD hover after climb (s)
HOVER_LOITER_PRE_S     = 5.0   # LOITER hover before corner detection (s)
HOVER_LOITER_POST_YAW_S = 5.0  # LOITER hover after each yaw 180 (s)
HOVER_FH_POST_DESCEND_S = 10.0 # FLOWHOLD hover after descent (s)

# Yaw 180°
YAW_180_PWM    = 1560
YAW_180_TIME_S = 8.0

# Corner detection
CORNER_PITCH_PWM     = 1470  # pitch forward PWM for corner detection
CORNER_PITCH_TIMEOUT = 30.0  # max s pitching until yellow front
CORNER_ROLL_R_PWM    = 1530  # roll right PWM for corner detection
CORNER_ROLL_TIMEOUT  = 30.0  # max s rolling until yellow right
CORNER_YELLOW_HOVER_S = 3.0  # neutral hover after yellow detected

# Arena sweep
N_LOOPS          = 1
ROLL_R_PWM       = 1530
ROLL_L_PWM       = 1470
ROLL_TIMEOUT     = 30.0
PITCH_F_PWM      = 1470
T_PITCH          = 8.0
SHIFT_PITCH_PWM  = 1470
T_SHIFT          = 10.0
YELLOW_STOP_HOVER_S = 5.0
HOVER_S          = 5.0

# RTL return
RTL_WAIT_S         = 50.0  # seconds to wait for RTL to complete before forcing LAND
RETURN_PITCH_PWM   = 1470  # pitch forward PWM for return
RETURN_EXTRA_PITCH_S = 0.5 # extra pitch after yellow front detected

# Land
LAND_WAIT_S      = 20.0

# VIO guard
VIO_XY_LIMIT_M   = 400.0
VIO_CHECK_INTV_S = 0.5

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

rc_pub = arm_srv = mode_srv = None

# ══════════════════════════════════════════════════════════════
#  CALLBACKS
# ══════════════════════════════════════════════════════════════
def state_cb(msg):      global state;       state = msg
def local_pos_cb(msg):  global local_pose;  local_pose = msg
def vision_cb(msg):     global vision_pose; vision_pose = msg
def rf_cb(msg):         global rf_range;    rf_range = msg.range

def yellow_cb(msg):
    global yellow_status, yellow_connected
    with yellow_status_lock: yellow_status = msg.data
    yellow_connected = True

# ══════════════════════════════════════════════════════════════
#  LOGGING
# ══════════════════════════════════════════════════════════════
def log(m):  rospy.loginfo(f"[CVT-RTL] {m}")
def warn(m): rospy.logwarn(f"[WARN]    {m}")
def err(m):  rospy.logerr(f"[ERROR]   {m}")

# ══════════════════════════════════════════════════════════════
#  YELLOW HELPERS
# ══════════════════════════════════════════════════════════════
def yellow_active(region):
    with yellow_status_lock: return region in yellow_status

def get_yellow():
    with yellow_status_lock: return yellow_status

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

def _ok():
    return not abort_flag.is_set() and not land_flag.is_set() and not rospy.is_shutdown()

# ══════════════════════════════════════════════════════════════
#  RC OVERRIDE
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
#  RC TAKEOFF
# ══════════════════════════════════════════════════════════════
def rc_takeoff():
    log("Switch to LOITER")
    if not set_mode("LOITER"): return False
    if not wait_mode("LOITER"): return False
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
#  HOVER HELPERS
# ══════════════════════════════════════════════════════════════
def do_loiter_neutral(duration_s):
    """Neutral hover in LOITER."""
    set_rc_hb(1500, 1500, 1500, 1500)
    return sleep_check(duration_s)

def do_hover(duration_s, hover_mode):
    """Arena sweep hover — neutral in LOITER or FLOWHOLD."""
    if hover_mode == "flowhold":
        set_mode("22"); wait_mode("CMODE(22)", timeout=5)
        set_rc_hb(1500,1500,1500,1500)
        ok = sleep_check(duration_s)
        set_mode("LOITER"); wait_mode("LOITER", timeout=5)
        return ok
    else:
        set_rc_hb(1500,1500,1500,1500)
        return sleep_check(duration_s)

# ══════════════════════════════════════════════════════════════
#  FLOWHOLD CLIMB / DESCEND
# ══════════════════════════════════════════════════════════════
def flowhold_climb(ground):
    target_alt = ground + CLIMB_ALT_M
    log(f"Climbing to {CLIMB_ALT_M}m in FLOWHOLD (target: {target_alt:.2f}m)...")
    curr_throttle = 1500; r = rospy.Rate(LOOP_RATE_HZ)
    while _ok():
        if rf_range >= target_alt:
            log(f"Climb alt reached: {rf_range:.2f}m"); break
        if curr_throttle < CLIMB_PWM_MAX: curr_throttle += CLIMB_PWM_STEP
        set_rc_hb(throttle=curr_throttle); r.sleep()
    set_rc_hb(throttle=1500)
    return _ok()

def flowhold_descend(ground):
    target_alt = ground + DESCEND_ALT_M
    log(f"Descending to {DESCEND_ALT_M}m in FLOWHOLD (target: {target_alt:.2f}m)...")
    curr_throttle = 1500; r = rospy.Rate(LOOP_RATE_HZ)
    while _ok():
        if rf_range <= target_alt:
            log(f"Descend alt reached: {rf_range:.2f}m"); break
        if curr_throttle > DESCEND_PWM_MIN: curr_throttle -= DESCEND_PWM_STEP
        set_rc_hb(throttle=curr_throttle); r.sleep()
    set_rc_hb(throttle=1500)
    return _ok()

# --------------------------------------------------------------
#  MOTION HELPERS � CORNER DETECTION (LOITER neutral hovers)
# --------------------------------------------------------------
def corner_yaw_180():
    log(f"Yaw 180 ({YAW_180_PWM}) for {YAW_180_TIME_S}s")
    set_rc_hb(roll=1500, pitch=1500, throttle=1500, yaw=YAW_180_PWM)
    if not sleep_check(YAW_180_TIME_S): return False
    set_rc_hb(1500, 1500, 1500, 1500)
    log(f"Yaw done. LOITER neutral {HOVER_LOITER_POST_YAW_S}s...")
    return do_loiter_neutral(HOVER_LOITER_POST_YAW_S)

def corner_pitch_fwd_until_front():
    log(f"Pitch fwd ({CORNER_PITCH_PWM}) until yellow FRONT (timeout {CORNER_PITCH_TIMEOUT}s)...")
    set_rc_hb(roll=1500, pitch=CORNER_PITCH_PWM, throttle=1500, yaw=1500)
    t0 = time.time()
    while _ok():
        if yellow_active("front"):
            log("Yellow FRONT! Stopping.")
            set_rc_hb(1500, 1500, 1500, 1500)
            return do_loiter_neutral(CORNER_YELLOW_HOVER_S)
        if time.time()-t0 > CORNER_PITCH_TIMEOUT:
            warn("Corner pitch timeout. Stopping.")
            set_rc_hb(1500, 1500, 1500, 1500)
            return do_loiter_neutral(CORNER_YELLOW_HOVER_S)
        time.sleep(0.05)
    return False

def corner_roll_right_until_right():
    log(f"Roll right ({CORNER_ROLL_R_PWM}) until yellow RIGHT (timeout {CORNER_ROLL_TIMEOUT}s)...")
    set_rc_hb(roll=CORNER_ROLL_R_PWM, pitch=1500, throttle=1500, yaw=1500)
    t0 = time.time()
    while _ok():
        if yellow_active("right"):
            log("Yellow RIGHT! Stopping.")
            set_rc_hb(1500, 1500, 1500, 1500)
            return do_loiter_neutral(CORNER_YELLOW_HOVER_S)
        if time.time()-t0 > CORNER_ROLL_TIMEOUT:
            warn("Corner roll timeout. Stopping.")
            set_rc_hb(1500, 1500, 1500, 1500)
            return do_loiter_neutral(CORNER_YELLOW_HOVER_S)
        time.sleep(0.05)
    return False

# --------------------------------------------------------------
#  MOTION HELPERS � ARENA SWEEP
# --------------------------------------------------------------
def arena_yaw_180(hover_mode):
    log(f"Arena yaw 180 ({YAW_180_PWM}) for {YAW_180_TIME_S}s")
    set_rc_hb(roll=1500, pitch=1500, throttle=1500, yaw=YAW_180_PWM)
    if not sleep_check(YAW_180_TIME_S): return False
    set_rc_hb(1500, 1500, 1500, 1500)
    return do_hover(HOVER_S, hover_mode)

def roll_until_yellow(roll_pwm, yellow_region, timeout, hover_mode):
    direction = "RIGHT" if roll_pwm > 1500 else "LEFT"
    log(f"Roll {direction} ({roll_pwm}) until yellow '{yellow_region}' (timeout {timeout}s)")
    set_rc_hb(roll=roll_pwm, pitch=1500, throttle=1500, yaw=1500)
    t0 = time.time()
    while _ok():
        if yellow_active(yellow_region):
            log(f"Yellow '{yellow_region}' detected!")
            set_rc_hb(1500, 1500, 1500, 1500)
            return do_hover(YELLOW_STOP_HOVER_S, hover_mode)
        if time.time()-t0 > timeout:
            warn(f"Roll timeout {timeout}s without yellow '{yellow_region}'.")
            set_rc_hb(1500, 1500, 1500, 1500)
            return do_hover(YELLOW_STOP_HOVER_S, hover_mode)
        time.sleep(0.05)
    return False

def pitch_timed(pitch_pwm, duration, hover_mode, check_front_yellow=True):
    direction = "FORWARD" if pitch_pwm < 1500 else "BACKWARD"
    log(f"Pitch {direction} ({pitch_pwm}) for {duration}s")
    set_rc_hb(roll=1500, pitch=pitch_pwm, throttle=1500, yaw=1500)
    t0 = time.time()
    while _ok():
        if time.time()-t0 >= duration:
            set_rc_hb(1500, 1500, 1500, 1500)
            return do_hover(HOVER_S, hover_mode)
        if check_front_yellow and yellow_active("front"):
            log("Yellow FRONT during pitch! Stopping.")
            set_rc_hb(1500, 1500, 1500, 1500)
            return do_hover(YELLOW_STOP_HOVER_S, hover_mode)
        time.sleep(0.05)
    return False

# --------------------------------------------------------------
#  MISSION
# --------------------------------------------------------------
def run_mission():
    log("="*55)
    log("CORNER + VERTICAL TAKEOFF + ARENA LOITER RTL")
    log("="*55)
    log(f"Takeoff: {TAKEOFF_TARGET_OFF}m  Climb: {CLIMB_ALT_M}m  Descend: {DESCEND_ALT_M}m")
    log(f"Arena loops: {N_LOOPS}  Roll R/L: {ROLL_R_PWM}/{ROLL_L_PWM}")
    log(f"Pitch: {PITCH_F_PWM}/{T_PITCH}s  Shift: {SHIFT_PITCH_PWM}/{T_SHIFT}s")
    log(f"Yaw180: {YAW_180_PWM} for {YAW_180_TIME_S}s")

    if not yellow_connected:
        warn("yellow_border_node NOT connected!")
        c = input("Continue without yellow? (y/n): ").strip().lower()
        if c != "y": log("Aborted."); return

    while True:
        c = input("\nArena hover mode:\n  1 - NEUTRAL (LOITER)\n  2 - FLOWHOLD\nEnter: ").strip()
        if c == "1": hover_mode = "neutral"; break
        elif c == "2": hover_mode = "flowhold"; break
        print("Invalid.")

    threading.Thread(target=_kb_thread, daemon=True).start()
    start_vio_guard()

    log("Calibrating ground level...")
    rospy.sleep(1.0)
    ground = rf_range
    if ground == 0.0:
        warn("Rangefinder 0.0 - waiting...")
        while not rospy.is_shutdown() and rf_range == 0.0: rospy.sleep(0.5)
        ground = rf_range
    log(f"Ground: {ground:.2f}m")

    # 1a) LOITER TAKEOFF
    if not rc_takeoff(): abort_land("Takeoff failed"); return
    start_rc_heartbeat(); set_rc_hb(1500, 1500, 1500, 1500)
    if state.mode != "LOITER":
        if not set_mode("LOITER") or not wait_mode("LOITER"):
            abort_land("LOITER switch failed"); return

    # 1b) FLOWHOLD hover 10s
    if not set_mode("22") or not wait_mode("CMODE(22)", timeout=5):
        abort_land("FLOWHOLD switch failed"); return
    log(f"FLOWHOLD hover {HOVER_FH_INITIAL_S}s...")
    if not sleep_check(HOVER_FH_INITIAL_S): land_and_disarm(); return

    # 1c) FLOWHOLD step climb
    if not flowhold_climb(ground): land_and_disarm(); return

    # 1d) FLOWHOLD post-climb hover
    log(f"Post-climb FLOWHOLD hover {HOVER_FH_POST_CLIMB_S}s...")
    if not sleep_check(HOVER_FH_POST_CLIMB_S): land_and_disarm(); return

    # 1e) LOITER 5s neutral hover
    if not set_mode("LOITER") or not wait_mode("LOITER", timeout=5):
        abort_land("LOITER switch failed"); return
    if not do_loiter_neutral(HOVER_LOITER_PRE_S): land_and_disarm(); return
    if not _ok(): land_and_disarm(); return

    # 2) CORNER DETECTION
    log("=== CORNER DETECTION START ===")
    if not corner_yaw_180(): land_and_disarm(); return
    if not _ok(): land_and_disarm(); return
    if not corner_pitch_fwd_until_front(): land_and_disarm(); return
    if not _ok(): land_and_disarm(); return
    if not corner_roll_right_until_right(): land_and_disarm(); return
    if not _ok(): land_and_disarm(); return
    log("*** CORNER DETECTED ***")
    if not corner_yaw_180(): land_and_disarm(); return
    if not _ok(): land_and_disarm(); return
    log("=== CORNER DETECTION COMPLETE - STARTING ARENA SWEEP ===")

    # 3) ARENA LOITER SWEEP
    log(f"Pre-sweep LOITER hover {HOVER_S}s...")
    if not do_hover(HOVER_S, hover_mode): land_and_disarm(); return

    for loop in range(1, N_LOOPS+1):
        log(f"=== Loop {loop}/{N_LOOPS} ===")
        if not _ok(): land_and_disarm(); return
        if not roll_until_yellow(ROLL_R_PWM, "right", ROLL_TIMEOUT, hover_mode):
            land_and_disarm(); return
        if not _ok(): land_and_disarm(); return
        if not pitch_timed(PITCH_F_PWM, T_PITCH, hover_mode, check_front_yellow=True):
            land_and_disarm(); return
        if not _ok(): land_and_disarm(); return
        if not roll_until_yellow(ROLL_L_PWM, "left", ROLL_TIMEOUT, hover_mode):
            land_and_disarm(); return
        if not _ok(): land_and_disarm(); return
        if loop < N_LOOPS:
            if not pitch_timed(SHIFT_PITCH_PWM, T_SHIFT, hover_mode, check_front_yellow=True):
                land_and_disarm(); return

    log("All sweep loops complete.")
    if not _ok(): land_and_disarm(); return

    # Arena yaw 180
    if not arena_yaw_180(hover_mode): land_and_disarm(); return
    if not _ok(): land_and_disarm(); return

    # -- RTL -- drone returns and lands automatically
    log("Activating RTL - drone will return to base.")
    set_mode("RTL")
    log(f"Monitoring yellow-front during RTL (up to {RTL_WAIT_S}s)...")
    t_rtl = time.time()
    yellow_intercept = False
    rate = rospy.Rate(10)
    while _ok() and (time.time() - t_rtl) < RTL_WAIT_S:
        if yellow_active("front"):
            log("Yellow FRONT during RTL! Cancelling RTL -> hover -> new landing.")
            yellow_intercept = True
            break
        rate.sleep()
    if yellow_intercept:
        log(f"Yellow intercept: hovering {YELLOW_STOP_HOVER_S}s...")
        do_hover(YELLOW_STOP_HOVER_S, hover_mode)
    else:
        log("RTL timeout - proceeding to FLOWHOLD descent.")
    if not _ok(): land_and_disarm(); return
    # -- 4) FLOWHOLD DESCENT + LAND --
    log("Switching to FLOWHOLD for descent...")
    if not set_mode("22") or not wait_mode("CMODE(22)", timeout=5):
        abort_land("FLOWHOLD switch failed for descent"); return
    if not flowhold_descend(ground): land_and_disarm(); return
    log(f"Post-descent FLOWHOLD hover {HOVER_FH_POST_DESCEND_S}s...")
    if not sleep_check(HOVER_FH_POST_DESCEND_S): land_and_disarm(); return
    log("Landing...")
    land_and_disarm()
    log("="*55); log("MISSION COMPLETE"); log("="*55)

# --------------------------------------------------------------
#  SIGINT
# --------------------------------------------------------------
def sigint_handler(sig, frame):
    warn("Ctrl+C -> Emergency land!")
    abort_flag.set(); do_land(); time.sleep(3.0); arm(False); _restore_kb(); sys.exit(0)

# --------------------------------------------------------------
#  MAIN
# --------------------------------------------------------------
def main():
    global rc_pub, arm_srv, mode_srv
    rospy.init_node("corner_vt_arena_rtl", anonymous=False)
    signal.signal(signal.SIGINT, sigint_handler)
    rospy.Subscriber(f"{MAVROS_NS}/state",                   State,       state_cb)
    rospy.Subscriber(f"{MAVROS_NS}/local_position/pose",     PoseStamped, local_pos_cb)
    rospy.Subscriber(f"{MAVROS_NS}/vision_pose/pose",        PoseStamped, vision_cb)
    rospy.Subscriber(f"{MAVROS_NS}/rangefinder/rangefinder", Range,       rf_cb)
    rospy.Subscriber("/yellow_border/status",                 String,      yellow_cb)
    rc_pub = rospy.Publisher(f"{MAVROS_NS}/rc/override", OverrideRCIn, queue_size=5)
    log("Waiting for MAVROS services...")
    rospy.wait_for_service(f"{MAVROS_NS}/cmd/arming")
    rospy.wait_for_service(f"{MAVROS_NS}/set_mode")
    arm_srv  = rospy.ServiceProxy(f"{MAVROS_NS}/cmd/arming", CommandBool)
    mode_srv = rospy.ServiceProxy(f"{MAVROS_NS}/set_mode",   SetMode)
    log("Waiting for MAVROS connection...")
    r = rospy.Rate(2)
    while not rospy.is_shutdown() and not state.connected: r.sleep()
    log("MAVROS connected.")
    rospy.sleep(2.0)
    if yellow_connected: log(f"Yellow node connected. Status: {get_yellow()}")
    else: warn("Yellow node not detected - will ask during setup.")
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
