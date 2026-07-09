#!/usr/bin/env python3
"""
corner_detection.py  —  Corner detection using yellow border sensing
=====================================================================
Mission:
  1) RC Loiter takeoff → FLOWHOLD 10s hover → FLOWHOLD climb to CLIMB_ALT_M
     → FLOWHOLD hover 10s → LOITER 5s neutral hover
  2) Yaw 180° (LOITER, timer-based, 8s) → neutral hover →
     Pitch forward (1470) until yellow FRONT detected → neutral →
     Roll right  (1530) until yellow RIGHT detected → neutral →
     Log "CORNER DETECTED" →
     Yaw 180° (LOITER, timer-based, 8s) → neutral hover
  3) Switch to FLOWHOLD → step-descend to DESCEND_ALT_M → neutral hover 10s → LAND

Safety: 'l' key → land, Ctrl+C → emergency land, VIO divergence → abort land.
Requires: yellow_border_node.py running in a separate terminal.
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

# Flowhold climb target after initial 10s hover
CLIMB_ALT_M        = 1.75   # climb to this height in FLOWHOLD (m above ground)
CLIMB_PWM_STEP     = 1      # PWM step increment per control tick
CLIMB_PWM_MAX      = 1800   # max throttle safety limit during climb

# Flowhold descent target for landing sequence
DESCEND_ALT_M      = 1.0    # descend to this height in FLOWHOLD before landing (m above ground)
DESCEND_PWM_STEP   = 1      # PWM step decrement per control tick
DESCEND_PWM_MIN    = 1400   # min throttle safety floor during descent

# Control loop rate
LOOP_RATE_HZ       = 10

# General hover durations
HOVER_FH_INITIAL_S = 10.0   # flowhold hover after takeoff (s)
HOVER_FH_POST_CLIMB_S = 10.0  # flowhold hover after climb (s)
HOVER_LOITER_PRE_S = 5.0    # loiter neutral hover before corner detection (s)
HOVER_LOITER_POST_YAW_S = 5.0  # loiter neutral hover after each yaw 180 (s)
HOVER_FH_POST_DESCEND_S = 10.0 # flowhold hover after descend (s)
YELLOW_STOP_HOVER_S = 3.0   # hover after yellow detection (s)

# Yaw 180°
YAW_180_PWM    = 1560       # yaw channel PWM for 180° turn
YAW_180_TIME_S = 8.0        # timer for 180° at this PWM

# Corner detection motion
PITCH_F_PWM    = 1470       # pitch forward PWM
PITCH_TIMEOUT  = 30.0       # max seconds pitching forward before giving up
ROLL_R_PWM     = 1530       # roll right PWM
ROLL_TIMEOUT   = 30.0       # max seconds rolling right before giving up

# Land
LAND_WAIT_S    = 20.0

# VIO divergence guard
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
    with yellow_status_lock:
        yellow_status = msg.data
    yellow_connected = True

# ══════════════════════════════════════════════════════════════
#  LOGGING
# ══════════════════════════════════════════════════════════════
def log(m):  rospy.loginfo(f"[CORNER] {m}")
def warn(m): rospy.logwarn(f"[WARN]   {m}")
def err(m):  rospy.logerr(f"[ERROR]  {m}")

# ══════════════════════════════════════════════════════════════
#  YELLOW HELPERS
# ══════════════════════════════════════════════════════════════
def yellow_active(region):
    """Check if a region ('left','right','front') is currently detected."""
    with yellow_status_lock:
        return region in yellow_status

def get_yellow():
    """Return current yellow status string."""
    with yellow_status_lock:
        return yellow_status

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
#  RC TAKEOFF  (identical to arena_nav_loiter_rtl.py)
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
#  HOVER HELPER  (LOITER neutral only — copied from rtl.py pattern)
# ══════════════════════════════════════════════════════════════
def do_loiter_hover(duration_s):
    """Hover neutrally in LOITER for duration_s."""
    set_rc_hb(1500, 1500, 1500, 1500)
    return sleep_check(duration_s)

# ══════════════════════════════════════════════════════════════
#  MOTION HELPERS  (identical to arena_nav_loiter_rtl.py)
# ══════════════════════════════════════════════════════════════
def yaw_180():
    """Timer-based yaw 180° in LOITER."""
    log(f"Yaw 180° ({YAW_180_PWM}) for {YAW_180_TIME_S}s")
    set_rc_hb(roll=1500, pitch=1500, throttle=1500, yaw=YAW_180_PWM)
    if not sleep_check(YAW_180_TIME_S): return False
    set_rc_hb(1500, 1500, 1500, 1500)
    log(f"Yaw done. Holding neutral for {HOVER_LOITER_POST_YAW_S}s...")
    return do_loiter_hover(HOVER_LOITER_POST_YAW_S)

def pitch_until_yellow_front():
    """Pitch forward (PITCH_F_PWM) until yellow 'front' detected, then neutral."""
    log(f"Pitching forward ({PITCH_F_PWM}) until yellow FRONT (timeout {PITCH_TIMEOUT}s)...")
    set_rc_hb(roll=1500, pitch=PITCH_F_PWM, throttle=1500, yaw=1500)
    t0 = time.time()
    while _ok():
        if yellow_active("front"):
            log("Yellow FRONT detected! Stopping pitch.")
            set_rc_hb(1500, 1500, 1500, 1500)
            log(f"Hovering neutral {YELLOW_STOP_HOVER_S}s...")
            return do_loiter_hover(YELLOW_STOP_HOVER_S)
        if time.time()-t0 > PITCH_TIMEOUT:
            warn(f"Pitch timeout {PITCH_TIMEOUT}s — no yellow front. Stopping.")
            set_rc_hb(1500, 1500, 1500, 1500)
            return do_loiter_hover(YELLOW_STOP_HOVER_S)
        time.sleep(0.05)
    return False

def roll_until_yellow_right():
    """Roll right (ROLL_R_PWM) until yellow 'right' detected, then neutral."""
    log(f"Rolling right ({ROLL_R_PWM}) until yellow RIGHT (timeout {ROLL_TIMEOUT}s)...")
    set_rc_hb(roll=ROLL_R_PWM, pitch=1500, throttle=1500, yaw=1500)
    t0 = time.time()
    while _ok():
        if yellow_active("right"):
            log("Yellow RIGHT detected! Stopping roll.")
            set_rc_hb(1500, 1500, 1500, 1500)
            log(f"Hovering neutral {YELLOW_STOP_HOVER_S}s...")
            return do_loiter_hover(YELLOW_STOP_HOVER_S)
        if time.time()-t0 > ROLL_TIMEOUT:
            warn(f"Roll timeout {ROLL_TIMEOUT}s — no yellow right. Stopping.")
            set_rc_hb(1500, 1500, 1500, 1500)
            return do_loiter_hover(YELLOW_STOP_HOVER_S)
        time.sleep(0.05)
    return False

# ══════════════════════════════════════════════════════════════
#  FLOWHOLD STEP CLIMB
# ══════════════════════════════════════════════════════════════
def flowhold_climb(ground):
    """Step-increment throttle in FLOWHOLD until CLIMB_ALT_M is reached."""
    target_alt = ground + CLIMB_ALT_M
    log(f"Climbing to {CLIMB_ALT_M}m in FLOWHOLD (target: {target_alt:.2f}m)...")
    curr_throttle = 1500
    r = rospy.Rate(LOOP_RATE_HZ)
    while _ok():
        if rf_range >= target_alt:
            log(f"Climb altitude reached: {rf_range:.2f}m")
            break
        if curr_throttle < CLIMB_PWM_MAX:
            curr_throttle += CLIMB_PWM_STEP
        set_rc_hb(throttle=curr_throttle)
        r.sleep()
    set_rc_hb(throttle=1500)
    return _ok()

# ══════════════════════════════════════════════════════════════
#  FLOWHOLD STEP DESCEND
# ══════════════════════════════════════════════════════════════
def flowhold_descend(ground):
    """Step-decrement throttle in FLOWHOLD until DESCEND_ALT_M is reached."""
    target_alt = ground + DESCEND_ALT_M
    log(f"Descending to {DESCEND_ALT_M}m in FLOWHOLD (target: {target_alt:.2f}m)...")
    curr_throttle = 1500
    r = rospy.Rate(LOOP_RATE_HZ)
    while _ok():
        if rf_range <= target_alt:
            log(f"Descend altitude reached: {rf_range:.2f}m")
            break
        if curr_throttle > DESCEND_PWM_MIN:
            curr_throttle -= DESCEND_PWM_STEP
        set_rc_hb(throttle=curr_throttle)
        r.sleep()
    set_rc_hb(throttle=1500)
    return _ok()

# ══════════════════════════════════════════════════════════════
#  MISSION
# ══════════════════════════════════════════════════════════════
def run_mission():
    log("="*55)
    log("CORNER DETECTION MISSION")
    log("="*55)
    log(f"Takeoff: {TAKEOFF_TARGET_OFF}m  Climb: {CLIMB_ALT_M}m  Descend: {DESCEND_ALT_M}m")
    log(f"Yaw180: {YAW_180_PWM} PWM for {YAW_180_TIME_S}s")
    log(f"Pitch fwd: {PITCH_F_PWM}  Roll right: {ROLL_R_PWM}")

    # Yellow node check
    if not yellow_connected:
        warn("yellow_border_node NOT connected!")
        c = input("Continue without yellow detection? (y/n): ").strip().lower()
        if c != 'y': log("Aborted."); return

    threading.Thread(target=_kb_thread, daemon=True).start()
    start_vio_guard()

    # Calibrate ground
    log("Calibrating ground level...")
    rospy.sleep(1.0)
    ground = rf_range
    if ground == 0.0:
        warn("Rangefinder is 0.0 — waiting for valid data...")
        while not rospy.is_shutdown() and rf_range == 0.0:
            rospy.sleep(0.5)
        ground = rf_range
    log(f"Ground calibrated: {ground:.2f}m")

    # ── 1a) LOITER TAKEOFF ──
    log("Starting LOITER takeoff...")
    if not rc_takeoff("LOITER", "LOITER"):
        abort_land("Takeoff failed"); return

    start_rc_heartbeat()
    set_rc_hb(1500, 1500, 1500, 1500)

    # Ensure LOITER after takeoff
    if state.mode != "LOITER":
        if not set_mode("LOITER") or not wait_mode("LOITER"):
            abort_land("LOITER switch failed"); return

    # ── 1b) SWITCH TO FLOWHOLD, INITIAL 10s HOVER ──
    log("Switching to FLOWHOLD for initial hover...")
    if not set_mode("22") or not wait_mode("CMODE(22)", timeout=5):
        abort_land("FLOWHOLD switch failed"); return

    log(f"Hovering in FLOWHOLD for {HOVER_FH_INITIAL_S}s...")
    if not sleep_check(HOVER_FH_INITIAL_S): land_and_disarm(); return

    # ── 1c) STEP CLIMB TO CLIMB_ALT_M IN FLOWHOLD ──
    if not flowhold_climb(ground): land_and_disarm(); return

    # ── 1d) HOVER POST-CLIMB IN FLOWHOLD ──
    log(f"Post-climb hover in FLOWHOLD for {HOVER_FH_POST_CLIMB_S}s...")
    if not sleep_check(HOVER_FH_POST_CLIMB_S): land_and_disarm(); return

    # ── 1e) SWITCH TO LOITER, 5s NEUTRAL HOVER ──
    log("Switching to LOITER for pre-mission hover...")
    if not set_mode("LOITER") or not wait_mode("LOITER", timeout=5):
        abort_land("LOITER switch failed"); return

    log(f"LOITER neutral hover for {HOVER_LOITER_PRE_S}s before corner detection...")
    if not do_loiter_hover(HOVER_LOITER_PRE_S): land_and_disarm(); return

    if not _ok(): land_and_disarm(); return

    # ── 2a) YAW 180° ──
    log("Starting corner detection sequence...")
    if not yaw_180(): land_and_disarm(); return

    if not _ok(): land_and_disarm(); return

    # ── 2b) PITCH FORWARD UNTIL YELLOW FRONT ──
    if not pitch_until_yellow_front(): land_and_disarm(); return

    if not _ok(): land_and_disarm(); return

    # ── 2c) ROLL RIGHT UNTIL YELLOW RIGHT ──
    if not roll_until_yellow_right(): land_and_disarm(); return

    if not _ok(): land_and_disarm(); return

    # ── 2d) CORNER DETECTED ──
    log("★★★ CORNER DETECTED ★★★")
    log("="*55)

    # ── 2e) YAW 180° BACK ──
    log("Yawing back 180°...")
    if not yaw_180(): land_and_disarm(); return

    if not _ok(): land_and_disarm(); return

    # ── 3a) SWITCH TO FLOWHOLD FOR DESCENT ──
    log("Switching to FLOWHOLD for descent sequence...")
    if not set_mode("22") or not wait_mode("CMODE(22)", timeout=5):
        abort_land("FLOWHOLD switch failed for descent"); return

    # ── 3b) STEP DESCEND TO DESCEND_ALT_M IN FLOWHOLD ──
    if not flowhold_descend(ground): land_and_disarm(); return

    # ── 3c) HOVER POST-DESCEND IN FLOWHOLD ──
    log(f"Post-descent hover in FLOWHOLD for {HOVER_FH_POST_DESCEND_S}s...")
    if not sleep_check(HOVER_FH_POST_DESCEND_S): land_and_disarm(); return

    # ── 3d) LAND ──
    log("Landing...")
    land_and_disarm()
    log("="*55)
    log("CORNER DETECTION MISSION COMPLETE")
    log("="*55)

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
    global rc_pub, arm_srv, mode_srv
    rospy.init_node("corner_detection", anonymous=False)
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
    if yellow_connected:
        log(f"Yellow border node connected. Status: {get_yellow()}")
    else:
        warn("Yellow border node not detected — will ask during mission setup.")

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
