#!/usr/bin/env python3
"""
arena_nav_loiter.py  —  Loiter-based arena sweep with yellow border detection
==============================================================================
Mission: RC takeoff → LOITER sweep → yellow-border-aware turns → yaw 180 → return → land

Loop (×N_LOOPS):
  Roll RIGHT until yellow LEFT → hover 5s →
  Pitch FORWARD T_PITCH sec (stop if yellow FRONT) → hover 5s →
  Roll LEFT until yellow RIGHT → hover 5s →
  [between loops: shift pitch T_SHIFT sec]

After loops:
  Yaw 180° → Pitch forward until yellow FRONT → hover → FLOWHOLD → LAND (20s)

Requires: yellow_border_node.py running in a separate terminal.

Safety: 'l' key, Ctrl+C, VIO divergence, yellow border, unhandled exception → all LAND with 20s timer.
"""

import rospy, threading, math, sys, time, signal, termios, tty, select
from mavros_msgs.msg import State, OverrideRCIn, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String, Bool, Float32

# ══════════════════════════════════════════════════════════════
#  PARAMETERS  (edit here)
# ══════════════════════════════════════════════════════════════

# Takeoff ramp
TAKEOFF_PWM_START  = 1400
TAKEOFF_PWM_MAX    = 1850
TAKEOFF_PWM_STEP   = 10
TAKEOFF_TICK_S     = 0.3
GROUND_ALT_OFFSET  = 0.10
TAKEOFF_TARGET_OFF = 1.5

# Number of sweep loops
N_LOOPS = 1

# Roll (until yellow border detected)
ROLL_R_PWM   = 1530   # roll right PWM — use +/- to flip direction
ROLL_L_PWM   = 1470   # roll left PWM
ROLL_TIMEOUT = 30.0   # max seconds for a roll before giving up

# Pitch (timer-based)
PITCH_F_PWM  = 1470   # pitch forward PWM
T_PITCH      = 8.0    # pitch forward duration (s)

# Shift pitch between loops
SHIFT_PITCH_PWM = 1470
T_SHIFT         = 10.0

# Hover duration after yellow border detection
YELLOW_STOP_HOVER_S = 5.0

# General hover between commands
HOVER_S = 5.0

# Yaw 180° (timer-based — tune on ground)
YAW_180_PWM    = 1560   # yaw channel PWM for turning
YAW_180_TIME_S = 8.0    # approximate time for 180° at this PWM

# Return pitch (after yaw 180, until yellow front detected)
RETURN_PITCH_PWM  = 1470
RETURN_TIMEOUT    = 30.0   # max seconds for return pitch
RETURN_EXTRA_PITCH_S = 0.5  # extra pitch (s) after yellow front first detected before stopping

# Land
LAND_WAIT_S = 20.0

# VIO divergence guard
VIO_XY_LIMIT_M   = 400.0
VIO_CHECK_INTV_S = 0.5

# Aruco precision landing — topic names (change these if node changes)
ARUCO_STATUS_TOPIC   = "/aruco/status"     # std_msgs/Bool
ARUCO_X_OFFSET_TOPIC = "/aruco/x_offset"   # std_msgs/Float32 (cm)
ARUCO_Y_OFFSET_TOPIC = "/aruco/y_offset"   # std_msgs/Float32 (cm)

# Aruco alignment thresholds (cm)
ARUCO_X_TARGET_CM  = 20.0   # x offset must be within ±this to land
ARUCO_Y_TARGET_CM  = 20.0   # y offset must be within ±this to land
# X correction: +x (marker ahead) → pitch 1470 (fwd), -x (marker behind) → pitch 1530 (bwd)
ARUCO_PITCH_FWD    = 1470   # pitch to decrease +x offset (move forward)
ARUCO_PITCH_BWD    = 1530   # pitch to decrease -x offset (move backward)
# Y correction: +y (marker right) → roll 1530 (right), -y (marker left) → roll 1470 (left)
ARUCO_ROLL_PWM_POS = 1530   # roll to decrease +y offset (move right)
ARUCO_ROLL_PWM_NEG = 1470   # roll to decrease -y offset (move left)
ARUCO_ALIGN_TIMEOUT = 20.0  # max seconds for EACH axis alignment

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

# Yellow border status
yellow_status     = "none"
yellow_status_lock = threading.Lock()
yellow_connected   = False

# Aruco state
aruco_status     = False
aruco_x_offset   = 0.0    # cm
aruco_y_offset   = 0.0    # cm
aruco_lock       = threading.Lock()
aruco_connected  = False

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

def aruco_status_cb(msg):
    global aruco_status, aruco_connected
    with aruco_lock:
        aruco_status = msg.data
    aruco_connected = True

def aruco_x_cb(msg):
    global aruco_x_offset
    with aruco_lock:
        aruco_x_offset = msg.data

def aruco_y_cb(msg):
    global aruco_y_offset
    with aruco_lock:
        aruco_y_offset = msg.data

# ══════════════════════════════════════════════════════════════
#  LOGGING
# ══════════════════════════════════════════════════════════════
def log(m):  rospy.loginfo(f"[ARENA-L] {m}")
def warn(m): rospy.logwarn(f"[WARN]    {m}")
def err(m):  rospy.logerr(f"[ERROR]   {m}")

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
#  ARUCO HELPERS
# ══════════════════════════════════════════════════════════════
def aruco_visible():
    with aruco_lock:
        return aruco_status

def get_aruco_offsets():
    """Returns (x_cm, y_cm)."""
    with aruco_lock:
        return aruco_x_offset, aruco_y_offset

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
            "Disarming ACCEPTED" if res.success else f"Arm REJECTED")
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
#  HOVER HELPER
# ══════════════════════════════════════════════════════════════
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

def _ok():
    return not abort_flag.is_set() and not land_flag.is_set() and not rospy.is_shutdown()

# ══════════════════════════════════════════════════════════════
#  MOTION HELPERS
# ══════════════════════════════════════════════════════════════
def roll_until_yellow(roll_pwm, yellow_region, timeout, hover_mode):
    """Roll at roll_pwm until yellow_region detected. Then hover YELLOW_STOP_HOVER_S."""
    direction = "RIGHT" if roll_pwm > 1500 else "LEFT"
    log(f"Roll {direction} ({roll_pwm}) until yellow '{yellow_region}' (timeout {timeout}s)")
    set_rc_hb(roll=roll_pwm, pitch=1500, throttle=1500, yaw=1500)
    t0 = time.time()
    while _ok():
        if yellow_active(yellow_region):
            log(f"Yellow '{yellow_region}' detected! Stopping.")
            set_rc_hb(1500,1500,1500,1500)
            return do_hover(YELLOW_STOP_HOVER_S, hover_mode)
        if time.time()-t0 > timeout:
            warn(f"Roll timeout {timeout}s without yellow '{yellow_region}'. Stopping.")
            set_rc_hb(1500,1500,1500,1500)
            return do_hover(YELLOW_STOP_HOVER_S, hover_mode)
        time.sleep(0.05)
    return False


def pitch_timed(pitch_pwm, duration, hover_mode, check_front_yellow=True):
    """Pitch for duration seconds. If check_front_yellow and yellow 'front' detected, stop early."""
    direction = "FORWARD" if pitch_pwm < 1500 else "BACKWARD"
    log(f"Pitch {direction} ({pitch_pwm}) for {duration}s (yellow_front_check={check_front_yellow})")
    set_rc_hb(roll=1500, pitch=pitch_pwm, throttle=1500, yaw=1500)
    t0 = time.time()
    while _ok():
        if time.time()-t0 >= duration:
            log("Pitch timer done.")
            set_rc_hb(1500,1500,1500,1500)
            return do_hover(HOVER_S, hover_mode)
        if check_front_yellow and yellow_active("front"):
            log("Yellow 'front' detected during pitch! Stopping.")
            set_rc_hb(1500,1500,1500,1500)
            return do_hover(YELLOW_STOP_HOVER_S, hover_mode)
        time.sleep(0.05)
    return False


def yaw_180(hover_mode):
    """Timer-based yaw 180°."""
    log(f"Yaw 180° ({YAW_180_PWM}) for {YAW_180_TIME_S}s")
    set_rc_hb(roll=1500, pitch=1500, throttle=1500, yaw=YAW_180_PWM)
    if not sleep_check(YAW_180_TIME_S): return False
    set_rc_hb(1500,1500,1500,1500)
    return do_hover(HOVER_S, hover_mode)


def pitch_until_yellow_front(pitch_pwm, timeout):
    """Pitch forward until yellow 'front' detected, then keep pitching for
    RETURN_EXTRA_PITCH_S more seconds before stopping (aruco fully ignored).
    Returns True on yellow detection, False on timeout or abort.
    """
    log(f"Return pitch ({pitch_pwm}) until yellow-front (timeout {timeout}s)")
    set_rc_hb(roll=1500, pitch=pitch_pwm, throttle=1500, yaw=1500)
    t0 = time.time()
    yellow_seen = None  # timestamp when yellow front was first detected

    while _ok():
        elapsed = time.time() - t0

        if yellow_seen is None:
            if yellow_active("front"):
                yellow_seen = time.time()
                log(f"Yellow 'front' detected! Continuing pitch for {RETURN_EXTRA_PITCH_S}s...")
        else:
            # Extra pitch window — keep going a bit further to centre over pad
            if time.time() - yellow_seen >= RETURN_EXTRA_PITCH_S:
                log("Return extra-pitch complete. Stopping.")
                set_rc_hb(1500,1500,1500,1500)
                return True

        if elapsed > timeout:
            warn(f"Return pitch timeout {timeout}s. Stopping.")
            set_rc_hb(1500,1500,1500,1500)
            return False

        time.sleep(0.05)
    return False


def aruco_align_x(hover_mode):
    """Correct X offset using pitch until within ±ARUCO_X_TARGET_CM. Runs in LOITER.
    +x → pitch ARUCO_PITCH_FWD (1470, forward) to decrease positive offset
    -x → pitch ARUCO_PITCH_BWD (1530, backward) to decrease negative offset
    """
    log(f"Aruco X alignment: target ±{ARUCO_X_TARGET_CM}cm (timeout {ARUCO_ALIGN_TIMEOUT}s)")

    if state.mode != "LOITER":
        set_mode("LOITER"); wait_mode("LOITER", timeout=5)

    t0 = time.time()
    while _ok():
        if not aruco_visible():
            set_rc_hb(1500,1500,1500,1500)
            time.sleep(0.1)
            if time.time()-t0 > ARUCO_ALIGN_TIMEOUT:
                warn("Aruco lost during X align. Timeout."); break
            continue

        ax, _ = get_aruco_offsets()

        if abs(ax) <= ARUCO_X_TARGET_CM:
            log(f"X aligned! x_offset={ax:.1f}cm (within ±{ARUCO_X_TARGET_CM}cm)")
            set_rc_hb(1500,1500,1500,1500)
            return do_hover(HOVER_S, hover_mode)

        # +x → pitch forward (1470), -x → pitch backward (1530)
        if ax > 0:
            log(f"  +x={ax:.1f}cm → pitch FWD ({ARUCO_PITCH_FWD})")
            set_rc_hb(roll=1500, pitch=ARUCO_PITCH_FWD, throttle=1500, yaw=1500)
        else:
            log(f"  -x={ax:.1f}cm → pitch BWD ({ARUCO_PITCH_BWD})")
            set_rc_hb(roll=1500, pitch=ARUCO_PITCH_BWD, throttle=1500, yaw=1500)

        if time.time()-t0 > ARUCO_ALIGN_TIMEOUT:
            warn(f"X align timeout. x_offset={ax:.1f}cm. Continuing to Y align.")
            set_rc_hb(1500,1500,1500,1500)
            break

        time.sleep(0.05)

    set_rc_hb(1500,1500,1500,1500)
    return do_hover(HOVER_S, hover_mode)


def aruco_align_y(hover_mode):
    """Correct Y offset using roll until within ±ARUCO_Y_TARGET_CM. Runs in LOITER.
    +y → roll ARUCO_ROLL_PWM_POS (1530, right) to decrease positive offset
    -y → roll ARUCO_ROLL_PWM_NEG (1470, left) to decrease negative offset
    """
    log(f"Aruco Y alignment: target ±{ARUCO_Y_TARGET_CM}cm (timeout {ARUCO_ALIGN_TIMEOUT}s)")

    if state.mode != "LOITER":
        set_mode("LOITER"); wait_mode("LOITER", timeout=5)

    t0 = time.time()
    while _ok():
        if not aruco_visible():
            set_rc_hb(1500,1500,1500,1500)
            time.sleep(0.1)
            if time.time()-t0 > ARUCO_ALIGN_TIMEOUT:
                warn("Aruco lost during Y align. Timeout."); break
            continue

        _, ay = get_aruco_offsets()

        if abs(ay) <= ARUCO_Y_TARGET_CM:
            log(f"Y aligned! y_offset={ay:.1f}cm (within ±{ARUCO_Y_TARGET_CM}cm)")
            set_rc_hb(1500,1500,1500,1500)
            return do_hover(HOVER_S, hover_mode)

        # +y → roll right (1530), -y → roll left (1470)
        if ay > 0:
            log(f"  +y={ay:.1f}cm → roll RIGHT ({ARUCO_ROLL_PWM_POS})")
            set_rc_hb(roll=ARUCO_ROLL_PWM_NEG, pitch=1500, throttle=1500, yaw=1500)
        else:
            log(f"  -y={ay:.1f}cm → roll LEFT ({ARUCO_ROLL_PWM_NEG})")
            set_rc_hb(roll=ARUCO_ROLL_PWM_POS, pitch=1500, throttle=1500, yaw=1500)

        if time.time()-t0 > ARUCO_ALIGN_TIMEOUT:
            warn(f"Y align timeout. y_offset={ay:.1f}cm. Landing anyway.")
            set_rc_hb(1500,1500,1500,1500)
            break

        time.sleep(0.05)

    set_rc_hb(1500,1500,1500,1500)
    return do_hover(HOVER_S, hover_mode)

# ══════════════════════════════════════════════════════════════
#  MISSION
# ══════════════════════════════════════════════════════════════
def run_mission():
    log("="*50); log("ARENA NAV — LOITER SWEEP"); log("="*50)
    log(f"Loops: {N_LOOPS}")
    log(f"Roll R/L: {ROLL_R_PWM}/{ROLL_L_PWM}  Pitch: {PITCH_F_PWM}/{T_PITCH}s")
    log(f"Shift: {SHIFT_PITCH_PWM}/{T_SHIFT}s  Yaw180: {YAW_180_PWM}/{YAW_180_TIME_S}s")
    log(f"Return: {RETURN_PITCH_PWM}  Yellow hover: {YELLOW_STOP_HOVER_S}s")

    # Check yellow node
    if not yellow_connected:
        warn("yellow_border_node NOT connected! Run it first.")
        c = input("Continue without yellow detection? (y/n): ").strip().lower()
        if c != 'y': log("Aborted."); return

    # Takeoff mode
    while True:
        c = input("\nTakeoff mode:\n  1 - FLOWHOLD\n  2 - LOITER\nEnter: ").strip()
        if c=="1": arm_mode,wait_str="22","CMODE(22)"; break
        elif c=="2": arm_mode,wait_str="LOITER","LOITER"; break
        print("Invalid.")

    # Hover mode
    while True:
        c = input("\nHover mode:\n  1 - NEUTRAL (1500 in LOITER)\n  2 - FLOWHOLD\nEnter: ").strip()
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
    log(f"Pre-sequence hover for {HOVER_S}s...")
    if not do_hover(10, hover_mode): land_and_disarm(); return

    # ── MAIN LOOP ──
    for loop in range(1, N_LOOPS+1):
        log(f"═══ Loop {loop}/{N_LOOPS} ═══")

        if not _ok(): land_and_disarm(); return

        # 1. Roll RIGHT until yellow LEFT
        if not roll_until_yellow(ROLL_R_PWM, "right", ROLL_TIMEOUT, hover_mode):
            land_and_disarm(); return

        if not _ok(): land_and_disarm(); return

        # 2. Pitch FORWARD (timer, with yellow front check)
        if not pitch_timed(PITCH_F_PWM, T_PITCH, hover_mode, check_front_yellow=True):
            land_and_disarm(); return

        if not _ok(): land_and_disarm(); return

        # 3. Roll LEFT until yellow RIGHT
        if not roll_until_yellow(ROLL_L_PWM, "left", ROLL_TIMEOUT, hover_mode):
            land_and_disarm(); return

        if not _ok(): land_and_disarm(); return

        # Shift between loops (not after last)
        if loop < N_LOOPS:
            if not pitch_timed(SHIFT_PITCH_PWM, T_SHIFT, hover_mode, check_front_yellow=True):
                land_and_disarm(); return

    log("All loops complete.")

    if not _ok(): land_and_disarm(); return

    # ── YAW 180° ──
    if not yaw_180(hover_mode): land_and_disarm(); return

    if not _ok(): land_and_disarm(); return


    # ── RETURN: pitch until yellow front (aruco ignored) ──
    if not pitch_until_yellow_front(RETURN_PITCH_PWM, RETURN_TIMEOUT):
        land_and_disarm(); return

    # Hover after return pitch
    if not do_hover(YELLOW_STOP_HOVER_S, hover_mode):
        land_and_disarm(); return

    # ── FLOWHOLD then LAND ──
    log("Switching to FLOWHOLD before land...")
    set_mode("22"); sleep_check(2.0)

    land_and_disarm()
    log("="*50); log("ARENA NAV LOITER — COMPLETE"); log("="*50)

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
    rospy.init_node("arena_nav_loiter", anonymous=False)
    signal.signal(signal.SIGINT, sigint_handler)

    rospy.Subscriber(f"{MAVROS_NS}/state",                   State,       state_cb)
    rospy.Subscriber(f"{MAVROS_NS}/local_position/pose",     PoseStamped, local_pos_cb)
    rospy.Subscriber(f"{MAVROS_NS}/vision_pose/pose",        PoseStamped, vision_cb)
    rospy.Subscriber(f"{MAVROS_NS}/rangefinder/rangefinder", Range,       rf_cb)
    rospy.Subscriber("/yellow_border/status",                 String,      yellow_cb)
    rospy.Subscriber(ARUCO_STATUS_TOPIC,                       Bool,        aruco_status_cb)
    rospy.Subscriber(ARUCO_X_OFFSET_TOPIC,                     Float32,     aruco_x_cb)
    rospy.Subscriber(ARUCO_Y_OFFSET_TOPIC,                     Float32,     aruco_y_cb)

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

    # Wait briefly for yellow node
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

