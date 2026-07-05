#!/usr/bin/env python3
"""
takeoff_land_test.py  —  RC takeoff to 1.0m → FLOWHOLD 10s → LOITER step climb to 1.75m → hover 10s → LOITER step descend to 1.0m → hover 10s → LAND
"""

import rospy, threading, sys, time, signal, termios, tty, select
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range

# ══════════════════════════════════════════════════════════════
#  PARAMETERS
# ══════════════════════════════════════════════════════════════
TAKEOFF_PWM_START  = 1400
TAKEOFF_PWM_MAX    = 1750
TAKEOFF_PWM_STEP   = 10
TAKEOFF_TICK_S     = 0.3
GROUND_ALT_OFFSET  = 0.10

TAKEOFF_TARGET_OFF = 1.0    # takeoff target height (m)
CLIMB_TARGET_OFF   = 1.75   # climb target height (m)
DESCEND_TARGET_OFF = 1.0    # descend target height (m)

CLIMB_PWM_STEP     = 1      # PWM step increment
CLIMB_PWM_MAX      = 1800   # maximum throttle safety limit
DESCEND_PWM_STEP   = 1      # PWM step decrement
DESCEND_PWM_MIN    = 1300   # minimum throttle safety limit

HOVER_DURATION_S   = 10.0   # hover duration (s)
LOOP_RATE_HZ       = 10     # control loop rate (Hz)

LAND_WAIT_S        = 20.0
VIO_XY_LIMIT_M     = 4.0
VIO_CHECK_INTV_S   = 0.5
MAVROS_NS          = "/mavros"

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

rc_pub = arm_srv = mode_srv = None

# ══════════════════════════════════════════════════════════════
#  CALLBACKS
# ══════════════════════════════════════════════════════════════
def state_cb(msg):      global state;       state = msg
def local_pos_cb(msg):  global local_pose;  local_pose = msg
def vision_cb(msg):     global vision_pose; vision_pose = msg
def rf_cb(msg):         global rf_range;    rf_range = msg.range

# ══════════════════════════════════════════════════════════════
#  LOGGING
# ══════════════════════════════════════════════════════════════
def log(m):  rospy.loginfo(f"[TEST] {m}")
def warn(m): rospy.logwarn(f"[WARN] {m}")
def err(m):  rospy.logerr(f"[ERROR] {m}")

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
#  KEYBOARD INTERCEPT
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
#  MISSION EXECUTION
# ══════════════════════════════════════════════════════════════
def run_test():
    log("="*50); log("TAKEOFF LAND FLIGHT TEST"); log("="*50)
    
    # Wait for rangefinder data to settle
    log("Calibrating ground level...")
    rospy.sleep(1.0)
    ground = rf_range
    if ground == 0.0:
        warn("Rangefinder reading is 0.0! Waiting for valid rangefinder data...")
        while not rospy.is_shutdown() and rf_range == 0.0:
            rospy.sleep(0.5)
        ground = rf_range
    log(f"Ground level calibrated: {ground:.2f}m")

    threading.Thread(target=_kb_thread, daemon=True).start()
    start_vio_guard()

    # ── 1) TAKEOFF TO 1.0M ──
    log("Starting LOITER takeoff to 1.0m...")
    if not rc_takeoff("LOITER", "LOITER"):
        abort_land("Takeoff failed"); return

    start_rc_heartbeat()
    set_rc_hb(1500, 1500, 1500, 1500)

    # Ensure LOITER
    if state.mode != "LOITER":
        set_mode("LOITER"); wait_mode("LOITER", timeout=5)

    # ── 2) FLOWHOLD HOVER FOR 10 SEC ──
    log("Switching to FLOWHOLD (mode 22) for 10s hover...")
    if not set_mode("22") or not wait_mode("CMODE(22)", timeout=5):
        abort_land("FLOWHOLD switch failed"); return
    
    log("Hovering in FLOWHOLD...")
    if not sleep_check(10.0): land_and_disarm(); return

    # ── 3) SWITCH TO LOITER & CLIMB TO 1.75M ──
    log("Switching to LOITER for step climb...")
    if not set_mode("LOITER") or not wait_mode("LOITER", timeout=5):
        abort_land("LOITER switch failed"); return

    target_climb_alt = ground + CLIMB_TARGET_OFF
    log(f"Climbing to 1.75m (absolute target: {target_climb_alt:.2f}m)...")
    
    curr_throttle = 1500
    r = rospy.Rate(LOOP_RATE_HZ)
    while _ok():
        curr_alt = rf_range
        if curr_alt >= target_climb_alt:
            log(f"Reached climb altitude target: {curr_alt:.2f}m")
            break
        
        # Increment throttle in small steps
        if curr_throttle < CLIMB_PWM_MAX:
            curr_throttle += CLIMB_PWM_STEP
            
        set_rc_hb(throttle=curr_throttle)
        r.sleep()

    # Stay in LOITER neutral for 10 seconds
    log(f"Setting throttle to neutral. Hovering in LOITER for {HOVER_DURATION_S}s...")
    set_rc_hb(throttle=1500)
    if not sleep_check(HOVER_DURATION_S): land_and_disarm(); return

    # ── 4) LANDING SEQUENCE: REDUCE THROTTLE TO 1.0M ──
    target_descend_alt = ground + DESCEND_TARGET_OFF
    log(f"Descending back to 1.0m (absolute target: {target_descend_alt:.2f}m)...")
    
    curr_throttle = 1500
    while _ok():
        curr_alt = rf_range
        if curr_alt <= target_descend_alt:
            log(f"Reached descent altitude target: {curr_alt:.2f}m")
            break
        
        # Decrement throttle in small steps
        if curr_throttle > DESCEND_PWM_MIN:
            curr_throttle -= DESCEND_PWM_STEP
            
        set_rc_hb(throttle=curr_throttle)
        r.sleep()

    # Hover at 1.0m for 10 seconds
    log(f"Setting throttle to neutral. Hovering in LOITER for {HOVER_DURATION_S}s...")
    set_rc_hb(throttle=1500)
    if not sleep_check(HOVER_DURATION_S): land_and_disarm(); return

    # ── 5) LAND MODE CALLING ──
    log("Calling land mode...")
    land_and_disarm()
    log("="*50); log("TAKEOFF LAND FLIGHT TEST COMPLETE"); log("="*50)

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
    rospy.init_node("takeoff_land_test", anonymous=False)
    signal.signal(signal.SIGINT, sigint_handler)

    rospy.Subscriber(f"{MAVROS_NS}/state",                   State,       state_cb)
    rospy.Subscriber(f"{MAVROS_NS}/local_position/pose",     PoseStamped, local_pos_cb)
    rospy.Subscriber(f"{MAVROS_NS}/vision_pose/pose",        PoseStamped, vision_cb)
    rospy.Subscriber(f"{MAVROS_NS}/rangefinder/rangefinder", Range,       rf_cb)

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

    try:
        run_test()
    except Exception as e:
        err(f"Unhandled exception: {e}")
        abort_flag.set(); do_land(); time.sleep(3.0); arm(False)
    finally:
        _restore_kb()
        rospy.signal_shutdown("Done.")

if __name__ == "__main__":
    main()
