#!/usr/bin/env python3
"""
guided_takeoff_forward.py  —  GUIDED takeoff + forward flight test
===================================================================
Mission:
  1) Takeoff to P1 meters
  2) Hover 5 seconds
  3) Fly forward P2 meters in GUIDED (North direction)
  4) Hover 5 seconds
  5) LAND

Two takeoff methods:
  Method 1 — Standard GUIDED takeoff (RC ramp in LOITER → switch GUIDED)
  Method 2 — "Motor spin trick" takeoff (no OpenVINS required):
              FLOWHOLD → spin motors to ~1550-1600 PWM while on ground →
              switch to LOITER (retry until confirmed) while motors spinning →
              continue RC ramp up to target altitude in LOITER →
              then switch to GUIDED for forward flight
              (hover alt is recorded at hover point and used for forward wp)

Safety: 'l' key → LAND, Ctrl+C → emergency LAND, VIO divergence → abort LAND.
"""

import rospy, threading, math, sys, time, signal, termios, tty, select
from mavros_msgs.msg import State, OverrideRCIn, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range

# ══════════════════════════════════════════════════════════════
#  PARAMETERS  (edit here)
# ══════════════════════════════════════════════════════════════

# Mission targets
P1_TAKEOFF_M   = 1.5    # takeoff altitude target (m above ground)
P2_FORWARD_M   = 2.0    # forward distance in GUIDED (m, North direction)

# Takeoff ramp (RC override phase)
TAKEOFF_PWM_START  = 1400
TAKEOFF_PWM_MAX    = 1850
TAKEOFF_PWM_STEP   = 10
TAKEOFF_TICK_S     = 0.3
GROUND_ALT_OFFSET  = 0.10   # rangefinder offset to detect liftoff (m)

# Method 2 — Motor spin trick
M2_SPIN_PWM        = 1575   # throttle PWM to spin motors on ground in FLOWHOLD
M2_SPIN_DURATION_S = 2.0    # how long to spin motors before switching to LOITER
M2_LOITER_RETRY_S  = 5.0    # timeout to confirm LOITER switch (retries until confirmed)

# Hovers
HOVER_AFTER_TAKEOFF_S = 5.0  # hover after reaching P1
HOVER_AFTER_FWD_S     = 5.0  # hover after reaching forward WP

# GUIDED waypoint navigation
WP_TOLERANCE   = 0.30   # reached radius (m)
WP_TIMEOUT_S   = 15.0   # warn + proceed if not reached in this time
WP_RATE_HZ     = 10

# Pre-GUIDED streaming (prevents ArduPilot mode switch rejection)
PRE_GUIDED_STREAM_S  = 3.0
POST_GUIDED_ANCHOR_S = 2.0

# Land
LAND_WAIT_S = 20.0

# VIO guard
VIO_XY_LIMIT_M   = 4.0
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

rc_pub = sp_pub = arm_srv = mode_srv = None

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
def log(m):  rospy.loginfo(f"[G-TF-FWD] {m}")
def warn(m): rospy.logwarn(f"[WARN]     {m}")
def err(m):  rospy.logerr(f"[ERROR]    {m}")

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

def force_mode(target, retry_timeout=5.0):
    """Keep calling set_mode until confirmed — used in M2 motor-spin trick."""
    t0 = time.time()
    while not rospy.is_shutdown() and not abort_flag.is_set():
        if state.mode == target:
            log(f"Mode confirmed: {target}"); return True
        set_mode(target)
        time.sleep(0.5)
        if time.time()-t0 > retry_timeout:
            warn(f"force_mode({target}) timed out after {retry_timeout}s")
            return False
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

def get_pos():
    """Returns (north, east, up) in local ENU frame."""
    p = local_pose.pose.position
    return p.y, p.x, p.z

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
#  TAKEOFF METHOD 1 — Standard LOITER ramp
# ══════════════════════════════════════════════════════════════
def takeoff_method1(ground):
    """Standard LOITER RC ramp takeoff to P1_TAKEOFF_M."""
    target_alt = ground + P1_TAKEOFF_M
    log(f"[M1] LOITER takeoff: ground={ground:.2f}m target={target_alt:.2f}m")

    if not set_mode("LOITER"): return False
    if not wait_mode("LOITER"): return False
    log("[M1] Arming...")
    if not arm(True): return False
    if not wait_armed(True): return False

    detect_alt = ground + GROUND_ALT_OFFSET
    has_liftoff = False; frozen_pwm = TAKEOFF_PWM_START

    for pwm in range(TAKEOFF_PWM_START, TAKEOFF_PWM_MAX+1, TAKEOFF_PWM_STEP):
        if not _ok(): return False
        curr = rf_range
        if not has_liftoff and curr > detect_alt:
            has_liftoff = True; frozen_pwm = 1500
            log(f"[M1] Liftoff at {curr:.2f}m")
        active = frozen_pwm if has_liftoff else pwm
        if curr >= target_alt:
            log("[M1] Target altitude reached!"); send_rc(throttle=1500); return True
        send_rc(throttle=active); time.sleep(TAKEOFF_TICK_S)

    warn("[M1] Ramp exhausted."); send_rc(throttle=1500); return True

# ══════════════════════════════════════════════════════════════
#  TAKEOFF METHOD 2 — Motor spin trick (no OpenVINS needed)
# ══════════════════════════════════════════════════════════════
def takeoff_method2(ground):
    """
    Motor-spin trick takeoff (tested manually):
      1. Switch to FLOWHOLD
      2. Arm
      3. Ramp throttle to M2_SPIN_PWM (~1575) until motors spin (drone still on ground)
      4. At that point, switch to LOITER (retry until confirmed)
      5. Continue normal RC ramp from current PWM to target altitude
    """
    target_alt = ground + P1_TAKEOFF_M
    log(f"[M2] Motor-spin trick takeoff: ground={ground:.2f}m target={target_alt:.2f}m")

    # Step 1: FLOWHOLD
    log("[M2] Switching to FLOWHOLD...")
    if not set_mode("22"): return False
    if not wait_mode("CMODE(22)", timeout=8): return False

    # Step 2: Arm
    log("[M2] Arming in FLOWHOLD...")
    if not arm(True): return False
    if not wait_armed(True): return False

    # Step 3: Ramp up to spin PWM while on ground
    log(f"[M2] Ramping throttle to M2_SPIN_PWM={M2_SPIN_PWM} (motors spinning on ground)...")
    for pwm in range(TAKEOFF_PWM_START, M2_SPIN_PWM+1, TAKEOFF_PWM_STEP):
        if not _ok(): return False
        send_rc(throttle=pwm)
        time.sleep(TAKEOFF_TICK_S)

    # Hold spin PWM for a moment
    log(f"[M2] Holding spin PWM={M2_SPIN_PWM} for {M2_SPIN_DURATION_S}s...")
    t0 = time.time()
    while time.time()-t0 < M2_SPIN_DURATION_S:
        if not _ok(): return False
        send_rc(throttle=M2_SPIN_PWM)
        time.sleep(0.1)

    # Step 4: Switch to LOITER with retry (motors keep spinning)
    log("[M2] Switching to LOITER (with retry) while motors are spinning...")
    if not force_mode("LOITER", retry_timeout=M2_LOITER_RETRY_S):
        abort_land("[M2] Could not confirm LOITER mode"); return False

    log("[M2] LOITER confirmed. Continuing RC ramp to target altitude...")

    # Step 5: Continue ramp from M2_SPIN_PWM to target alt in LOITER
    detect_alt = ground + GROUND_ALT_OFFSET
    has_liftoff = False; frozen_pwm = M2_SPIN_PWM
    current_pwm = M2_SPIN_PWM

    while _ok():
        curr = rf_range

        if not has_liftoff and curr > detect_alt:
            has_liftoff = True; frozen_pwm = 1500
            log(f"[M2] Liftoff at {curr:.2f}m")

        if curr >= target_alt:
            log("[M2] Target altitude reached!"); send_rc(throttle=1500); return True

        if has_liftoff:
            active = frozen_pwm
        else:
            active = min(current_pwm, TAKEOFF_PWM_MAX)
            current_pwm = min(current_pwm + TAKEOFF_PWM_STEP, TAKEOFF_PWM_MAX)

        send_rc(throttle=active)
        time.sleep(TAKEOFF_TICK_S)

    warn("[M2] Ramp aborted."); send_rc(throttle=1500); return False

# ══════════════════════════════════════════════════════════════
#  GUIDED HOVER (hold position)
# ══════════════════════════════════════════════════════════════
def guided_hover(duration_s, n, e, alt, yaw):
    """Stream a fixed setpoint for duration_s seconds to hold position in GUIDED."""
    log(f"GUIDED hover {duration_s}s at N={n:.2f} E={e:.2f} Alt={alt:.2f}m")
    rate = rospy.Rate(WP_RATE_HZ); t0 = time.time()
    while _ok() and time.time()-t0 < duration_s:
        publish_ned(n, e, alt, yaw); rate.sleep()
    return _ok()

# ══════════════════════════════════════════════════════════════
#  GUIDED GOTO WAYPOINT
# ══════════════════════════════════════════════════════════════
def goto_wp(target_n, target_e, alt, yaw, label):
    """Navigate to (target_n, target_e, alt) in GUIDED. Returns True when reached/timed-out."""
    log(f"GOTO {label}: N={target_n:.2f} E={target_e:.2f} Alt={alt:.2f}m")
    rate = rospy.Rate(WP_RATE_HZ); t0 = time.time()

    while _ok():
        publish_ned(target_n, target_e, alt, yaw)
        cn, ce, _ = get_pos()
        dist = math.sqrt((cn-target_n)**2 + (ce-target_e)**2)

        if dist < WP_TOLERANCE:
            log(f"Reached {label} (dist={dist:.2f}m)")
            return True

        if time.time()-t0 > WP_TIMEOUT_S:
            warn(f"Timeout on {label} after {WP_TIMEOUT_S}s. Proceeding anyway.")
            return True

        rate.sleep()

    return False

# ══════════════════════════════════════════════════════════════
#  MISSION
# ══════════════════════════════════════════════════════════════
def run_mission():
    log("="*55)
    log("GUIDED TAKEOFF + FORWARD FLIGHT TEST")
    log("="*55)
    log(f"P1 takeoff: {P1_TAKEOFF_M}m   P2 forward: {P2_FORWARD_M}m")
    log(f"WP tolerance: {WP_TOLERANCE}m   timeout: {WP_TIMEOUT_S}s")

    # Choose takeoff method
    while True:
        c = input(
            "\nTakeoff Method:\n"
            "  1 - Standard GUIDED (LOITER ramp → GUIDED)\n"
            "  2 - Motor-spin trick (FLOWHOLD spin → LOITER → GUIDED, no OpenVINS init needed)\n"
            "Enter: "
        ).strip()
        if c == "1": method = 1; break
        elif c == "2": method = 2; break
        print("Invalid.")

    threading.Thread(target=_kb_thread, daemon=True).start()
    start_vio_guard()

    # Calibrate ground level
    log("Calibrating ground level...")
    rospy.sleep(1.0)
    ground = rf_range
    if ground == 0.0:
        warn("Rangefinder 0.0 — waiting for valid data...")
        while not rospy.is_shutdown() and rf_range == 0.0: rospy.sleep(0.5)
        ground = rf_range
    log(f"Ground: {ground:.3f}m")

    # ── TAKEOFF ──
    if method == 1:
        ok = takeoff_method1(ground)
    else:
        ok = takeoff_method2(ground)

    if not ok:
        abort_land("Takeoff failed"); return

    start_rc_heartbeat()
    set_rc_hb(1500, 1500, 1500, 1500)

    # Ensure LOITER after takeoff ramp
    if state.mode != "LOITER":
        log("Switching to LOITER after takeoff...")
        if not set_mode("LOITER") or not wait_mode("LOITER"):
            abort_land("LOITER switch failed"); return

    # Small settle time
    rospy.sleep(0.5)

    # Record hover position (this is our GUIDED home + forward alt reference)
    home_n, home_e, home_z = get_pos()
    home_yaw = get_yaw()
    # Use actual local_pose z as the altitude for GUIDED setpoints
    hover_alt = home_z
    log(f"Hover position: N={home_n:.3f} E={home_e:.3f} Alt(z)={hover_alt:.3f}m  Yaw={math.degrees(home_yaw):.1f}°")

    # ── HOVER 5s after takeoff ──
    log(f"Hovering in LOITER for {HOVER_AFTER_TAKEOFF_S}s...")
    if not sleep_check(HOVER_AFTER_TAKEOFF_S): land_and_disarm(); return
    if not _ok(): land_and_disarm(); return

    # ── PRE-STREAM SETPOINTS before GUIDED switch ──
    log(f"Pre-streaming GUIDED setpoints for {PRE_GUIDED_STREAM_S}s...")
    rate = rospy.Rate(WP_RATE_HZ); t0 = time.time()
    while time.time()-t0 < PRE_GUIDED_STREAM_S and _ok():
        publish_ned(home_n, home_e, hover_alt, home_yaw); rate.sleep()

    # ── SWITCH TO GUIDED ──
    log("Switching to GUIDED...")
    if not set_mode("GUIDED") or not wait_mode("GUIDED"):
        abort_land("GUIDED switch failed"); return

    # Anchor at home in GUIDED
    log(f"Anchoring at hover position for {POST_GUIDED_ANCHOR_S}s...")
    t0 = time.time()
    while time.time()-t0 < POST_GUIDED_ANCHOR_S and _ok():
        publish_ned(home_n, home_e, hover_alt, home_yaw); rate.sleep()

    # Stop RC heartbeat — GUIDED uses setpoints
    clear_rc()
    if not _ok(): land_and_disarm(); return

    # ── FORWARD P2 meters ──
    fwd_n = home_n + P2_FORWARD_M   # forward = North direction
    fwd_e = home_e
    # Use the same hover_alt so altitude is consistent
    log(f"Flying forward {P2_FORWARD_M}m (N={fwd_n:.2f}, keeping Alt={hover_alt:.3f}m)...")
    if not goto_wp(fwd_n, fwd_e, hover_alt, home_yaw, f"Forward+{P2_FORWARD_M}m"):
        land_and_disarm(); return
    if not _ok(): land_and_disarm(); return

    # ── HOVER 5s at forward position ──
    log(f"Hovering at forward position for {HOVER_AFTER_FWD_S}s...")
    if not guided_hover(HOVER_AFTER_FWD_S, fwd_n, fwd_e, hover_alt, home_yaw):
        land_and_disarm(); return

    if not _ok(): land_and_disarm(); return

    # ── LAND ──
    log("Mission complete. Landing...")
    land_and_disarm()
    log("="*55)
    log("GUIDED TAKEOFF FORWARD — COMPLETE")
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
    global rc_pub, sp_pub, arm_srv, mode_srv
    rospy.init_node("guided_takeoff_forward", anonymous=False)
    signal.signal(signal.SIGINT, sigint_handler)

    rospy.Subscriber(f"{MAVROS_NS}/state",                   State,         state_cb)
    rospy.Subscriber(f"{MAVROS_NS}/local_position/pose",     PoseStamped,   local_pos_cb)
    rospy.Subscriber(f"{MAVROS_NS}/vision_pose/pose",        PoseStamped,   vision_cb)
    rospy.Subscriber(f"{MAVROS_NS}/rangefinder/rangefinder", Range,         rf_cb)

    rc_pub = rospy.Publisher(f"{MAVROS_NS}/rc/override",        OverrideRCIn,   queue_size=5)
    sp_pub = rospy.Publisher(f"{MAVROS_NS}/setpoint_raw/local", PositionTarget, queue_size=5)

    log("Waiting for MAVROS services...")
    rospy.wait_for_service(f"{MAVROS_NS}/cmd/arming")
    rospy.wait_for_service(f"{MAVROS_NS}/set_mode")
    arm_srv  = rospy.ServiceProxy(f"{MAVROS_NS}/cmd/arming", CommandBool)
    mode_srv = rospy.ServiceProxy(f"{MAVROS_NS}/set_mode",   SetMode)

    log("Waiting for MAVROS connection...")
    r = rospy.Rate(2)
    while not rospy.is_shutdown() and not state.connected: r.sleep()
    log("MAVROS connected.")

    rospy.sleep(1.0)

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
