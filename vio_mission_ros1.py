#!/usr/bin/env python3
"""
vio_mission_ros1.py  —  ROS1 Melodic / MAVROS  (ArduCopter)
=============================================================
Missions:
  1 – RC-override takeoff (FLOWHOLD or LOITER chosen at runtime)
      → switch LOITER → pitch-forward 5 s → neutral 10 s → LAND → disarm
  2 – GUIDED setpoint zigzag (mirrors dron_control_final1.py Mission 2)

Safety:
  • Press 'l' key  → immediate LAND
  • Ctrl+C         → immediate LAND + disarm
  • VIO divergence guard (Mission 1 only) – background thread
  • RC heartbeat (throttle=1500) runs all along until LAND

Run:  rosrun vio_bridge vio_mission_ros1.py   (or python3 vio_mission_ros1.py)
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
#  TUNABLE PARAMETERS  (edit here)
# ──────────────────────────────────────────────────────────────

# — General
MAVROS_NS          = "/mavros"          # MAVROS namespace
LOOP_RATE_HZ       = 10                 # main loop rate

# — Takeoff (Mission 1 & 2)
TAKEOFF_PWM_START  = 1400               # start PWM for ramp
TAKEOFF_PWM_MAX    = 1750               # max PWM ramp limit
TAKEOFF_PWM_STEP   = 10                 # PWM increment per tick
TAKEOFF_TICK_S     = 0.3               # seconds per tick
GROUND_ALT_OFFSET  = 0.10              # liftoff detect: ground + this (m)
TAKEOFF_TARGET_OFF = 0.35              # takeoff done: ground + this (m)

# — Mission 1
M1_TAKEOFF_MODE         = None              # set at runtime: "FLOWHOLD" or "LOITER"
M1_POST_TAKEOFF_HOVER_S = 3.0              # wait/hover after takeoff before pitch forward
M1_PITCH_FWD_PWM        = 1535              # pitch forward PWM
M1_PITCH_NEUTRAL        = 1500              # neutral PWM
M1_FWD_TIME_S           = 6.0               # forward pitch duration
M1_NEUTRAL_TIME_S       = 3.0              # neutral hold duration after forward pitch
M1_PITCH_BWD_PWM        = 1465              # pitch backward PWM
M1_BWD_TIME_S           = 6.0               # backward pitch duration
M1_POST_BWD_NEUTRAL_S   = 10.0              # neutral hold duration after backward pitch
M1_LAND_WAIT_S          = 20.0              # wait after LAND mode before disarm

# — Mission 2
M2_ALT             = 1.5              # guided takeoff altitude (m)
M2_FWD_M           = 2.0              # forward distance (m)
M2_SIDE_M          = 1.0              # side spacing (m)
M2_WP_TOLERANCE    = 0.25             # waypoint reached threshold (m)
M2_WP_TIMEOUT_S    = 10.0             # max seconds to reach each waypoint

# — VIO divergence (Mission 1)
VIO_XY_LIMIT_M     = 4.0              # abort if VIO x or y > this
VIO_CHECK_INTV_S   = 0.5              # divergence check interval

# ──────────────────────────────────────────────────────────────
#  GLOBALS (shared across threads)
# ──────────────────────────────────────────────────────────────
state        = State()
local_pose   = PoseStamped()
vision_pose  = PoseStamped()
rf_range     = 0.0
abort_flag   = threading.Event()       # set → everything stops, land
land_flag    = threading.Event()       # set by 'l' key press

rc_hb_active = threading.Event()      # RC heartbeat thread control
rc_hb_lock   = threading.Lock()
rc_hb_vals   = [1500, 1500, 1500, 1500]  # roll, pitch, throttle, yaw

rc_pub       = None
sp_pub       = None
arm_srv      = None
mode_srv     = None

# ──────────────────────────────────────────────────────────────
#  CALLBACKS
# ──────────────────────────────────────────────────────────────
def state_cb(msg):       global state;      state = msg
def local_pos_cb(msg):   global local_pose; local_pose = msg
def vision_cb(msg):      global vision_pose; vision_pose = msg
def rf_cb(msg):          global rf_range;   rf_range = msg.range

# ──────────────────────────────────────────────────────────────
#  LOW-LEVEL HELPERS
# ──────────────────────────────────────────────────────────────
def log(msg):  rospy.loginfo(f"[MISSION] {msg}")
def warn(msg): rospy.logwarn(f"[WARN]    {msg}")
def err(msg):  rospy.logerr(f"[ERROR]   {msg}")

def set_mode(mode):
    try:
        res = mode_srv(custom_mode=mode)
        ok = res.mode_sent
        log(f"Mode → {mode}" if ok else f"Mode {mode} FAILED")
        return ok
    except Exception as e:
        err(f"set_mode exception: {e}"); return False

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
    t0 = time.time()
    r  = rospy.Rate(10)
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
    """Sleep for secs seconds; returns False if abort/land triggered."""
    t0 = time.time(); r = rospy.Rate(10)
    while not rospy.is_shutdown() and not abort_flag.is_set() and not land_flag.is_set():
        if time.time() - t0 >= secs: return True
        r.sleep()
    return False

# ──────────────────────────────────────────────────────────────
#  RC OVERRIDE
# ──────────────────────────────────────────────────────────────
def send_rc(roll=1500, pitch=1500, throttle=1500, yaw=1500):
    msg = OverrideRCIn()
    msg.channels = [0]*18
    msg.channels[0] = roll
    msg.channels[1] = pitch
    msg.channels[2] = throttle
    msg.channels[3] = yaw
    rc_pub.publish(msg)

def clear_rc():
    """Release all RC overrides."""
    rc_hb_active.clear()
    msg = OverrideRCIn(); msg.channels = [0]*18
    rc_pub.publish(msg)

def set_rc_hb(roll=1500, pitch=1500, throttle=1500, yaw=1500):
    """Update values published by the RC heartbeat thread."""
    with rc_hb_lock:
        rc_hb_vals[:] = [roll, pitch, throttle, yaw]

def start_rc_heartbeat(throttle=1500):
    """Start background thread publishing RC override at 5 Hz."""
    set_rc_hb(throttle=throttle)
    if not rc_hb_active.is_set():
        rc_hb_active.set()
        threading.Thread(target=_rc_hb_thread, daemon=True).start()
    log("RC heartbeat started (throttle=1500).")

def _rc_hb_thread():
    while rc_hb_active.is_set() and not rospy.is_shutdown():
        with rc_hb_lock:
            r, p, t, y = rc_hb_vals
        send_rc(r, p, t, y)
        time.sleep(0.2)

# ──────────────────────────────────────────────────────────────
#  LAND (clears RC override, switches mode)
# ──────────────────────────────────────────────────────────────
def do_land():
    log("LAND mode initiated — clearing RC override.")
    clear_rc()
    time.sleep(0.1)
    set_mode("LAND")

def abort_land(reason=""):
    err(f"ABORT: {reason}")
    abort_flag.set()
    do_land()

# ──────────────────────────────────────────────────────────────
#  KEYBOARD MONITOR  ('l' → land)
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
        if _orig_tc:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, _orig_tc)
    except Exception: pass

def _kb_thread():
    _enable_raw_kb()
    while not abort_flag.is_set() and not rospy.is_shutdown():
        try:
            dr, _, _ = select.select([sys.stdin], [], [], 0.1)
            if dr:
                ch = sys.stdin.read(1)
                if ch.lower() == 'l':
                    log("'l' pressed → landing.")
                    land_flag.set()
        except Exception: break
    _restore_kb()

# ──────────────────────────────────────────────────────────────
#  VIO DIVERGENCE GUARD
# ──────────────────────────────────────────────────────────────
def _vio_guard_thread():
    """Abort mission if VIO XY exceeds limit."""
    r = rospy.Rate(1.0 / VIO_CHECK_INTV_S)
    while not abort_flag.is_set() and not land_flag.is_set() and not rospy.is_shutdown():
        vx = vision_pose.pose.position.x
        vy = vision_pose.pose.position.y
        if abs(vx) > VIO_XY_LIMIT_M or abs(vy) > VIO_XY_LIMIT_M:
            abort_land(f"VIO divergence! x={vx:.2f} y={vy:.2f}")
            break
        r.sleep()

def start_vio_guard():
    threading.Thread(target=_vio_guard_thread, daemon=True).start()
    log(f"VIO divergence guard started (limit ±{VIO_XY_LIMIT_M}m).")

# ──────────────────────────────────────────────────────────────
#  SETPOINT PUBLISH  (GUIDED / LOCAL NED)
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
    # ENU from /mavros/local_position/pose: x=East, y=North
    # FRAME_LOCAL_NED expects x=North, y=East for setpoints
    pt.position.x = north
    pt.position.y = east
    pt.position.z = alt_up   # positive UP
    pt.yaw = yaw_rad
    sp_pub.publish(pt)

def get_pos():
    """Return (north, east, alt) from ENU local_position/pose."""
    p = local_pose.pose.position
    return p.y, p.x, p.z   # ENU y=North, x=East, z=Up

def get_yaw():
    q = local_pose.pose.orientation
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))

# ──────────────────────────────────────────────────────────────
#  RC OVERRIDE TAKEOFF  (shared by M1 and M2)
# ──────────────────────────────────────────────────────────────
def rc_takeoff(arm_mode_str, wait_mode_str):
    """
    Arm in arm_mode_str, then ramp throttle PWM until liftoff.
    Returns True on success.
    """
    log(f"Step: Switch to {arm_mode_str}")
    if not set_mode(arm_mode_str): return False
    if not wait_mode(wait_mode_str): return False

    log("Step: Arming...")
    if not arm(True): return False
    if not wait_armed(True): return False

    log("Step: RC Override Takeoff ramp...")
    ground = rf_range
    detect_alt = ground + GROUND_ALT_OFFSET
    target_alt = ground + TAKEOFF_TARGET_OFF
    rospy.loginfo(f"[TAKEOFF] Ground={ground:.2f}m detect={detect_alt:.2f}m target={target_alt:.2f}m")

    has_liftoff = False
    frozen_pwm  = TAKEOFF_PWM_START

    for pwm in range(TAKEOFF_PWM_START, TAKEOFF_PWM_MAX + 1, TAKEOFF_PWM_STEP):
        if abort_flag.is_set() or land_flag.is_set() or rospy.is_shutdown():
            return False
        curr = rf_range

        if not has_liftoff and curr > detect_alt:
            has_liftoff = True
            frozen_pwm  = 1500
            log(f"[TAKEOFF] Liftoff! Freezing PWM at {frozen_pwm}")

        active = frozen_pwm if has_liftoff else pwm
        rospy.loginfo(f"[TAKEOFF] alt={curr:.2f}m loop={pwm} active={active}")

        if curr >= target_alt:
            log("[TAKEOFF] Target altitude reached!")
            send_rc(throttle=1500)
            return True

        if state.mode not in (wait_mode_str, arm_mode_str):
            warn("Mode changed during takeoff — aborting.")
            return False

        send_rc(throttle=active)
        time.sleep(TAKEOFF_TICK_S)

    warn("Takeoff ramp exhausted without reaching target altitude. Continuing anyway.")
    send_rc(throttle=1500)
    return True

# ──────────────────────────────────────────────────────────────
#  MISSION 1  – RC override sweep in LOITER
# ──────────────────────────────────────────────────────────────
def run_mission1():
    log("=" * 40)
    log("MISSION 1 START")
    log("=" * 40)

    # Choose takeoff mode at runtime
    while True:
        choice = input("\nTakeoff mode for Mission 1:\n  1 - FLOWHOLD\n  2 - LOITER\nEnter (1 or 2): ").strip()
        if choice == "1":
            arm_mode, wait_str = "22", "CMODE(22)"
            break
        elif choice == "2":
            arm_mode, wait_str = "LOITER", "LOITER"
            break
        print("Invalid. Try again.")

    # Start keyboard monitor and VIO guard
    threading.Thread(target=_kb_thread, daemon=True).start()
    start_vio_guard()

    # Takeoff
    if not rc_takeoff(arm_mode, wait_str):
        abort_land("Takeoff failed"); return

    # RC heartbeat at 1500 (all channels) — runs until do_land()
    start_rc_heartbeat(1500)

    # Explicitly publish neutral on all channels at start of hover
    # (clears any residual pitch from the takeoff ramp)
    set_rc_hb(1500, 1500, 1500, 1500)

    # Hover post-takeoff before pitch forward
    log(f"Hovering for {M1_POST_TAKEOFF_HOVER_S}s before pitch forward...")
    if not sleep_check(M1_POST_TAKEOFF_HOVER_S):
        do_land(); return

    # Switch to LOITER for pitch sweeps (if not already)
    if state.mode != "LOITER":
        log("Switching to LOITER for pitch sweep...")
        if not set_mode("LOITER") or not wait_mode("LOITER"):
            abort_land("LOITER switch failed"); return

    # Forward pitch 1550 for M1_FWD_TIME_S
    if land_flag.is_set() or abort_flag.is_set():
        do_land(); return
    log(f"Pitch forward ({M1_PITCH_FWD_PWM}) for {M1_FWD_TIME_S}s...")
    set_rc_hb(pitch=M1_PITCH_FWD_PWM, throttle=1500)
    if not sleep_check(M1_FWD_TIME_S):
        do_land(); return

    # Neutral all channels for M1_NEUTRAL_TIME_S
    log(f"Neutral hold for {M1_NEUTRAL_TIME_S}s...")
    set_rc_hb(1500, 1500, 1500, 1500)
    if not sleep_check(M1_NEUTRAL_TIME_S):
        do_land(); return

    # Pitch backward for M1_BWD_TIME_S
    log(f"Pitch backward ({M1_PITCH_BWD_PWM}) for {M1_BWD_TIME_S}s...")
    set_rc_hb(pitch=M1_PITCH_BWD_PWM, throttle=1500)
    if not sleep_check(M1_BWD_TIME_S):
        do_land(); return

    # Neutral all channels for M1_POST_BWD_NEUTRAL_S
    log(f"Neutral hold for {M1_POST_BWD_NEUTRAL_S}s...")
    set_rc_hb(1500, 1500, 1500, 1500)
    if not sleep_check(M1_POST_BWD_NEUTRAL_S):
        do_land(); return

    # Land
    log("Landing...")
    do_land()
    sleep_check(M1_LAND_WAIT_S)
    arm(False)

    log("=" * 40)
    log("MISSION 1 COMPLETE")
    log("=" * 40)

# ──────────────────────────────────────────────────────────────
#  MISSION 2  – GUIDED zigzag (mirrors dron_control_final1.py M2)
# ──────────────────────────────────────────────────────────────
def run_mission2():
    log("=" * 40)
    log("MISSION 2 START")
    log("=" * 40)

    threading.Thread(target=_kb_thread, daemon=True).start()
    start_vio_guard()

    # Choose takeoff mode at runtime
    while True:
        choice = input("\nTakeoff mode for Mission 2:\n  1 - FLOWHOLD\n  2 - LOITER\nEnter (1 or 2): ").strip()
        if choice == "1":
            arm_mode, wait_str = "22", "CMODE(22)"
            break
        elif choice == "2":
            arm_mode, wait_str = "LOITER", "LOITER"
            break
        print("Invalid. Try again.")

    # Takeoff via RC override
    if not rc_takeoff(arm_mode, wait_str):
        abort_land("Takeoff failed"); return

    start_rc_heartbeat(1500)

    # Hover post-takeoff before GUIDED transition
    log(f"Hovering for {M1_POST_TAKEOFF_HOVER_S}s before GUIDED switch...")
    if not sleep_check(M1_POST_TAKEOFF_HOVER_S):
        do_land(); return

    # Capture home and current yaw
    rospy.sleep(1.0)
    home_n, home_e, home_alt = get_pos()
    home_alt = M2_ALT   # use param altitude
    home_yaw = get_yaw()
    rospy.loginfo(f"[M2] Home: N={home_n:.3f} E={home_e:.3f} Alt={home_alt:.2f} Yaw={math.degrees(home_yaw):.1f}°")

    # Pre-stream setpoints before switching to GUIDED
    log("Pre-streaming setpoints 3s before GUIDED...")
    t0 = time.time(); r = rospy.Rate(10)
    while time.time() - t0 < 3.0 and not abort_flag.is_set() and not land_flag.is_set():
        publish_ned(home_n, home_e, home_alt, home_yaw); r.sleep()

    # Switch to GUIDED
    log("Switching to GUIDED...")
    if not set_mode("GUIDED") or not wait_mode("GUIDED"):
        abort_land("GUIDED switch failed"); return

    # Anchor at home in GUIDED
    log("Anchoring home in GUIDED 2s...")
    t0 = time.time()
    while time.time() - t0 < 2.0 and not abort_flag.is_set() and not land_flag.is_set():
        publish_ned(home_n, home_e, home_alt, home_yaw); r.sleep()

    # Build waypoints (zigzag: fwd → side → back)
    wps = [
        (home_n + M2_FWD_M, home_e,          home_alt, "Forward"),
        (home_n + M2_FWD_M, home_e + M2_SIDE_M, home_alt, "Right"),
        (home_n,             home_e + M2_SIDE_M, home_alt, "Back"),
    ]

    def goto_wp(n, e, alt, label):
        log(f"Going to {label}: N={n:.2f} E={e:.2f}")
        t0 = time.time()
        while not abort_flag.is_set() and not land_flag.is_set() and not rospy.is_shutdown():
            publish_ned(n, e, alt, home_yaw)
            cn, ce, ca = get_pos()
            dist = math.sqrt((cn-n)**2 + (ce-e)**2)
            if dist < M2_WP_TOLERANCE:
                log(f"Reached {label} (dist={dist:.2f}m)"); return True
            if time.time() - t0 > M2_WP_TIMEOUT_S:
                warn(f"Timeout reaching {label}"); return False
            r.sleep()
        return False

    for wn, we, wa, wlabel in wps:
        if not goto_wp(wn, we, wa, wlabel):
            abort_land(f"Failed at {wlabel}"); return

    # Switch back to FLOWHOLD then land
    log("Switching to FLOWHOLD before land...")
    set_mode("22")
    sleep_check(5.0)
    do_land()
    sleep_check(M1_LAND_WAIT_S)
    arm(False)

    log("=" * 40)
    log("MISSION 2 COMPLETE")
    log("=" * 40)

# ──────────────────────────────────────────────────────────────
#  SIGINT HANDLER
# ──────────────────────────────────────────────────────────────
def sigint_handler(sig, frame):
    warn("Ctrl+C → Emergency land & disarm!")
    abort_flag.set()
    do_land()
    time.sleep(3.0)
    arm(False)
    _restore_kb()
    sys.exit(0)

# ──────────────────────────────────────────────────────────────
#  MAIN
# ──────────────────────────────────────────────────────────────
def main():
    global rc_pub, sp_pub, arm_srv, mode_srv

    rospy.init_node("vio_mission", anonymous=False)
    signal.signal(signal.SIGINT, sigint_handler)

    # Subscribers
    rospy.Subscriber(f"{MAVROS_NS}/state",                   State,        state_cb)
    rospy.Subscriber(f"{MAVROS_NS}/local_position/pose",     PoseStamped,  local_pos_cb)
    rospy.Subscriber(f"{MAVROS_NS}/vision_pose/pose",        PoseStamped,  vision_cb)
    rospy.Subscriber(f"{MAVROS_NS}/rangefinder/rangefinder", Range,        rf_cb)

    # Publishers
    rc_pub = rospy.Publisher(f"{MAVROS_NS}/rc/override",          OverrideRCIn,   queue_size=5)
    sp_pub = rospy.Publisher(f"{MAVROS_NS}/setpoint_raw/local",   PositionTarget, queue_size=5)

    # Services
    log("Waiting for MAVROS services...")
    rospy.wait_for_service(f"{MAVROS_NS}/cmd/arming")
    rospy.wait_for_service(f"{MAVROS_NS}/set_mode")
    arm_srv  = rospy.ServiceProxy(f"{MAVROS_NS}/cmd/arming", CommandBool)
    mode_srv = rospy.ServiceProxy(f"{MAVROS_NS}/set_mode",   SetMode)

    log("Waiting for MAVROS heartbeat...")
    r = rospy.Rate(2)
    while not rospy.is_shutdown() and not state.connected:
        r.sleep()
    log("MAVROS connected.")

    rospy.sleep(1.0)  # let topics settle

    # Mission selection
    mission = 0
    while mission not in (1, 2) and not rospy.is_shutdown():
        try:
            mission = int(input("\nSelect mission:\n  1 - RC Takeoff + LOITER pitch sweep\n  2 - GUIDED zigzag\nEnter (1 or 2): ").strip())
        except ValueError:
            pass

    if mission == 1:
        run_mission1()
    elif mission == 2:
        run_mission2()

    _restore_kb()
    rospy.signal_shutdown("Mission complete.")

if __name__ == "__main__":
    main()
