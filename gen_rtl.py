"""Generate arena_nav_loiter_rtl.py from arena_nav_loiter.py."""
import re

with open(r"d:\open-vins\elimination_round\arena_nav_loiter.py", "r", encoding="utf-8") as f:
    src = f.read()

# 1. Update docstring header
src = src.replace(
    "arena_nav_loiter.py  —  Loiter-based arena sweep with yellow border detection",
    "arena_nav_loiter_rtl.py  —  Loiter-based arena sweep with RTL return"
)
src = src.replace(
    "Mission: RC takeoff → LOITER sweep → yellow-border-aware turns → yaw 180 → return → land",
    "Mission: RC takeoff → LOITER sweep → yellow-border-aware turns → yaw 180 → RTL → land"
)
src = src.replace(
    "After loops:\n  Yaw 180° → Pitch forward until yellow FRONT → hover → FLOWHOLD → LAND (20s)",
    "After loops:\n  Yaw 180° → hover → RTL (auto-return+land) → wait 50s → LAND (safety)"
)

# 2. Replace RETURN params with RTL_WAIT_S
src = src.replace(
    "# Return pitch (after yaw 180, until yellow front detected)\n"
    "RETURN_PITCH_PWM  = 1470\n"
    "RETURN_TIMEOUT    = 30.0   # max seconds for return pitch\n"
    "RETURN_EXTRA_PITCH_S = 0.5  # extra pitch (s) after yellow front first detected before stopping",
    "# RTL return timeout\n"
    "RTL_WAIT_S = 50.0          # seconds to wait for RTL to complete before forcing LAND"
)

# 3. Replace end of run_mission: everything after yaw_180 done
OLD_END = """    if not _ok(): land_and_disarm(); return


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
    log("="*50); log("ARENA NAV LOITER — COMPLETE"); log("="*50)"""

NEW_END = """    if not _ok(): land_and_disarm(); return

    # ── RTL — drone returns and lands automatically ──
    log("Activating RTL — drone will return to base and land.")
    set_mode("RTL")

    log(f"Waiting {RTL_WAIT_S}s for RTL to complete...")
    sleep_check(RTL_WAIT_S)

    # Safety: explicitly command LAND + disarm after timeout
    log("RTL wait complete. Commanding LAND + disarm (safety).")
    land_and_disarm()
    log("="*50); log("ARENA NAV LOITER RTL — COMPLETE"); log("="*50)"""

src = src.replace(OLD_END, NEW_END)

# 4. Change ROS node name
src = src.replace(
    'rospy.init_node("arena_nav_loiter"',
    'rospy.init_node("arena_nav_loiter_rtl"'
)

# 5. Update log prefix
src = src.replace('"[ARENA-L] {m}"', '"[ARENA-RTL] {m}"')

out_path = r"d:\open-vins\elimination_round\arena_nav_loiter_rtl.py"
with open(out_path, "w", encoding="utf-8") as f:
    f.write(src)

print(f"Written: {out_path}")
print(f"Lines: {src.count(chr(10))}")

# Sanity check
assert "RTL_WAIT_S" in src, "RTL_WAIT_S missing!"
assert "arena_nav_loiter_rtl" in src, "Node name not updated!"
assert "pitch_until_yellow_front" not in src.split("def run_mission")[1], \
    "Old return pitch still in run_mission!"
print("Sanity checks passed.")
