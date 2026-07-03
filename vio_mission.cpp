/*
 * vio_mission.cpp
 *
 * ROS1 (Melodic) C++ mission controller using MAVROS.
 *
 * Missions
 * --------
 * Mission 1:
 *   - Switch to FLOWHOLD (mode 22) -> Arm
 *   - Takeoff with RC Throttle Override ramping using /mavros/rangefinder/rangefinder
 *   - Once target alt >= 0.4m, hover (1550) -> release RC override -> stay 10 s
 *   - Switch to LOITER -> forward 10 s -> hold 10 s -> back 10 s -> hold 5 s
 *   - Switch to FLOWHOLD for 5 s -> LAND
 *
 * Mission 2:
 *   - Switch to FLOWHOLD (mode 22) -> Arm
 *   - Switch to GUIDED
 *   - Takeoff to 1.5 meters (RC Override) -> wait M2_POST_TAKEOFF_HOVER_S
 *   - Guided LOCAL_NED absolute waypoints: +Fwd -> +Right -> -Back -> -Left
 *     (each motion sends ONE setpoint then waits M2_MOVE_WAIT_S for arrival)
 *     (pause between moves sends zero-offset hold setpoints for M2_GAP_S)
 *     (yaw is locked to home heading throughout)
 *   - Switch to FLOWHOLD for 10 s -> LAND
 *
 * Mission 3:
 *   - Switch to GUIDED -> Arm
 *   - MAV_CMD_NAV_TAKEOFF to 1.5 m (no RC override ramp)
 *   - Pre-stream LOCAL_NED setpoints -> wait for FC to accept GUIDED setpoints
 *   - Hover at home position for 5 s
 *   - Fly 2 m North in LOCAL_NED frame (streaming setpoints for M3_MOVE_WAIT_S)
 *   - Hold north waypoint for 5 s
 *   - Switch FLOWHOLD -> LAND -> Disarm
 *
 * Safety features:
 *   - Background vision pose divergence monitor
 *   - Background drift guard monitor (for LOITER/GUIDED phases)
 *   - Non-blocking keyboard handler: Pressing 'l' at any time triggers landing
 *   - Custom SIGINT handler: Ctrl+C triggers safe recovery (FLOWHOLD -> LAND -> Disarm)
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/OverrideRCIn.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <signal.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

// ─────────────────────────────────────────────────────────
//  TUNABLE PARAMETERS
// ─────────────────────────────────────────────────────────
namespace Params {
    constexpr double ABORT_XY_M          = 5.0;  // vision divergence limit
    constexpr double DRIFT_THRESH_M      = 2.0;  // drift limit
    constexpr double DRIFT_WINDOW_S      = 3.0;  // drift time window

    // Mission 1 parameters (UNCHANGED)
    constexpr double M1_HOVER_S          = 10.0;
    constexpr double M1_FWD_S            = 5.0;
    constexpr double M1_FWD_HOLD_S       = 10.0;
    constexpr double M1_BACK_S           = 5.0;
    constexpr double M1_BACK_HOLD_S      = 10.0;
    constexpr double M1_FH_BEFORE_LAND_S = 5.0;
    constexpr uint16_t M1_FORWARD_PWM    = 1450; // Pitch forward (e.g. 1450)
    constexpr uint16_t M1_BACKWARD_PWM   = 1550; // Pitch backward (e.g. 1550)
    constexpr uint16_t M1_YAW_PWM        = 1500; // Yaw (default 1500)

    // Mission 2 parameters
    constexpr double M2_TAKEOFF_ALT_M        = 1.5;   // target hover altitude (m)
    constexpr double M2_POST_TAKEOFF_HOVER_S = 20.0;  // hover time in FLOWHOLD after takeoff

    constexpr double M2_FWD_M            = 2.0;   // forward distance (m)
    constexpr double M2_RIGHT_M          = 2.0;   // right distance (m)
    constexpr double M2_BACK_M           = 2.0;   // backward distance (m)
    constexpr double M2_LEFT_M           = 2.0;   // left distance (m)

    constexpr double M2_MOVE_WAIT_S      = 8.0;   // seconds to wait for drone to reach waypoint
    constexpr double M2_GAP_S            = 5.0;   // seconds to hold position between legs

    constexpr double M2_FH_AFTER_LAND_S  = 10.0;  // FLOWHOLD hold before LAND

    // ── how long to pre-stream setpoints before GUIDED switch ──
    constexpr double M2_GUIDED_PRESTREAM_S = 2.0;  // seconds to stream before mode switch
    // ── how long to anchor position immediately after GUIDED switch ──
    constexpr double M2_GUIDED_ANCHOR_S    = 2.0;  // seconds to stream after mode switch

    // ─────────────────────────────────────────────────────
    //  Mission 3 parameters
    // ─────────────────────────────────────────────────────
    constexpr double M3_TAKEOFF_ALT_M        = 1.5;   // GUIDED takeoff altitude (m)

    // How long to pre-stream setpoints before sending the GUIDED takeoff command.
    // ArduPilot requires at least a few setpoints already in the pipe before it
    // will honour GUIDED setpoint commands post-takeoff.
    constexpr double M3_GUIDED_PRESTREAM_S   = 2.0;

    // After the takeoff command is accepted, keep streaming the home setpoint
    // while waiting for the drone to reach target altitude.
    constexpr double M3_TAKEOFF_TIMEOUT_S    = 15.0; // seconds to wait for altitude

    // Altitude threshold (m) below target to consider takeoff complete.
    constexpr double M3_TAKEOFF_ALT_THRESH_M = 0.15;

    // How long to hover at home after takeoff before moving north.
    constexpr double M3_POST_TAKEOFF_HOVER_S = 5.0;

    // North movement
    constexpr double M3_NORTH_M          = 1.0;   // distance to fly north (m)
    constexpr double M3_MOVE_WAIT_S      = 8.0;   // seconds to stream north waypoint

    // How long to hold the north waypoint before landing.
    constexpr double M3_NORTH_HOLD_S     = 5.0;

    // FLOWHOLD settle time before LAND command.
    constexpr double M3_FH_BEFORE_LAND_S = 5.0;

    // Safety / Emergency
    constexpr double ABORT_FH_HOLD_S     = 5.0;
    constexpr double DISARM_WAIT_S       = 10.0;
}

// ─────────────────────────────────────────────────────────
//  GLOBAL SIGNAL STATE
// ─────────────────────────────────────────────────────────
std::atomic<bool> g_sigint_triggered(false);
struct termios g_orig_termios;
bool g_termios_changed = false;

static void print(const std::string& s)  { std::cout << "[MISSION] " << s << std::endl; }
static void warn (const std::string& s)  { std::cout << "[WARN]    " << s << std::endl; }
static void err  (const std::string& s)  { std::cerr << "[ERROR]   " << s << std::endl; }

static double now_s() { return ros::Time::now().toSec(); }

void resetTerminal() {
    if (g_termios_changed) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
        g_termios_changed = false;
    }
}

void initTerminal() {
    if (tcgetattr(STDIN_FILENO, &g_orig_termios) >= 0) {
        struct termios raw = g_orig_termios;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 1;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) >= 0) {
            g_termios_changed = true;
            std::atexit(resetTerminal);
        }
    }
}

bool kbhit() {
    struct timeval tv = {0, 0};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
}

void sigintHandler(int sig) {
    g_sigint_triggered = true;
}

// ─────────────────────────────────────────────────────────
//  VioMission Controller Class
// ─────────────────────────────────────────────────────────
class VioMission {
public:
    explicit VioMission(ros::NodeHandle& nh) : nh_(nh) {
        state_sub_ = nh_.subscribe("/mavros/state", 5, &VioMission::stateCb, this);
        vp_sub_    = nh_.subscribe("/mavros/vision_pose/pose", 5, &VioMission::visionPoseCb, this);
        lp_sub_    = nh_.subscribe("/mavros/local_position/pose", 5, &VioMission::localPosCb, this);
        rf_sub_    = nh_.subscribe("/mavros/rangefinder/rangefinder", 5, &VioMission::rangefinderCb, this);

        sp_pub_    = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        rc_pub_    = nh_.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

        arm_cl_    = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        mode_cl_   = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        cmd_cl_    = nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
        takeoff_cl_= nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    }

    void run(int mission, int m1_takeoff_mode = 1) {
        print("Waiting for MAVROS heartbeat...");
        waitConnected();

        abort_flag_   = false;
        abort_reason_ = "";

        std::thread kb_thread(&VioMission::keyboardMonitor, this);

        if      (mission == 1) runMission1(m1_takeoff_mode);
        else if (mission == 2) runMission2();
        else                   runMission3();

        if (kb_thread.joinable()) kb_thread.join();
    }

    bool isArmed() { return getState().armed; }

private:
    ros::NodeHandle&   nh_;
    ros::Subscriber    state_sub_, vp_sub_, lp_sub_, rf_sub_;
    ros::Publisher     sp_pub_, rc_pub_;
    ros::ServiceClient arm_cl_, mode_cl_, cmd_cl_, takeoff_cl_;

    mavros_msgs::State  state_;
    std::mutex          state_mtx_;

    double vp_x_{0}, vp_y_{0}, vp_z_{0};
    std::mutex vp_mtx_;
    bool vp_valid_{false};

    double lp_x_{0}, lp_y_{0}, lp_z_{0};
    double lp_qx_{0}, lp_qy_{0}, lp_qz_{0}, lp_qw_{1};
    std::mutex lp_mtx_;

    double rf_dist_{0.0};
    std::mutex rf_mtx_;

    std::atomic<bool>   abort_flag_{false};
    std::string         abort_reason_;

    std::atomic<bool>   rc_hb_active_{false};
    std::thread         rc_hb_thread_;
    uint16_t            rc_hb_roll_{1500};
    uint16_t            rc_hb_pitch_{1500};
    uint16_t            rc_hb_throttle_{1500};
    uint16_t            rc_hb_yaw_{1500};
    std::mutex          rc_hb_mtx_;

    double m2_home_yaw_{0.0};

    // Callbacks
    void stateCb(const mavros_msgs::State::ConstPtr& m) {
        std::lock_guard<std::mutex> g(state_mtx_); state_ = *m;
    }
    void visionPoseCb(const geometry_msgs::PoseStamped::ConstPtr& m) {
        std::lock_guard<std::mutex> g(vp_mtx_);
        vp_x_ = m->pose.position.x;
        vp_y_ = m->pose.position.y;
        vp_z_ = m->pose.position.z;
        vp_valid_ = true;
    }
    void localPosCb(const geometry_msgs::PoseStamped::ConstPtr& m) {
        std::lock_guard<std::mutex> g(lp_mtx_);
        lp_x_ = m->pose.position.x;
        lp_y_ = m->pose.position.y;
        lp_z_ = m->pose.position.z;
        lp_qx_ = m->pose.orientation.x;
        lp_qy_ = m->pose.orientation.y;
        lp_qz_ = m->pose.orientation.z;
        lp_qw_ = m->pose.orientation.w;
    }
    void rangefinderCb(const sensor_msgs::Range::ConstPtr& m) {
        std::lock_guard<std::mutex> g(rf_mtx_); rf_dist_ = m->range;
    }

    mavros_msgs::State getState() {
        std::lock_guard<std::mutex> g(state_mtx_); return state_;
    }
    void getVisionPose(double& x, double& y, double& z) {
        std::lock_guard<std::mutex> g(vp_mtx_); x = vp_x_; y = vp_y_; z = vp_z_;
    }
    void getLocalPos(double& north, double& east) {
        std::lock_guard<std::mutex> g(lp_mtx_);
        // /mavros/local_position/pose is ENU: x=East, y=North
        north = lp_y_;  // ENU y = North
        east  = lp_x_;  // ENU x = East
    }
    double getLocalAlt() {
        std::lock_guard<std::mutex> g(lp_mtx_); return lp_z_;
    }
    // Extract yaw (ENU frame) from the local position quaternion
    double getCurrentYaw() {
        double qx, qy, qz, qw;
        { std::lock_guard<std::mutex> g(lp_mtx_); qx = lp_qx_; qy = lp_qy_; qz = lp_qz_; qw = lp_qw_; }
        // yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        return std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
    }
    double getRangefinder() {
        std::lock_guard<std::mutex> g(rf_mtx_); return rf_dist_;
    }

    void waitConnected() {
        ros::Rate r(2);
        while (ros::ok() && !getState().connected && !g_sigint_triggered) {
            ros::spinOnce(); r.sleep();
        }
        if (getState().connected) print("MAVROS connected.");
    }

    bool setMode(const std::string& m) {
        mavros_msgs::SetMode srv;
        srv.request.custom_mode = m;
        if (!mode_cl_.call(srv) || !srv.response.mode_sent) {
            err("SetMode(" + m + ") failed."); return false;
        }
        print("Mode → " + m); return true;
    }

    bool arm(bool do_arm) {
        mavros_msgs::CommandBool srv;
        srv.request.value = do_arm;
        if (!arm_cl_.call(srv) || !srv.response.success) {
            err(do_arm ? "Arm failed." : "Disarm failed."); return false;
        }
        print(do_arm ? "Armed." : "Disarmed."); return true;
    }

    void land() {
        print("Switching to LAND mode.");
        clearRcOverride();
        setMode("LAND");
    }

    bool waitMode(const std::string& m, double timeout_s = 0) {
        double t0 = now_s(); ros::Rate r(10);
        while (ros::ok() && !abort_flag_ && !g_sigint_triggered) {
            ros::spinOnce();
            if (getState().mode == m) return true;
            if (timeout_s > 0 && (now_s() - t0) > timeout_s) return false;
            r.sleep();
        }
        return false;
    }

    bool waitArmed(bool armed_state, double timeout_s = 8.0) {
        double t0 = now_s(); ros::Rate r(10);
        while (ros::ok() && !abort_flag_ && !g_sigint_triggered) {
            ros::spinOnce();
            if (getState().armed == armed_state) return true;
            if ((now_s() - t0) > timeout_s) return false;
            r.sleep();
        }
        return false;
    }

    bool sleepCheck(double secs) {
        double t0 = now_s(); ros::Rate r(10);
        while (ros::ok() && !abort_flag_ && !g_sigint_triggered) {
            ros::spinOnce();
            if ((now_s() - t0) >= secs) return true;
            r.sleep();
        }
        return false;
    }

    bool visionPoseOk() {
        double x, y, z; getVisionPose(x, y, z);
        if (std::abs(x) > Params::ABORT_XY_M || std::abs(y) > Params::ABORT_XY_M) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "Vision pose diverged! x=%.2f y=%.2f (limit ±%.1f m)", x, y, Params::ABORT_XY_M);
            abort_reason_ = buf; warn(abort_reason_); return false;
        }
        return true;
    }

    void keyboardMonitor() {
        while (ros::ok() && !abort_flag_ && !g_sigint_triggered) {
            if (kbhit()) {
                char c = std::cin.get();
                if (c == 'l' || c == 'L') {
                    warn("Safety landing key 'l' pressed! Aborting mission.");
                    abort_reason_ = "safety land key pressed";
                    abort_flag_ = true; break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    struct DriftGuard {
        VioMission*       vm;
        std::atomic<bool> commanding{false};
        std::thread       th;

        void start(VioMission* v) {
            vm = v; commanding = false;
            th = std::thread([this]() {
                double sx, sy; vm->getLocalPos(sx, sy);
                double snap_t = now_s(); ros::Rate r(5);
                while (ros::ok() && !vm->abort_flag_ && !g_sigint_triggered) {
                    ros::spinOnce();
                    if (!commanding) {
                        double cx, cy; vm->getLocalPos(cx, cy);
                        double dt = now_s() - snap_t;
                        double dx = std::abs(cx - sx), dy = std::abs(cy - sy);
                        if (dt >= Params::DRIFT_WINDOW_S) {
                            if (dx > Params::DRIFT_THRESH_M || dy > Params::DRIFT_THRESH_M) {
                                char buf[128];
                                std::snprintf(buf, sizeof(buf),
                                    "Drift guard triggered! Δx=%.2f Δy=%.2f over %.1fs", dx, dy, dt);
                                vm->abort_reason_ = buf; ::warn(buf);
                                vm->abort_flag_ = true; break;
                            }
                            sx = cx; sy = cy; snap_t = now_s();
                        }
                    } else { vm->getLocalPos(sx, sy); snap_t = now_s(); }
                    r.sleep();
                }
            });
        }
        void stop() { if (th.joinable()) th.join(); }
    };

    void sendRcOverride(uint16_t roll, uint16_t pitch, uint16_t throttle, uint16_t yaw) {
        mavros_msgs::OverrideRCIn msg;
        msg.channels[0] = roll;
        msg.channels[1] = pitch;
        msg.channels[2] = throttle;
        msg.channels[3] = yaw;
        for (int i = 4; i < 18; ++i) msg.channels[i] = 0;
        rc_pub_.publish(msg);
    }

    void sendRcOverride(uint16_t throttle) {
        sendRcOverride(1500, 1500, throttle, 1500);
    }

    void clearRcOverride() {
        stopRcHeartbeat();
        mavros_msgs::OverrideRCIn msg;
        for (int i = 0; i < 18; ++i) msg.channels[i] = 0;
        rc_pub_.publish(msg);
    }

    void startRcHeartbeat(uint16_t throttle = 1500) {
        {
            std::lock_guard<std::mutex> g(rc_hb_mtx_);
            rc_hb_roll_ = 1500;
            rc_hb_pitch_ = 1500;
            rc_hb_throttle_ = throttle;
            rc_hb_yaw_ = 1500;
        }
        if (!rc_hb_active_.exchange(true)) {
            rc_hb_thread_ = std::thread([this]() {
                while (rc_hb_active_.load() && ros::ok()) {
                    uint16_t r, p, t, y;
                    {
                        std::lock_guard<std::mutex> g(rc_hb_mtx_);
                        r = rc_hb_roll_;
                        p = rc_hb_pitch_;
                        t = rc_hb_throttle_;
                        y = rc_hb_yaw_;
                    }
                    sendRcOverride(r, p, t, y);
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }
            });
        }
    }

    void setRcHeartbeat(uint16_t roll, uint16_t pitch, uint16_t throttle, uint16_t yaw) {
        std::lock_guard<std::mutex> g(rc_hb_mtx_);
        rc_hb_roll_ = roll;
        rc_hb_pitch_ = pitch;
        rc_hb_throttle_ = throttle;
        rc_hb_yaw_ = yaw;
    }

    void stopRcHeartbeat() {
        rc_hb_active_ = false;
        if (rc_hb_thread_.joinable()) rc_hb_thread_.join();
    }

    // ── Mission 1 only: body-frame velocity publish ──
    void publishBodyVel(double vx, double vy) {
        mavros_msgs::PositionTarget pt;
        pt.header.stamp      = ros::Time::now();
        pt.coordinate_frame  = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        pt.type_mask =
            mavros_msgs::PositionTarget::IGNORE_PX  | mavros_msgs::PositionTarget::IGNORE_PY  |
            mavros_msgs::PositionTarget::IGNORE_PZ  |
            mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        pt.velocity.x = vx; pt.velocity.y = vy; pt.velocity.z = 0;
        sp_pub_.publish(pt);
    }

    // ── Mission 1 only: stream body-frame velocity for given duration ──
    bool streamVel(double vx, double vy, double secs, DriftGuard& dg) {
        dg.commanding = true;
        double t0 = now_s(); ros::Rate r(10);
        while (ros::ok() && !abort_flag_ && !g_sigint_triggered) {
            ros::spinOnce();
            if ((now_s() - t0) >= secs) break;
            publishBodyVel(vx, vy);
            r.sleep();
        }
        publishBodyVel(0, 0);
        dg.commanding = false;
        return !abort_flag_.load() && !g_sigint_triggered;
    }

    // ─────────────────────────────────────────────────────
    //  Shared M2/M3 helpers: LOCAL_NED setpoint publishing
    // ─────────────────────────────────────────────────────

    /*
     * publishLocalNED — sends ONE LOCAL_NED absolute position setpoint.
     * Yaw is locked to m2_home_yaw_ to prevent rotation during flight.
     */
    void publishLocalNED(double n, double e, double alt) {
        mavros_msgs::PositionTarget pt;
        pt.header.stamp     = ros::Time::now();
        pt.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        pt.type_mask =
            mavros_msgs::PositionTarget::IGNORE_VX  | mavros_msgs::PositionTarget::IGNORE_VY  |
            mavros_msgs::PositionTarget::IGNORE_VZ  |
            mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        pt.position.x = n;
        pt.position.y = e;
        pt.position.z = alt;    // Z positive = UP in MAVROS setpoint frame

        pt.yaw = static_cast<float>(m2_home_yaw_);

        sp_pub_.publish(pt);
    }

    /*
     * holdLocalNED — streams position hold setpoints at 10 Hz for given duration.
     */
    bool holdLocalNED(double n, double e, double alt, double secs, DriftGuard& dg) {
        dg.commanding = false;  // drift guard active during hold
        double t0 = now_s();
        ros::Rate r(10);
        print("Holding position at N=" + std::to_string(n).substr(0,5) +
              " E=" + std::to_string(e).substr(0,5) +
              " for " + std::to_string((int)secs) + "s...");
        while (ros::ok() && !abort_flag_ && !g_sigint_triggered) {
            ros::spinOnce();
            if ((now_s() - t0) >= secs) break;
            publishLocalNED(n, e, alt);
            r.sleep();
        }
        return !abort_flag_.load() && !g_sigint_triggered;
    }

    /*
     * flowHold — switches to FLOWHOLD for secs, then switches back to GUIDED.
     * Used for position hold phases in Mission 3 where FLOWHOLD is more reliable.
     */
    bool flowHold(double secs, DriftGuard& dg) {
        dg.commanding = false;
        print("FLOWHOLD hold for " + std::to_string((int)secs) + "s...");
        if (!setMode("22")) {
            warn("FLOWHOLD switch failed during hold; staying in GUIDED.");
        }
        if (!sleepCheck(secs)) return false;

        print("Returning to GUIDED...");
        if (!setMode("GUIDED")) {
            warn("GUIDED switch failed after FLOWHOLD hold!");
            return false;
        }
        if (!waitMode("GUIDED", 8)) {
            warn("GUIDED not confirmed after FLOWHOLD hold!");
            return false;
        }
        // Re-stream setpoints for a moment to re-anchor position
        ros::Rate r(10);
        double t0 = now_s();
        while (ros::ok() && !abort_flag_ && !g_sigint_triggered) {
            ros::spinOnce();
            if ((now_s() - t0) >= 1.0) break;
            r.sleep();
        }
        return !abort_flag_.load() && !g_sigint_triggered;
    }

    /*
     * gotoLocalNED — streams the waypoint setpoint at 10 Hz for move_wait_s seconds.
     * Continuous streaming keeps the FC target fresh throughout the move window.
     */
    bool gotoLocalNED(const std::string& label, double n, double e, double alt,
                      double move_wait_s, DriftGuard& dg) {
        dg.commanding = true;
        print("Waypoint [" + label + "] → N=" + std::to_string(n).substr(0,6) +
              " E=" + std::to_string(e).substr(0,6) +
              " Alt=" + std::to_string(alt).substr(0,4) + "m");

        double t0 = now_s();
        ros::Rate r(10);
        while (ros::ok() && !abort_flag_ && !g_sigint_triggered) {
            ros::spinOnce();
            if ((now_s() - t0) >= move_wait_s) break;
            publishLocalNED(n, e, alt);
            r.sleep();
        }

        dg.commanding = false;
        return !abort_flag_.load() && !g_sigint_triggered;
    }

    void doAbortLand(const std::string& reason) {
        err("ABORT — " + reason);
        clearRcOverride();
        print("Switching to FLOWHOLD (mode 22) for recovery...");
        setMode("22");
        ros::Duration(Params::ABORT_FH_HOLD_S).sleep();
        land();
        ros::Duration(Params::DISARM_WAIT_S).sleep();
        arm(false);
    }

    // ─────────────────────────────────────────────────────
    //  MISSION 1 — UNCHANGED
    // ─────────────────────────────────────────────────────
    void runMission1(int m1_takeoff_mode) {
        print("========================================");
        print("MISSION 1 START");
        print("========================================");

        std::string mode_str = (m1_takeoff_mode == 2) ? "LOITER" : "22";
        std::string wait_mode_str = (m1_takeoff_mode == 2) ? "LOITER" : "CMODE(22)";

        ros::spinOnce();
        print("Waiting for valid rangefinder reading...");
        double start_check = now_s();
        while (ros::ok() && getRangefinder() <= 0.0 && !g_sigint_triggered) {
            ros::spinOnce(); ros::Duration(0.1).sleep();
            if (now_s() - start_check > 5.0) {
                warn("No rangefinder data received yet. Proceeding with 0.0m start.");
                break;
            }
        }

        print("Step 1: Switch to " + mode_str + " pre-arm");
        if (!setMode(mode_str))              { doAbortLand("SetMode " + mode_str + " failed");     return; }
        if (!waitMode(wait_mode_str, 8))   { doAbortLand(mode_str + " not confirmed"); return; }

        print("Step 2: Arming in " + mode_str);
        if (!arm(true))                  { doAbortLand("Arm failed");            return; }
        if (!waitArmed(true))            { doAbortLand("Arm timed out");         return; }

        print("Step 3: Taking off via RC Throttle Override (" + mode_str + ")...");
        bool takeoff_success = false;
        bool has_taken_off   = false;
        int  takeoff_pwm     = 0;

        ros::spinOnce();
        double ground_alt         = getRangefinder();
        double takeoff_detect_alt = ground_alt + 0.10;
        double target_alt         = ground_alt + 0.35;
        std::printf("[TAKEOFF] Ground: %.2fm | Detect: %.2fm | Target: %.2fm\n",
                    ground_alt, takeoff_detect_alt, target_alt);

        for (int pwm = 1400; pwm <= 1850; pwm += 10) {
            if (abort_flag_ || g_sigint_triggered) break;
            ros::spinOnce();
            double curr_alt = getRangefinder();

            if (!has_taken_off && curr_alt > takeoff_detect_alt) {
                has_taken_off = true; takeoff_pwm = pwm;
                print("[TAKEOFF] Liftoff detected! Freezing PWM at " + std::to_string(takeoff_pwm));
            }

            int active_pwm = has_taken_off ? takeoff_pwm : pwm;
            std::printf("[TAKEOFF] Alt: %.2fm, Loop PWM: %d, Active PWM: %d\n",
                        curr_alt, pwm, active_pwm);

            if (curr_alt >= target_alt) {
                print("[TAKEOFF] Target altitude reached!");
                setMode(mode_str); sendRcOverride(1500);
                takeoff_success = true; break;
            }
            if (getState().mode != wait_mode_str) {
                warn("Flight mode changed during takeoff! Aborting."); break;
            }
            sendRcOverride(active_pwm);
            ros::Duration(0.3).sleep();
        }

        if (!takeoff_success) {
            doAbortLand(abort_reason_.empty() ? "Takeoff failed" : abort_reason_); return;
        }

        print("Step 4: Starting RC override heartbeat at 1500");
        startRcHeartbeat(1500);

        print("Step 5: " + mode_str + " hover for " + std::to_string((int)Params::M1_HOVER_S) + "s");
        if (!sleepCheck(Params::M1_HOVER_S)) { doAbortLand(abort_reason_); return; }

        print("Step 6: Vision pose check...");
        if (!visionPoseOk()) { doAbortLand(abort_reason_); return; }

        print("Step 7: Switching to LOITER");
        if (getState().mode != "LOITER") {
            if (!setMode("LOITER"))           { doAbortLand("SetMode LOITER failed"); return; }
            if (!waitMode("LOITER", 8))       { doAbortLand("LOITER not confirmed");  return; }
        } else {
            print("Already in LOITER.");
        }

        DriftGuard dg; dg.start(this);

        print("Step 8: LOITER - Forward (Pitch Override=" + std::to_string(Params::M1_FORWARD_PWM) + ") for " + std::to_string((int)Params::M1_FWD_S) + "s");
        dg.commanding = true;
        setRcHeartbeat(1500, Params::M1_FORWARD_PWM, 1500, Params::M1_YAW_PWM);
        if (!sleepCheck(Params::M1_FWD_S)) {
            dg.stop(); doAbortLand(abort_reason_); return;
        }

        print("Step 9: LOITER - Hold for " + std::to_string((int)Params::M1_FWD_HOLD_S) + "s");
        dg.commanding = false;
        setRcHeartbeat(1500, 1500, 1500, 1500);
        if (!sleepCheck(Params::M1_FWD_HOLD_S)) {
            dg.stop(); doAbortLand(abort_reason_); return;
        }

        print("Step 10: LOITER - Backward (Pitch Override=" + std::to_string(Params::M1_BACKWARD_PWM) + ") for " + std::to_string((int)Params::M1_BACK_S) + "s");
        dg.commanding = true;
        setRcHeartbeat(1500, Params::M1_BACKWARD_PWM, 1500, Params::M1_YAW_PWM);
        if (!sleepCheck(Params::M1_BACK_S)) {
            dg.stop(); doAbortLand(abort_reason_); return;
        }

        print("Step 11: LOITER - Hold for " + std::to_string((int)Params::M1_BACK_HOLD_S) + "s");
        dg.commanding = false;
        setRcHeartbeat(1500, 1500, 1500, 1500);
        if (!sleepCheck(Params::M1_BACK_HOLD_S)) {
            dg.stop(); doAbortLand(abort_reason_); return;
        }

        dg.stop();

        print("Step 12: Switch to FLOWHOLD (mode 22) before LAND for " + std::to_string((int)Params::M1_FH_BEFORE_LAND_S) + "s");
        setMode("22");
        setRcHeartbeat(1500, 1500, 1500, 1500);
        sleepCheck(Params::M1_FH_BEFORE_LAND_S);

        print("Step 13: LAND");
        land();
        sleepCheck(Params::DISARM_WAIT_S);
        arm(false);

        print("========================================");
        print("MISSION 1 COMPLETE");
        print("========================================");
    }

    // ─────────────────────────────────────────────────────
    //  MISSION 2 — UNCHANGED
    // ─────────────────────────────────────────────────────
    void runMission2() {
        print("========================================");
        print("MISSION 2 START");
        print("========================================");

        ros::spinOnce();
        print("Waiting for valid rangefinder reading...");
        double start_check = now_s();
        while (ros::ok() && getRangefinder() <= 0.0 && !g_sigint_triggered) {
            ros::spinOnce(); ros::Duration(0.1).sleep();
            if (now_s() - start_check > 5.0) {
                warn("No rangefinder data received yet. Proceeding with 0.0m start.");
                break;
            }
        }

        print("Step 1: Switch to FLOWHOLD (mode 22) pre-arm");
        if (!setMode("22"))             { doAbortLand("SetMode 22 failed");      return; }
        if (!waitMode("CMODE(22)", 8))  { doAbortLand("FLOWHOLD not confirmed"); return; }

        print("Step 2: Arming in FLOWHOLD");
        if (!arm(true))                 { doAbortLand("Arm failed");             return; }
        if (!waitArmed(true))           { doAbortLand("Arm timed out");          return; }

        print("Step 3: Taking off via RC Throttle Override...");
        bool takeoff_success = false;
        bool has_taken_off   = false;
        int  takeoff_pwm     = 0;

        ros::spinOnce();
        double ground_alt         = getRangefinder();
        double takeoff_detect_alt = ground_alt + 0.10;
        double target_alt         = ground_alt + 0.20;
        std::printf("[TAKEOFF] Ground reading: %.2fm | Detect at: %.2fm | Target: %.2fm\n",
                    ground_alt, takeoff_detect_alt, target_alt);

        for (int pwm = 1400; pwm <= 1750; pwm += 10) {
            if (abort_flag_ || g_sigint_triggered) break;

            ros::spinOnce();
            double curr_alt = getRangefinder();

            if (!has_taken_off && curr_alt > takeoff_detect_alt) {
                has_taken_off = true;
                takeoff_pwm   = pwm;
                print("[TAKEOFF] Takeoff detected! Freezing throttle PWM at " + std::to_string(takeoff_pwm));
            }

            int active_pwm = has_taken_off ? takeoff_pwm : pwm;
            std::printf("[TAKEOFF] Altitude: %.2fm, Loop PWM: %d, Active PWM: %d\n",
                        curr_alt, pwm, active_pwm);

            if (curr_alt >= target_alt) {
                print("[TAKEOFF] Target altitude reached!");
                setMode("22");
                sendRcOverride(1500);
                takeoff_success = true;
                break;
            }

            if (getState().mode != "CMODE(22)") {
                warn("Flight mode changed by pilot or autopilot during takeoff! Aborting.");
                break;
            }

            sendRcOverride(active_pwm);
            ros::Duration(0.3).sleep();
        }

        print("Step 4: Starting RC override heartbeat at 1500");
        startRcHeartbeat(1500);

        print("Step 5: Staying in FLOWHOLD hover for 10s");
        if (!sleepCheck(10.0)) { doAbortLand(abort_reason_); return; }

        print("Step 6: Checking vision pose divergence...");
        if (!visionPoseOk()) { doAbortLand(abort_reason_); return; }

        print("Step 7: Capturing home position...");
        ros::spinOnce();
        double home_n, home_e;
        getLocalPos(home_n, home_e);
        double home_alt = Params::M2_TAKEOFF_ALT_M;

        ros::spinOnce();
        m2_home_yaw_ = getCurrentYaw();
        std::printf("[M2] Home: N=%.2f E=%.2f Alt=%.2f | Yaw lock: %.2f rad (%.1f deg)\n",
                    home_n, home_e, home_alt, m2_home_yaw_, m2_home_yaw_ * 180.0 / M_PI);

        print("Step 7b: Pre-streaming LOCAL_NED setpoints for " +
              std::to_string(Params::M2_GUIDED_PRESTREAM_S) + "s before GUIDED switch...");
        {
            double t0 = now_s();
            ros::Rate r(10);
            while (ros::ok() && !abort_flag_ && !g_sigint_triggered) {
                ros::spinOnce();
                if ((now_s() - t0) >= Params::M2_GUIDED_PRESTREAM_S) break;
                publishLocalNED(home_n, home_e, home_alt);
                r.sleep();
            }
        }
        if (abort_flag_ || g_sigint_triggered) { doAbortLand(abort_reason_); return; }

        print("Step 7c: Switching to GUIDED...");
        if (!setMode("GUIDED"))      { doAbortLand("SetMode GUIDED failed"); return; }
        if (!waitMode("GUIDED", 8))  { doAbortLand("GUIDED not confirmed");  return; }

        print("Step 7d: Anchoring position post-GUIDED switch for " +
              std::to_string(Params::M2_GUIDED_ANCHOR_S) + "s...");
        {
            double t0 = now_s();
            ros::Rate r(10);
            while (ros::ok() && !abort_flag_ && !g_sigint_triggered) {
                ros::spinOnce();
                if ((now_s() - t0) >= Params::M2_GUIDED_ANCHOR_S) break;
                publishLocalNED(home_n, home_e, home_alt);
                r.sleep();
            }
        }
        if (abort_flag_ || g_sigint_triggered) { doAbortLand(abort_reason_); return; }

        double wp_fwd_n   = home_n + Params::M2_FWD_M;
        double wp_fwd_e   = home_e;
        double wp_right_n = wp_fwd_n;
        double wp_right_e = home_e + Params::M2_RIGHT_M;
        double wp_back_n  = home_n;
        double wp_back_e  = wp_right_e;
        double wp_home_n  = home_n;
        double wp_home_e  = home_e;
        double alt = home_alt;

        DriftGuard dg; dg.start(this);

        if (!gotoLocalNED("Forward", wp_fwd_n, wp_fwd_e, alt, Params::M2_MOVE_WAIT_S, dg))
            goto abort2;
        if (!holdLocalNED(wp_fwd_n, wp_fwd_e, alt, Params::M2_GAP_S, dg))
            goto abort2;

        if (!gotoLocalNED("Right", wp_right_n, wp_right_e, alt, Params::M2_MOVE_WAIT_S, dg))
            goto abort2;
        if (!holdLocalNED(wp_right_n, wp_right_e, alt, Params::M2_GAP_S, dg))
            goto abort2;

        if (!gotoLocalNED("Back", wp_back_n, wp_back_e, alt, Params::M2_MOVE_WAIT_S, dg))
            goto abort2;
        if (!holdLocalNED(wp_back_n, wp_back_e, alt, Params::M2_GAP_S, dg))
            goto abort2;

        if (!gotoLocalNED("Home", wp_home_n, wp_home_e, alt, Params::M2_MOVE_WAIT_S, dg))
            goto abort2;
        if (!holdLocalNED(wp_home_n, wp_home_e, alt, Params::M2_GAP_S, dg))
            goto abort2;

        dg.stop();

        print("Step 10: Switch to FLOWHOLD for " +
              std::to_string((int)Params::M2_FH_AFTER_LAND_S) + "s");
        setMode("22");
        sleepCheck(Params::M2_FH_AFTER_LAND_S);

        print("Step 11: LAND");
        land();
        sleepCheck(Params::DISARM_WAIT_S);
        arm(false);

        print("========================================");
        print("MISSION 2 COMPLETE");
        print("========================================");
        return;

    abort2:
        dg.stop();
        doAbortLand(abort_reason_);
    }

    // ─────────────────────────────────────────────────────
    //  MISSION 3
    //  Full GUIDED mission — no RC ramp takeoff.
    //
    //  Sequence:
    //    1. Switch to GUIDED
    //    2. Arm in GUIDED
    //    3. Pre-stream LOCAL_NED setpoints at home position (ArduPilot requirement)
    //    4. Send MAV_CMD_NAV_TAKEOFF to M3_TAKEOFF_ALT_M
    //    5. Keep streaming home setpoint while waiting for altitude
    //    6. Hover at home for M3_POST_TAKEOFF_HOVER_S
    //    7. Vision pose check
    //    8. Fly 2 m North (stream for M3_MOVE_WAIT_S)
    //    9. Hold north waypoint for M3_NORTH_HOLD_S
    //   10. Switch FLOWHOLD → wait M3_FH_BEFORE_LAND_S → LAND → Disarm
    // ─────────────────────────────────────────────────────
    void runMission3() {
        print("========================================");
        print("MISSION 3 START");
        print("========================================");

        // ── Step 1: Switch directly to GUIDED ────────────────────────
        //
        // ArduCopter requires the mode to be GUIDED before arming when using
        // the GUIDED takeoff command. No FLOWHOLD step needed here.
        print("Step 1: Switch to GUIDED mode");
        if (!setMode("GUIDED"))     { doAbortLand("SetMode GUIDED failed"); return; }
        if (!waitMode("GUIDED", 8)) { doAbortLand("GUIDED not confirmed");  return; }

        // ── Step 2: Arm in GUIDED ─────────────────────────────────────
        print("Step 2: Arming in GUIDED");
        if (!arm(true))          { doAbortLand("Arm failed");    return; }
        if (!waitArmed(true))    { doAbortLand("Arm timed out"); return; }

        // ── Step 3: Capture home position, lock yaw ──────────────────
        //
        // Capture immediately after arming so the local frame origin is set.
        // Yaw is locked to 0 (north) throughout the mission.
        ros::spinOnce();
        double home_n, home_e;
        getLocalPos(home_n, home_e);
        double home_alt = Params::M3_TAKEOFF_ALT_M;
        ros::spinOnce();
        m2_home_yaw_ = getCurrentYaw();  // lock to current yaw at takeoff

        std::printf("[M3] Home: N=%.2f E=%.2f | Target alt: %.2fm | Yaw lock: %.2f rad (%.1f deg)\n",
                    home_n, home_e, home_alt, m2_home_yaw_, m2_home_yaw_ * 180.0 / M_PI);

        // ── Step 4: Pre-stream LOCAL_NED setpoints ────────────────────
        //
        // ArduPilot in GUIDED will not accept setpoint commands unless the
        // setpoint publisher is already streaming.  We stream the home
        // position at ground level (alt = 0) so the FC knows our intent
        // before the takeoff command arrives.
        // ── Step 5: Send MAV_CMD_NAV_TAKEOFF via CommandLong ─────────
        print("Step 5: Sending MAV_CMD_NAV_TAKEOFF to " +
              std::to_string(Params::M3_TAKEOFF_ALT_M) + "m...");
        {
            mavros_msgs::CommandLong srv;
            srv.request.command = 22; // MAV_CMD_NAV_TAKEOFF
            srv.request.param7 = Params::M3_TAKEOFF_ALT_M;
            if (!cmd_cl_.call(srv) || !srv.response.success) {
                doAbortLand("MAV_CMD_NAV_TAKEOFF failed"); return;
            }
            print("Takeoff command accepted.");
        }

        // ── Step 6: Wait for climb (no setpoint streaming) ────────────
        print("Step 6: Waiting " + std::to_string((int)Params::M3_TAKEOFF_TIMEOUT_S) +
              "s for drone to reach altitude...");
        if (!sleepCheck(Params::M3_TAKEOFF_TIMEOUT_S)) {
            doAbortLand(abort_reason_); return;
        }
        // ── Step 7: Vision pose check ─────────────────────────────────
        print("Step 7: Vision pose divergence check...");
        if (!visionPoseOk()) { doAbortLand(abort_reason_); return; }

        // ── Start RC override heartbeat at 1500 throttle ─────────────
        // Keeps throttle mid-stick for FLOWHOLD throughout the mission.
        // Stopped automatically when land() calls clearRcOverride().
        print("Starting RC override heartbeat (throttle=1500)...");
        startRcHeartbeat(1500);

        // ── Step 8: Fly 1m North in LOCAL_NED (GUIDED) ───────────────
        DriftGuard dg; dg.start(this);
        {
            double wp_north_n = home_n + Params::M3_NORTH_M;
            double wp_north_e = home_e;

            print("Step 9: Flying " + std::to_string(Params::M3_NORTH_M) +
                  "m North to N=" + std::to_string(wp_north_n).substr(0,6) + "...");

            if (!gotoLocalNED("North", wp_north_n, wp_north_e, home_alt,
                              Params::M3_MOVE_WAIT_S, dg))
                goto abort3;

            // ── Step 10: FLOWHOLD hold at north waypoint ─────────────
            print("Step 10: FLOWHOLD hold at north waypoint for " +
                  std::to_string((int)Params::M3_NORTH_HOLD_S) + "s...");
            if (!flowHold(Params::M3_NORTH_HOLD_S, dg))
                goto abort3;
        }

        dg.stop();

        // ── Step 11: FLOWHOLD → LAND ──────────────────────────────────
        //
        // Per the brief: last land is FLOWHOLD first (from GUIDED), then LAND.
        print("Step 11: Switching to FLOWHOLD (mode 22) for " +
              std::to_string(Params::M3_FH_BEFORE_LAND_S) + "s settle...");
        if (!setMode("22")) {
            // Non-fatal: proceed to land even if mode switch fails
            warn("FLOWHOLD switch failed; proceeding directly to LAND.");
        }
        sleepCheck(Params::M3_FH_BEFORE_LAND_S);

        print("Step 12: LAND");
        land();
        sleepCheck(Params::DISARM_WAIT_S);
        arm(false);

        print("========================================");
        print("MISSION 3 COMPLETE");
        print("========================================");
        return;

    abort3:
        dg.stop();
        doAbortLand(abort_reason_);
    }
};

// ─────────────────────────────────────────────────────────
//  MAIN
// ─────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    ros::init(argc, argv, "vio_mission",
              ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
    ros::NodeHandle nh;

    signal(SIGINT, sigintHandler);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    int mission = 0;
    int m1_takeoff_mode = 1;
    while (mission != 1 && mission != 2 && mission != 3 && !g_sigint_triggered) {
        std::cout << "\nSelect mission:\n"
                  << "  1 - FLOWHOLD/LOITER Takeoff with RC override & LOITER pitch sweep\n"
                  << "  2 - GUIDED Takeoff & LOCAL_NED square (continuous setpoint streaming)\n"
                  << "  3 - Full GUIDED: MAV_CMD_NAV_TAKEOFF 1.5m, 2m North, FLOWHOLD->LAND\n"
                  << "Enter (1, 2, or 3): ";
        struct timeval tv = {0, 500000};
        fd_set fds; FD_ZERO(&fds); FD_SET(STDIN_FILENO, &fds);
        int ret = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
        if (g_sigint_triggered) break;
        if (ret > 0) {
            std::string line; std::getline(std::cin, line);
            try { mission = std::stoi(line); } catch (...) {}
            if (mission != 1 && mission != 2 && mission != 3)
                std::cout << "Invalid input. Try again.\n";
        }
    }

    if (mission == 1 && !g_sigint_triggered) {
        m1_takeoff_mode = 0;
        while (m1_takeoff_mode != 1 && m1_takeoff_mode != 2 && !g_sigint_triggered) {
            std::cout << "\nSelect takeoff mode for Mission 1:\n"
                      << "  1 - FLOWHOLD Takeoff\n"
                      << "  2 - LOITER Takeoff\n"
                      << "Enter (1 or 2): ";
            struct timeval tv = {0, 500000};
            fd_set fds; FD_ZERO(&fds); FD_SET(STDIN_FILENO, &fds);
            int ret = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
            if (g_sigint_triggered) break;
            if (ret > 0) {
                std::string line; std::getline(std::cin, line);
                try { m1_takeoff_mode = std::stoi(line); } catch (...) {}
                if (m1_takeoff_mode != 1 && m1_takeoff_mode != 2)
                    std::cout << "Invalid input. Try again.\n";
            }
        }
    }

    VioMission vm(nh);
    if (!g_sigint_triggered) {
        initTerminal();
        vm.run(mission, m1_takeoff_mode);
        resetTerminal();
    }

    if (g_sigint_triggered) {
        resetTerminal();
        if (vm.isArmed()) {
            warn("[SIGINT] Ctrl+C detected while armed! Emergency landing...");
            ros::ServiceClient mode_cl = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
            ros::ServiceClient arm_cl  = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

            mavros_msgs::SetMode mode_srv;
            mode_srv.request.custom_mode = "22";
            mode_cl.call(mode_srv);
            ros::Duration(2.0).sleep();

            mode_srv.request.custom_mode = "LAND";
            mode_cl.call(mode_srv);
            ros::Duration(8.0).sleep();

            mavros_msgs::CommandBool arm_srv;
            arm_srv.request.value = false;
            arm_cl.call(arm_srv);
            print("Emergency recovery complete.");
        } else {
            print("[SIGINT] Disarmed. Exiting safely.");
        }
    }

    spinner.stop();
    ros::shutdown();
    return 0;
}
