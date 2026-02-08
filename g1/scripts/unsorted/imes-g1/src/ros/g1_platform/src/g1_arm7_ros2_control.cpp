/*
  Dual Arm 7-DoF Controller for G1 Platform
*/

#include <chrono>
#include <cmath>
#include <thread>
#include <vector>
#include <map>
#include <mutex>
#include <atomic>
#include <algorithm>
#include <array>
#include <unordered_set>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using FollowJT = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJT>;

static const std::string kTopicArmSDK = "rt/arm_sdk";
static const std::string kTopicState = "rt/lowstate";

enum JointIndex
{
  // Left leg
  kLeftHipPitch,
  kLeftHipRoll,
  kLeftHipYaw,
  kLeftKnee,
  kLeftAnkle,
  kLeftAnkleRoll,
  // Right leg
  kRightHipPitch,
  kRightHipRoll,
  kRightHipYaw,
  kRightKnee,
  kRightAnkle,
  kRightAnkleRoll,
  // Trunk
  kWaistYaw,
  kWaistRoll,
  kWaistPitch,
  // Left arm (7 DoF)
  kLeftShoulderPitch,
  kLeftShoulderRoll,
  kLeftShoulderYaw,
  kLeftElbow,
  kLeftWristRoll,
  kLeftWristPitch,
  kLeftWristYaw,
  // Right arm (7 DoF)
  kRightShoulderPitch,
  kRightShoulderRoll,
  kRightShoulderYaw,
  kRightElbow,
  kRightWristRoll,
  kRightWristPitch,
  kRightWristYaw,
  // Misc / reserved
  kNotUsedJoint,
  kNotUsedJoint1,
  kNotUsedJoint2,
  kNotUsedJoint3,
  kNotUsedJoint4,
  kNotUsedJoint5
};

static inline double clamp(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }
static inline double sec_from_msg(const builtin_interfaces::msg::Duration &d)
{
  return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}

class G1DualArmSimple final : public rclcpp::Node
{
public:
  explicit G1DualArmSimple(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions())
      : rclcpp::Node("g1_dual_arm_simple", opts)
  {
    // ---- Parameters (simple) ----
    eth_ = declare_parameter<std::string>("eth", "eth0");
    control_dt_ = declare_parameter<double>("control_dt", 0.02); // 50 Hz
    rate_hz_ = std::max(1.0, 1.0 / control_dt_);

    // Gains - SMOOTHER DEFAULTS
    kp_move_ = declare_parameter<double>("kp", 30.0); // Reduced from 40.0
    kd_move_ = declare_parameter<double>("kd", 1.0);  // Reduced from 1.2
    tau_ff_ = declare_parameter<double>("tau_ff", 0.0);
    kp_hold_ = declare_parameter<double>("hold_kp", 6.0); // Reduced from 8.0
    kd_hold_ = declare_parameter<double>("hold_kd", 0.6); // Reduced from 0.8

    // Transition gains (even smoother for start/stop)
    kp_transition_ = declare_parameter<double>("transition_kp", 15.0); // NEW
    kd_transition_ = declare_parameter<double>("transition_kd", 0.5);  // NEW

    // Fixed weight (held)
    weight_fixed_ = declare_parameter<double>("weight", 0.75); // 0..1 — constant every tick

    // Limits (shared per joint index) - SMOOTHER VELOCITY LIMITS
    max_vel_ = declare_parameter<double>("max_joint_velocity", 0.30); // Reduced from 0.50
    vel_lim_ = declare_parameter<std::vector<double>>("velocity_limits", std::vector<double>(7, max_vel_));
    pos_min_ = declare_parameter<std::vector<double>>("pos_limits_min", std::vector<double>(7, -M_PI));
    pos_max_ = declare_parameter<std::vector<double>>("pos_limits_max", std::vector<double>(7, M_PI));

    // Smooth transition parameters
    transition_time_ = declare_parameter<double>("transition_time", 1.5);         // Time for start transitions
    stop_time_ = declare_parameter<double>("stop_time", 0.8);                     // Time to gradually stop
    startup_delay_ = declare_parameter<double>("startup_delay", 2.0);             // Delay before startup motion
    min_move_threshold_ = declare_parameter<double>("min_move_threshold", 0.087); // ~5 degrees minimum movement

    // Names
    names_left_ = declare_parameter<std::vector<std::string>>("joint_names_left",
                                                              std::vector<std::string>{
                                                                  "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
                                                                  "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw"});
    names_right_ = declare_parameter<std::vector<std::string>>("joint_names_right",
                                                               std::vector<std::string>{
                                                                   "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
                                                                   "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw"});

    // Startup
    startup_on_launch_ = declare_parameter<bool>("startup_on_launch", true);
    startup_time_ = declare_parameter<double>("startup_time", 3.0); // Increased from 2.0
    startup_left_ = declare_parameter<std::vector<double>>("startup_pose_left", std::vector<double>(7, 0.0));
    startup_right_ = declare_parameter<std::vector<double>>("startup_pose_right", std::vector<double>(7, 0.0));

    // ---- Unitree DDS ----
    unitree::robot::ChannelFactory::Instance()->Init(0, eth_);
    arm_pub_.reset(new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(kTopicArmSDK));
    arm_pub_->InitChannel();
    low_state_sub_.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(kTopicState));
    low_state_sub_->InitChannel([&](const void *m)
                                {
      const auto* s = (const unitree_hg::msg::dds_::LowState_*)m;
      std::lock_guard<std::mutex> lk(state_mtx_);
      memcpy(&state_msg_, s, sizeof(state_msg_)); }, 1);

    // Optional /joint_states (used if available)
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&G1DualArmSimple::onJointStates, this, std::placeholders::_1));

    // Services
    srv_start_ = create_service<std_srvs::srv::Trigger>(
        "hardware/control/arm_dof7/start",
        std::bind(&G1DualArmSimple::onStart, this, std::placeholders::_1, std::placeholders::_2));

    srv_stop_ = create_service<std_srvs::srv::Trigger>(
        "hardware/control/arm_dof7/stop",
        std::bind(&G1DualArmSimple::onStop, this, std::placeholders::_1, std::placeholders::_2));

    // Action servers (left/right)
    server_left_ = rclcpp_action::create_server<FollowJT>(
        this, "left_arm/follow_joint_trajectory",
        std::bind(&G1DualArmSimple::onGoalLeft, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&G1DualArmSimple::onCancelLeft, this, std::placeholders::_1),
        std::bind(&G1DualArmSimple::onAcceptLeft, this, std::placeholders::_1));

    server_right_ = rclcpp_action::create_server<FollowJT>(
        this, "right_arm/follow_joint_trajectory",
        std::bind(&G1DualArmSimple::onGoalRight, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&G1DualArmSimple::onCancelRight, this, std::placeholders::_1),
        std::bind(&G1DualArmSimple::onAcceptRight, this, std::placeholders::_1));

    // Action server (dual-arm, 14 joints)
    server_dual_ = rclcpp_action::create_server<FollowJT>(
        this, "dual_arm/follow_joint_trajectory",
        std::bind(&G1DualArmSimple::onGoalDual, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&G1DualArmSimple::onCancelDual, this, std::placeholders::_1),
        std::bind(&G1DualArmSimple::onAcceptDual, this, std::placeholders::_1));

    // Timer publisher (single writer -> always both arms + weight)
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000.0)),
        std::bind(&G1DualArmSimple::tick, this));

    // Init desired from current state (then optional startup)
    readCurrent(left_des_, idx_left);
    readCurrent(right_des_, idx_right);
    left_target_ = left_des_;
    right_target_ = right_des_;
    armed_.store(true);

    // IMPROVED STARTUP LOGIC
    if (startup_on_launch_)
    {
      RCLCPP_INFO(get_logger(), "Waiting %.1f seconds before startup motion...", startup_delay_);

      // Create a one-shot timer for delayed startup instead of immediate
      startup_timer_ = create_wall_timer(
          std::chrono::milliseconds(static_cast<int>(startup_delay_ * 1000.0)),
          [this]()
          {
            this->performStartupMove();
            startup_timer_->cancel(); // Cancel after first execution
          });
    }

    RCLCPP_INFO(get_logger(), "DualArmSimple ready. weight=%.2f, rate=%.1f Hz", weight_fixed_, rate_hz_);

    // Validate parameters
    if (control_dt_ < 0.01)
    {
      RCLCPP_WARN(get_logger(), "Control rate very high (%.3f ms), consider increasing to reduce violence", control_dt_ * 1000);
    }
  }

private:
  // Indices
  static constexpr std::array<int, 7> idx_left{
      kLeftShoulderPitch, kLeftShoulderRoll, kLeftShoulderYaw, kLeftElbow,
      kLeftWristRoll, kLeftWristPitch, kLeftWristYaw};
  static constexpr std::array<int, 7> idx_right{
      kRightShoulderPitch, kRightShoulderRoll, kRightShoulderYaw, kRightElbow,
      kRightWristRoll, kRightWristPitch, kRightWristYaw};

  // ------ Helpers ------
  static std::array<double, 7> vecToArray(const std::vector<double> &v)
  {
    std::array<double, 7> a{};
    for (size_t i = 0; i < 7 && i < v.size(); ++i)
      a[i] = v[i];
    return a;
  }

  void onJointStates(const sensor_msgs::msg::JointState::SharedPtr js)
  {
    std::lock_guard<std::mutex> lk(js_mtx_);
    last_js_ = *js;
  }

  void readCurrent(std::array<double, 7> &out, const std::array<int, 7> &idx)
  {
    // Try /joint_states first (by name); else LowState by index
    {
      std::lock_guard<std::mutex> lk(js_mtx_);
      if (!last_js_.name.empty() && last_js_.name.size() == last_js_.position.size())
      {
        const auto &names = (&idx == &idx_left) ? names_left_ : names_right_;
        std::map<std::string, double> m;
        for (size_t i = 0; i < last_js_.name.size(); ++i)
          m[last_js_.name[i]] = last_js_.position[i];
        bool ok = true;
        for (int j = 0; j < 7; ++j)
        {
          auto it = m.find(names[j]);
          if (it == m.end())
          {
            ok = false;
            break;
          }
          out[j] = clamp(it->second, pos_min_[j], pos_max_[j]);
        }
        if (ok)
          return;
      }
    }
    // Fallback LowState
    std::lock_guard<std::mutex> lk(state_mtx_);
    int n = state_msg_.motor_state().size();
    for (int j = 0; j < 7; ++j)
    {
      int id = idx[j];
      out[j] = (id >= 0 && id < n) ? state_msg_.motor_state().at(id).q() : 0.0;
    }
  }

  void startLinearMove(std::array<double, 7> &from, const std::array<double, 7> &to,
                       std::atomic<int> &steps_out, double time_s)
  {
    from = clampArray(from);
    auto tgt = clampArray(to);
    // If time is provided use it; else derive from velocity limits
    int steps = (time_s > 0.0) ? std::max(1, static_cast<int>(time_s * rate_hz_)) : 1;
    if (time_s <= 0.0)
    {
      double max_steps_needed = 1.0;
      for (int j = 0; j < 7; ++j)
      {
        double per_step = vel_lim_[j] / rate_hz_;
        if (per_step <= 0)
          per_step = 1e-6;
        max_steps_needed = std::max(max_steps_needed, std::fabs(tgt[j] - from[j]) / per_step);
      }
      steps = static_cast<int>(std::ceil(max_steps_needed));
    }
    target_buf_ = tgt; // temporary buffer used by the caller before setting target_
    steps_out.store(steps);
  }

  std::array<double, 7> clampArray(const std::array<double, 7> &q) const
  {
    std::array<double, 7> c{};
    for (int j = 0; j < 7; ++j)
      c[j] = clamp(q[j], pos_min_[j], pos_max_[j]);
    return c;
  }

  // Safe startup with position checking
  void performStartupMove()
  {
    // Read current positions again (they might have changed)
    std::array<double, 7> current_left, current_right;
    readCurrent(current_left, idx_left);
    readCurrent(current_right, idx_right);

    // Check if startup positions are significantly different from current
    auto startup_left_arr = vecToArray(startup_left_);
    auto startup_right_arr = vecToArray(startup_right_);

    double max_diff_left = 0.0, max_diff_right = 0.0;
    for (int i = 0; i < 7; ++i)
    {
      max_diff_left = std::max(max_diff_left, std::abs(current_left[i] - startup_left_arr[i]));
      max_diff_right = std::max(max_diff_right, std::abs(current_right[i] - startup_right_arr[i]));
    }

    // Only move if the difference is significant
    if (max_diff_left > min_move_threshold_)
    {
      {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        startLinearMove(left_target_, startup_left_arr, left_steps_, startup_time_);
        left_target_ = target_buf_;
      }
      left_moving_.store(true);
      left_transitioning_.store(true); // NEW FLAG
      RCLCPP_INFO(get_logger(), "Moving left arm to startup position (%.2f rad max diff)", max_diff_left);
    }

    if (max_diff_right > min_move_threshold_)
    {
      {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        startLinearMove(right_target_, startup_right_arr, right_steps_, startup_time_);
        right_target_ = target_buf_;
      }
      right_moving_.store(true);
      right_transitioning_.store(true); // NEW FLAG
      RCLCPP_INFO(get_logger(), "Moving right arm to startup position (%.2f rad max diff)", max_diff_right);
    }

    if (max_diff_left <= min_move_threshold_ && max_diff_right <= min_move_threshold_)
    {
      RCLCPP_INFO(get_logger(), "Arms already near startup positions, no movement needed");
    }
  }

  // ------ Services ------
  void onStart(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    // Instead of immediately snapping to current position, smoothly transition
    std::array<double, 7> current_left, current_right;
    readCurrent(current_left, idx_left);
    readCurrent(current_right, idx_right);

    // If we're already moving, gradually slow down to current position
    if (left_moving_.load() || right_moving_.load())
    {
      // Cancel any existing moves first
      left_cancel_.store(true);
      right_cancel_.store(true);

      // Wait a moment for cancellation to take effect
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      // Start smooth move from current desired positions to measured positions
      startLinearMove(left_des_, current_left, left_steps_, transition_time_);
      left_target_ = target_buf_;

      startLinearMove(right_des_, current_right, right_steps_, transition_time_);
      right_target_ = target_buf_;
    }

    left_moving_.store(true); // Enable smooth movement
    right_moving_.store(true);
    left_transitioning_.store(true); // Use transition gains
    right_transitioning_.store(true);
    left_cancel_.store(false);
    right_cancel_.store(false);
    armed_.store(true);

    res->success = true;
    res->message = "Smoothly transitioning to armed state with current positions.";
    RCLCPP_INFO(get_logger(), "Start service: smooth transition to current positions over %.1fs", transition_time_);
  }

  void onStop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
              std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    // Signal cancellation
    left_cancel_.store(true);
    right_cancel_.store(true);

    // Don't immediately stop - let the current motion complete over a short time
    // Reduce the remaining steps to create a gradual stop
    int left_remaining = left_steps_.load();
    int right_remaining = right_steps_.load();

    // Limit remaining steps to create a gradual stop
    int max_stop_steps = static_cast<int>(stop_time_ * rate_hz_);

    if (left_remaining > max_stop_steps)
    {
      left_steps_.store(max_stop_steps);
      RCLCPP_INFO(get_logger(), "Left arm: reducing from %d to %d steps for gradual stop", left_remaining, max_stop_steps);
    }
    if (right_remaining > max_stop_steps)
    {
      right_steps_.store(max_stop_steps);
      RCLCPP_INFO(get_logger(), "Right arm: reducing from %d to %d steps for gradual stop", right_remaining, max_stop_steps);
    }

    // Enable transition mode for smoother stopping
    left_transitioning_.store(true);
    right_transitioning_.store(true);

    // Keep armed to maintain position holding
    armed_.store(true);

    res->success = true;
    res->message = "Gradually stopping both arms; will hold final positions.";
    RCLCPP_INFO(get_logger(), "Stop service: gradual stop over %.1fs", stop_time_);
  }

  // ------ Action servers (Left) ------
  rclcpp_action::GoalResponse onGoalLeft(const rclcpp_action::GoalUUID &,
                                         std::shared_ptr<const FollowJT::Goal> goal)
  {
    if (!armed_.load())
      return rclcpp_action::GoalResponse::REJECT;
    if (goal->trajectory.joint_names.size() != 7 || goal->trajectory.points.empty())
      return rclcpp_action::GoalResponse::REJECT;
    for (const auto &n : goal->trajectory.joint_names)
      if (std::find(names_left_.begin(), names_left_.end(), n) == names_left_.end())
        return rclcpp_action::GoalResponse::REJECT;
    double prev = -1.0;
    for (const auto &p : goal->trajectory.points)
    {
      const double t = sec_from_msg(p.time_from_start);
      if (t <= prev)
        return rclcpp_action::GoalResponse::REJECT;
      prev = t;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse onCancelLeft(const std::shared_ptr<GoalHandleFJT>)
  {
    left_cancel_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void onAcceptLeft(const std::shared_ptr<GoalHandleFJT> gh)
  {
    std::thread([this, gh]
                { executeArm(gh, /*left=*/true); })
        .detach();
  }

  // ------ Action servers (Right) ------
  rclcpp_action::GoalResponse onGoalRight(const rclcpp_action::GoalUUID &,
                                          std::shared_ptr<const FollowJT::Goal> goal)
  {
    if (!armed_.load())
      return rclcpp_action::GoalResponse::REJECT;
    if (goal->trajectory.joint_names.size() != 7 || goal->trajectory.points.empty())
      return rclcpp_action::GoalResponse::REJECT;
    for (const auto &n : goal->trajectory.joint_names)
      if (std::find(names_right_.begin(), names_right_.end(), n) == names_right_.end())
        return rclcpp_action::GoalResponse::REJECT;
    double prev = -1.0;
    for (const auto &p : goal->trajectory.points)
    {
      const double t = sec_from_msg(p.time_from_start);
      if (t <= prev)
        return rclcpp_action::GoalResponse::REJECT;
      prev = t;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse onCancelRight(const std::shared_ptr<GoalHandleFJT>)
  {
    right_cancel_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void onAcceptRight(const std::shared_ptr<GoalHandleFJT> gh)
  {
    std::thread([this, gh]
                { executeArm(gh, /*left=*/false); })
        .detach();
  }

  // ------ Action server (Dual: 14 joints) ------
  rclcpp_action::GoalResponse onGoalDual(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const FollowJT::Goal> goal)
  {
    if (!armed_.load())
      return rclcpp_action::GoalResponse::REJECT;
    const auto &jn = goal->trajectory.joint_names;
    const auto &pts = goal->trajectory.points;
    if (jn.size() != 14 || pts.empty())
      return rclcpp_action::GoalResponse::REJECT;

    // Ensure all left & right names are present (order can be arbitrary)
    for (const auto &n : names_left_)
      if (std::find(jn.begin(), jn.end(), n) == jn.end())
        return rclcpp_action::GoalResponse::REJECT;
    for (const auto &n : names_right_)
      if (std::find(jn.begin(), jn.end(), n) == jn.end())
        return rclcpp_action::GoalResponse::REJECT;

    // Reject duplicates
    std::unordered_set<std::string> s(jn.begin(), jn.end());
    if (s.size() != jn.size())
      return rclcpp_action::GoalResponse::REJECT;

    // Strictly increasing times
    double prev = -1.0;
    for (const auto &p : pts)
    {
      const double t = sec_from_msg(p.time_from_start);
      if (t <= prev)
        return rclcpp_action::GoalResponse::REJECT;
      prev = t;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse onCancelDual(const std::shared_ptr<GoalHandleFJT>)
  {
    left_cancel_.store(true);
    right_cancel_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void onAcceptDual(const std::shared_ptr<GoalHandleFJT> gh)
  {
    // Preempt any ongoing motions
    left_cancel_.store(true);
    right_cancel_.store(true);
    // brief pause to let cancellation propagate
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if (dual_busy_.exchange(true))
    {
      auto result = std::make_shared<FollowJT::Result>();
      gh->abort(result);
      return;
    }
    std::thread([this, gh]
                {
  executeDual(gh);
  dual_busy_.store(false); })
        .detach();
  }

  void executeDual(std::shared_ptr<GoalHandleFJT> gh)
  {
    auto goal = gh->get_goal();
    auto feedback = std::make_shared<FollowJT::Feedback>();
    auto result = std::make_shared<FollowJT::Result>();

    const auto &jn = goal->trajectory.joint_names;
    const auto &pts = goal->trajectory.points;

    // Build permutations from incoming order -> left/right index
    std::array<int, 14> perm_all{};
    for (int i = 0; i < 14; ++i)
      perm_all[i] = -1;

    // Map joint name -> (isLeft, idx0..6)
    auto map_index = [&](const std::string &name, bool &is_left, int &jidx) -> bool
    {
      auto itL = std::find(names_left_.begin(), names_left_.end(), name);
      if (itL != names_left_.end())
      {
        is_left = true;
        jidx = int(std::distance(names_left_.begin(), itL));
        return true;
      }
      auto itR = std::find(names_right_.begin(), names_right_.end(), name);
      if (itR != names_right_.end())
      {
        is_left = false;
        jidx = int(std::distance(names_right_.begin(), itR));
        return true;
      }
      return false;
    };

    struct Slot
    {
      bool is_left;
      int jidx;
    };
    std::array<Slot, 14> slots{};
    for (int i = 0; i < 14; ++i)
    {
      bool is_left;
      int jidx;
      if (!map_index(jn[i], is_left, jidx))
      {
        gh->abort(result);
        return;
      }
      slots[i] = {is_left, jidx};
    }

    // Current desired snapshots (start from controller's current)
    std::array<double, 7> curL, curR;
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      curL = left_des_;
      curR = right_des_;
    }

    // Loop each trajectory point
    for (size_t pt = 0; pt < pts.size(); ++pt)
    {
      if (gh->is_canceling())
      {
        gh->canceled(result);
        return;
      }

      const auto &point = pts[pt];
      if (point.positions.size() != 14)
      {
        gh->abort(result);
        return;
      }

      // Split incoming 14 positions into left/right targets
      std::array<double, 7> tgtL{}, tgtR{};
      for (int i = 0; i < 14; ++i)
      {
        double v = point.positions[i];
        if (slots[i].is_left)
          tgtL[slots[i].jidx] = clamp(v, pos_min_[slots[i].jidx], pos_max_[slots[i].jidx]);
        else
          tgtR[slots[i].jidx] = clamp(v, pos_min_[slots[i].jidx], pos_max_[slots[i].jidx]);
      }

      // Segment duration
      const double T = std::max(
          sec_from_msg(point.time_from_start) -
              (pt == 0 ? 0.0 : sec_from_msg(pts[pt - 1].time_from_start)),
          0.0);

      // Arm both sides with the same T
      {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        startLinearMove(curL, tgtL, left_steps_, T);
        left_target_ = target_buf_;
        startLinearMove(curR, tgtR, right_steps_, T);
        right_target_ = target_buf_;
      }
      left_cancel_.store(false);
      right_cancel_.store(false);
      left_moving_.store(true);
      right_moving_.store(true);
      left_transitioning_.store(false);
      right_transitioning_.store(false);

      // Wait until BOTH arms finish this segment (or are cancelled)
      rclcpp::Rate rate(rate_hz_);
      while (rclcpp::ok())
      {
        // 1) Client cancel request → valid CANCELED
        if (gh->is_canceling())
        {
          gh->canceled(result);
          return;
        }

        // 2) Server-side preemption → ABORT, not CANCELED
        if (left_cancel_.load() || right_cancel_.load())
        {
          gh->abort(result);
          return;
        }
        const int ls = left_steps_.load();
        const int rs = right_steps_.load();

        // Publish combined feedback (14 joints, incoming order)
        feedback->joint_names = jn;
        {
          std::array<double, 7> desL{}, desR{};
          {
            std::lock_guard<std::mutex> lk(cmd_mtx_);
            desL = left_des_;
            desR = right_des_;
          }
          feedback->actual.positions.resize(14);
          feedback->desired.positions.resize(14);
          feedback->error.positions.resize(14);
          for (int i = 0; i < 14; ++i)
          {
            const bool L = slots[i].is_left;
            const int j = slots[i].jidx;
            const double a = L ? desL[j] : desR[j];
            const double d = L ? tgtL[j] : tgtR[j];
            feedback->actual.positions[i] = a;
            feedback->desired.positions[i] = d;
            feedback->error.positions[i] = d - a;
          }
        }
        feedback->actual.time_from_start = goal->trajectory.points[pt].time_from_start;
        feedback->desired.time_from_start = goal->trajectory.points[pt].time_from_start;
        gh->publish_feedback(feedback);

        if (ls <= 0 && rs <= 0)
          break;
        rate.sleep();
      }

      // Update bases for next segment
      curL = tgtL;
      curR = tgtR;
    }

    left_moving_.store(false);
    right_moving_.store(false);
    left_transitioning_.store(false);
    right_transitioning_.store(false);
    gh->succeed(result);
  }

  // ------ Execute arm (very simple: set target & step each tick) ------
  void executeArm(std::shared_ptr<GoalHandleFJT> gh, bool left)
  {
    auto goal = gh->get_goal();
    auto feedback = std::make_shared<FollowJT::Feedback>();
    auto result = std::make_shared<FollowJT::Result>();

    const auto &expected = left ? names_left_ : names_right_;
    std::vector<int> perm(7, -1);
    for (size_t i = 0; i < 7; ++i)
    {
      auto it = std::find(expected.begin(), expected.end(), goal->trajectory.joint_names[i]);
      if (it == expected.end())
      {
        gh->abort(result);
        return;
      }
      perm[i] = static_cast<int>(std::distance(expected.begin(), it));
    }

    auto current_des = left ? left_des_ : right_des_;

    // Loop through each trajectory point
    for (size_t pt = 0; pt < goal->trajectory.points.size(); ++pt)
    {
      // 1) Client requested cancel → report CANCELED (valid transition)
      if (gh->is_canceling())
      {
        gh->canceled(result);
        return;
      }

      // 2) Internal preemption (server-side) → report ABORTED (not canceled)
      if ((left && left_cancel_.load()) || (!left && right_cancel_.load()))
      {
        gh->abort(result);
        return;
      }

      const auto &point = goal->trajectory.points[pt];
      if (point.positions.size() != 7)
      {
        gh->abort(result);
        return;
      }

      std::array<double, 7> q_tgt{};
      for (size_t i = 0; i < 7; ++i)
        q_tgt[perm[i]] = clamp(point.positions[i], pos_min_[i], pos_max_[i]);

      double T = std::max(sec_from_msg(point.time_from_start) -
                              (pt == 0 ? 0.0 : sec_from_msg(goal->trajectory.points[pt - 1].time_from_start)),
                          0.0);

      {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        startLinearMove(current_des, q_tgt,
                        left ? left_steps_ : right_steps_, T);
        if (left)
        {
          left_target_ = target_buf_;
        }
        else
        {
          right_target_ = target_buf_;
        }
      }

      if (left)
      {
        left_cancel_.store(false);
        left_moving_.store(true);
        left_transitioning_.store(false);
      }
      else
      {
        right_cancel_.store(false);
        right_moving_.store(true);
        right_transitioning_.store(false);
      }

      // Wait until this segment finishes
      rclcpp::Rate rate(rate_hz_);
      while (rclcpp::ok())
      {
        if ((left && left_cancel_.load()) || (!left && right_cancel_.load()))
        {
          gh->canceled(result);
          return;
        }
        int steps = left ? left_steps_.load() : right_steps_.load();

        // Publish feedback
        feedback->joint_names = expected;
        {
          std::array<double, 7> des{};
          {
            std::lock_guard<std::mutex> lk(cmd_mtx_);
            des = left ? left_des_ : right_des_;
          }
          feedback->actual.positions.assign(des.begin(), des.end());
          feedback->desired.positions.assign(q_tgt.begin(), q_tgt.end());
          feedback->error.positions.resize(7);
          for (size_t j = 0; j < 7; ++j)
            feedback->error.positions[j] = q_tgt[j] - des[j];
        }
        feedback->actual.time_from_start = goal->trajectory.points[pt].time_from_start;
        feedback->desired.time_from_start = goal->trajectory.points[pt].time_from_start;
        gh->publish_feedback(feedback);

        if (steps <= 0)
          break;
        rate.sleep();
      }

      // Update current position for next segment
      current_des = q_tgt;
    }

    if (left)
    {
      left_moving_.store(false);
      left_transitioning_.store(false);
    }
    else
    {
      right_moving_.store(false);
      right_transitioning_.store(false);
    }
    gh->succeed(result);
  }

  // ------ Main 50 Hz publisher (IMPROVED) ------
  void tick()
  {
    if (!armed_.load())
      return;

    // Step desired toward target at limited velocity
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      stepToward(left_des_, left_target_, left_steps_);
      stepToward(right_des_, right_target_, right_steps_);
    }

    // Gains: transition vs move vs hold
    const bool Lmoving = left_moving_.load();
    const bool Rmoving = right_moving_.load();
    const bool Ltransition = left_transitioning_.load();
    const bool Rtransition = right_transitioning_.load();

    // Use different gains for transitions (smoother)
    double kpL, kdL, kpR, kdR;
    if (Ltransition)
    {
      kpL = kp_transition_;
      kdL = kd_transition_;
    }
    else if (Lmoving)
    {
      kpL = kp_move_;
      kdL = kd_move_;
    }
    else
    {
      kpL = kp_hold_;
      kdL = kd_hold_;
    }

    if (Rtransition)
    {
      kpR = kp_transition_;
      kdR = kd_transition_;
    }
    else if (Rmoving)
    {
      kpR = kp_move_;
      kdR = kd_move_;
    }
    else
    {
      kpR = kp_hold_;
      kdR = kd_hold_;
    }

    // Publish one LowCmd with both arms + fixed weight
    sendLowCmdBoth(left_des_, kpL, kdL, right_des_, kpR, kdR, weight_fixed_);

    // Clear transition flags when movement completes
    if (Ltransition && left_steps_.load() <= 0)
    {
      left_transitioning_.store(false);
    }
    if (Rtransition && right_steps_.load() <= 0)
    {
      right_transitioning_.store(false);
    }
  }

  // IMPROVED stepToward method with smoother deceleration
  void stepToward(std::array<double, 7> &cur, const std::array<double, 7> &tgt, std::atomic<int> &steps)
  {
    int s = steps.load();
    if (s <= 0)
      return;

    const double dt_step = 1.0 / rate_hz_;
    bool done = true;

    // Add deceleration factor as we approach the target (smoother stopping)
    double decel_factor = 1.0;
    const int decel_steps = 15; // Start decelerating in last 15 steps
    if (s <= decel_steps)
    {
      // Smooth sigmoid-like deceleration curve
      double progress = static_cast<double>(s) / decel_steps;
      decel_factor = 0.3 + 0.7 * progress; // Range from 0.3 to 1.0
    }

    for (int j = 0; j < 7; ++j)
    {
      const double per_step = vel_lim_[j] * dt_step * decel_factor;
      double delta = clamp(tgt[j] - cur[j], -per_step, per_step);
      cur[j] = clamp(cur[j] + delta, pos_min_[j], pos_max_[j]);
      if (std::fabs(tgt[j] - cur[j]) > 1e-4)
        done = false;
    }

    if (done || s == 1)
      steps.store(0);
    else
      steps.store(s - 1);
  }

  void sendLowCmdBoth(const std::array<double, 7> &qL, double kpL, double kdL,
                      const std::array<double, 7> &qR, double kpR, double kdR,
                      double weight_now)
  {
    unitree_hg::msg::dds_::LowCmd_ msg;
    const int ncmd = msg.motor_cmd().size();

    auto apply = [&](const std::array<int, 7> &idx, const std::array<double, 7> &q, double kp, double kd)
    {
      for (int j = 0; j < 7; ++j)
      {
        int id = idx[j];
        if (id < 0 || id >= ncmd)
          continue;
        msg.motor_cmd().at(id).q(static_cast<float>(q[j]));
        msg.motor_cmd().at(id).dq(0.0f);
        msg.motor_cmd().at(id).kp(static_cast<float>(kp));
        msg.motor_cmd().at(id).kd(static_cast<float>(kd));
        msg.motor_cmd().at(id).tau(static_cast<float>(tau_ff_));
      }
    };

    apply(idx_left, qL, kpL, kdL);
    apply(idx_right, qR, kpR, kdR);

    // Fixed weight embedded in reserved slot (held every tick)
    if (kNotUsedJoint >= 0 && kNotUsedJoint < ncmd)
      msg.motor_cmd().at(kNotUsedJoint).q(static_cast<float>(clamp(weight_now, 0.0, 1.0)));

    arm_pub_->Write(msg);
  }

private:
  // Params
  std::string eth_;
  double control_dt_{0.02}, rate_hz_{50.0};
  double kp_move_{30.0}, kd_move_{1.0}, tau_ff_{0.0}; // Reduced gains
  double kp_hold_{6.0}, kd_hold_{0.6};                // Reduced gains
  double kp_transition_{15.0}, kd_transition_{0.5};   // NEW: Transition gains
  double weight_fixed_{0.75};
  double max_vel_{0.3}; // Reduced max velocity
  std::vector<double> vel_lim_, pos_min_, pos_max_;
  std::vector<std::string> names_left_, names_right_;
  bool startup_on_launch_{true};
  double startup_time_{3.0}; // Increased startup time
  std::vector<double> startup_left_, startup_right_;

  // NEW: Smooth transition parameters
  double transition_time_{1.5};
  double stop_time_{0.8};
  double startup_delay_{2.0};
  double min_move_threshold_{0.087};

  // State
  std::mutex state_mtx_, js_mtx_, cmd_mtx_;
  unitree_hg::msg::dds_::LowState_ state_msg_{};
  sensor_msgs::msg::JointState last_js_{};

  std::array<double, 7> left_des_{}, right_des_{};
  std::array<double, 7> left_target_{}, right_target_{};
  std::array<double, 7> target_buf_{}; // helper

  std::atomic<bool> dual_busy_{false};
  std::atomic<int> left_steps_{0}, right_steps_{0};
  std::atomic<bool> armed_{true};
  std::atomic<bool> left_moving_{false}, right_moving_{false};
  std::atomic<bool> left_cancel_{false}, right_cancel_{false};
  std::atomic<bool> left_transitioning_{false}, right_transitioning_{false}; // NEW: Transition flags

  // DDS & ROS
  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> arm_pub_;
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> low_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp_action::Server<FollowJT>::SharedPtr server_left_, server_right_, server_dual_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_, srv_stop_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_; // NEW: Startup timer
};

// --------------------------- MAIN ---------------------------
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Create node with default options
  auto node = std::make_shared<G1DualArmSimple>();

  // Spin the node
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}