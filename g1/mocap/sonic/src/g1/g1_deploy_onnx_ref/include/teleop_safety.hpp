/**
 * @file teleop_safety.hpp
 * @brief Smoothing and safety supervision for full-body mocap teleoperation.
 *
 * The SONIC policy does not consume mocap joint angles as motor targets.  The
 * mocap stream is a *reference motion* that the encoder/policy tracks while the
 * policy keeps the robot balanced.  This layer therefore shapes the reference
 * (not the policy output) and adds a thin guard on the motor targets:
 *
 *   ZMQ mocap window ──► validate ──► URDF clamp ──► critically damped LPF
 *       ──► per-joint velocity / acceleration / per-cycle step limiter
 *       ──► gate (dead-man, mocap timeout, temperature, startup ramp)
 *       ──► filtered reference window ──► encoder / policy
 *
 *   policy q_target ──► non-finite check ──► per-cycle jump guard
 *       ──► URDF position clamp ──► 500 Hz DDS writer (with watchdog)
 *
 * Joint arrays handled by Supervisor are in **IsaacLab order** (the order of
 * MotionSequence joint data); motor targets handled by GuardMotorTargets() are
 * in **hardware/URDF order**.
 *
 * This header has no Unitree SDK / CUDA dependencies so that it can be unit
 * tested offline (see tests/test_teleop_safety.cpp).
 */

#ifndef TELEOP_SAFETY_HPP
#define TELEOP_SAFETY_HPP

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "policy_parameters.hpp"

namespace teleop_safety {

constexpr int kNumJoints = 29;
/// Longest reference window (frames) that the filter rolls out per tick.
/// The release encoder looks 45 frames ahead; 64 leaves headroom.
constexpr int kMaxWindowFrames = 64;

using JointArray = std::array<double, kNumJoints>;
using Quat = std::array<double, 4>;  // w, x, y, z

// ---------------------------------------------------------------------------
// G1 29-DOF joint limits (hardware / URDF order)
//
// Source: g1_description/urdf/g1_29dof_with_hand_rev_1_0_pkg.urdf (position
// limits identical to unitree_robots/g1/g1_29dof.xml).  Velocity in rad/s.
// The URDF provides no acceleration limits; see Config::nominal_acceleration_limit.
// ---------------------------------------------------------------------------
struct JointLimit {
  const char* name;
  double lower;
  double upper;
  double velocity;
};

inline constexpr std::array<JointLimit, kNumJoints> kG1JointLimits = {{
    {"left_hip_pitch", -2.5307, 2.8798, 32.0},
    {"left_hip_roll", -0.5236, 2.9671, 20.0},
    {"left_hip_yaw", -2.7576, 2.7576, 32.0},
    {"left_knee", -0.087267, 2.8798, 20.0},
    {"left_ankle_pitch", -0.87267, 0.5236, 30.0},
    {"left_ankle_roll", -0.2618, 0.2618, 30.0},
    {"right_hip_pitch", -2.5307, 2.8798, 32.0},
    {"right_hip_roll", -2.9671, 0.5236, 20.0},
    {"right_hip_yaw", -2.7576, 2.7576, 32.0},
    {"right_knee", -0.087267, 2.8798, 20.0},
    {"right_ankle_pitch", -0.87267, 0.5236, 30.0},
    {"right_ankle_roll", -0.2618, 0.2618, 30.0},
    {"waist_yaw", -2.618, 2.618, 32.0},
    {"waist_roll", -0.52, 0.52, 30.0},
    {"waist_pitch", -0.52, 0.52, 30.0},
    {"left_shoulder_pitch", -3.0892, 2.6704, 37.0},
    {"left_shoulder_roll", -1.5882, 2.2515, 37.0},
    {"left_shoulder_yaw", -2.618, 2.618, 37.0},
    {"left_elbow", -1.0472, 2.0944, 37.0},
    {"left_wrist_roll", -1.972222054, 1.972222054, 37.0},
    {"left_wrist_pitch", -1.614429558, 1.614429558, 22.0},
    {"left_wrist_yaw", -1.614429558, 1.614429558, 22.0},
    {"right_shoulder_pitch", -3.0892, 2.6704, 37.0},
    {"right_shoulder_roll", -2.2515, 1.5882, 37.0},
    {"right_shoulder_yaw", -2.618, 2.618, 37.0},
    {"right_elbow", -1.0472, 2.0944, 37.0},
    {"right_wrist_roll", -1.972222054, 1.972222054, 37.0},
    {"right_wrist_pitch", -1.614429558, 1.614429558, 22.0},
    {"right_wrist_yaw", -1.614429558, 1.614429558, 22.0},
}};

/// Hardware index of IsaacLab joint i (same mapping the policy uses).
inline int HwIndexOfIsaac(int isaac_index) { return mujoco_to_isaaclab[isaac_index]; }

// ===========================================================================
// Configuration
// ===========================================================================

/// All tunables.  Defaults are the conservative values used when a key is
/// absent from the config file.  See config/teleop_safety.conf for docs.
struct Config {
  double control_rate_hz = 50.0;
  double smoothing_time_ms = 200.0;
  double max_velocity_fraction = 0.25;
  double max_acceleration_fraction = 0.15;
  double nominal_acceleration_limit = 80.0;
  double max_step_per_cycle_rad = 0.04;
  double startup_ramp_s = 4.0;
  double init_ramp_s = 5.0;
  double mocap_timeout_ms = 200.0;
  double deadman_timeout_ms = 700.0;
  double reference_limit_margin_rad = 0.05;
  double invalid_frame_tolerance_rad = 0.30;
  double max_root_angular_velocity = 0.6;
  double root_smoothing_time_ms = 200.0;
  double command_watchdog_ms = 100.0;
  double output_max_step_rad = 0.35;
  double temperature_reduce_c = 70.0;
  double temperature_stop_c = 85.0;
  double temperature_hysteresis_c = 5.0;
  double temperature_reduce_factor = 0.5;
  double log_interval_s = 1.0;
  double status_interval_s = 1.0;
  double loop_overrun_factor = 1.5;
  double dry_run_synthetic_temperature_c = 40.0;
  double dry_run_inject_stall_after_s = 0.0;  ///< Dry-run fault injection; 0 = off.
  double dry_run_synthetic_joint_lag_ms = 0.0;  ///< 0 = synthetic joints stay still; >0 = echo targets.
  std::string csv_log_path;

  /// Path the config was loaded from (informational).
  std::string source_path = "<built-in defaults>";

  double ControlDt() const { return 1.0 / control_rate_hz; }
};

class ConfigLoader {
 public:
  /// Load `key = value` lines ('#' comments).  Unknown keys and malformed
  /// values throw: a typo must never silently fall back to a default.
  static Config LoadFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) { throw std::runtime_error("cannot open teleop safety config: " + path); }
    Config cfg;
    cfg.source_path = path;
    std::string line;
    int line_no = 0;
    while (std::getline(in, line)) {
      ++line_no;
      auto hash = line.find('#');
      if (hash != std::string::npos) { line = line.substr(0, hash); }
      line = Trim(line);
      if (line.empty()) { continue; }
      try {
        ApplyOverride(cfg, line);
      } catch (const std::exception& e) {
        throw std::runtime_error(path + ":" + std::to_string(line_no) + ": " + e.what());
      }
    }
    return cfg;
  }

  /// Apply one `key=value` assignment (also used by --safety-set).
  static void ApplyOverride(Config& cfg, const std::string& assignment) {
    auto eq = assignment.find('=');
    if (eq == std::string::npos) { throw std::runtime_error("expected key = value, got '" + assignment + "'"); }
    std::string key = Trim(assignment.substr(0, eq));
    std::string value = Trim(assignment.substr(eq + 1));
    if (key == "csv_log_path") {
      cfg.csv_log_path = value;
      return;
    }
    for (const auto& f : DoubleFields()) {
      if (key == f.key) {
        cfg.*(f.member) = ParseDouble(key, value);
        return;
      }
    }
    throw std::runtime_error("unknown teleop safety key '" + key + "'");
  }

  /// Range and consistency checks.  Throws on any violation.
  static void Validate(const Config& cfg) {
    for (const auto& f : DoubleFields()) {
      double v = cfg.*(f.member);
      if (!std::isfinite(v) || v < f.min || v > f.max) {
        std::ostringstream os;
        os << f.key << " = " << v << " is outside the allowed range [" << f.min << ", " << f.max << "]";
        throw std::runtime_error(os.str());
      }
    }
    if (cfg.temperature_reduce_c >= cfg.temperature_stop_c) {
      throw std::runtime_error("temperature_reduce_c must be below temperature_stop_c");
    }
  }

  /// Non-fatal advisories for values above the recommended starting range.
  static std::vector<std::string> Advisories(const Config& cfg) {
    std::vector<std::string> out;
    if (cfg.max_velocity_fraction > 0.30) { out.push_back("max_velocity_fraction above the recommended 0.20-0.30 starting range"); }
    if (cfg.max_acceleration_fraction > 0.20) { out.push_back("max_acceleration_fraction above the recommended 0.10-0.20 starting range"); }
    if (cfg.smoothing_time_ms < 150.0) { out.push_back("smoothing_time_ms below the recommended 150-250 ms range"); }
    if (cfg.startup_ramp_s < 3.0) { out.push_back("startup_ramp_s below the recommended 3-5 s range"); }
    return out;
  }

  static std::string Describe(const Config& cfg) {
    std::ostringstream os;
    os << "  source: " << cfg.source_path << "\n";
    for (const auto& f : DoubleFields()) {
      os << "  " << std::left << std::setw(34) << f.key << std::right << " = " << cfg.*(f.member) << "\n";
    }
    os << "  " << std::left << std::setw(34) << "csv_log_path" << std::right << " = "
       << (cfg.csv_log_path.empty() ? "<off>" : cfg.csv_log_path) << "\n";
    return os.str();
  }

 private:
  struct DoubleField {
    const char* key;
    double Config::*member;
    double min;
    double max;
  };

  // Upper bounds deliberately stop short of anything that would loosen the
  // robot's own protections (e.g. temperature thresholds cap at the existing
  // 90 C alarm; the init ramp can never be faster than the original 3 s).
  static const std::vector<DoubleField>& DoubleFields() {
    static const std::vector<DoubleField> fields = {
        {"control_rate_hz", &Config::control_rate_hz, 50.0, 50.0},
        {"smoothing_time_ms", &Config::smoothing_time_ms, 0.0, 1000.0},
        {"max_velocity_fraction", &Config::max_velocity_fraction, 0.01, 1.0},
        {"max_acceleration_fraction", &Config::max_acceleration_fraction, 0.01, 1.0},
        {"nominal_acceleration_limit", &Config::nominal_acceleration_limit, 1.0, 500.0},
        {"max_step_per_cycle_rad", &Config::max_step_per_cycle_rad, 0.001, 0.2},
        {"startup_ramp_s", &Config::startup_ramp_s, 0.5, 30.0},
        {"init_ramp_s", &Config::init_ramp_s, 3.0, 30.0},
        {"mocap_timeout_ms", &Config::mocap_timeout_ms, 20.0, 1000.0},
        {"deadman_timeout_ms", &Config::deadman_timeout_ms, 100.0, 2000.0},
        {"reference_limit_margin_rad", &Config::reference_limit_margin_rad, 0.0, 0.3},
        {"invalid_frame_tolerance_rad", &Config::invalid_frame_tolerance_rad, 0.0, 1.0},
        {"max_root_angular_velocity", &Config::max_root_angular_velocity, 0.05, 6.0},
        {"root_smoothing_time_ms", &Config::root_smoothing_time_ms, 0.0, 1000.0},
        {"command_watchdog_ms", &Config::command_watchdog_ms, 40.0, 500.0},
        {"output_max_step_rad", &Config::output_max_step_rad, 0.02, 1.0},
        {"temperature_reduce_c", &Config::temperature_reduce_c, 30.0, 90.0},
        {"temperature_stop_c", &Config::temperature_stop_c, 35.0, 90.0},
        {"temperature_hysteresis_c", &Config::temperature_hysteresis_c, 1.0, 20.0},
        {"temperature_reduce_factor", &Config::temperature_reduce_factor, 0.1, 1.0},
        {"log_interval_s", &Config::log_interval_s, 0.1, 60.0},
        {"status_interval_s", &Config::status_interval_s, 0.1, 60.0},
        {"loop_overrun_factor", &Config::loop_overrun_factor, 1.1, 10.0},
        {"dry_run_synthetic_temperature_c", &Config::dry_run_synthetic_temperature_c, 0.0, 120.0},
        {"dry_run_inject_stall_after_s", &Config::dry_run_inject_stall_after_s, 0.0, 3600.0},
        {"dry_run_synthetic_joint_lag_ms", &Config::dry_run_synthetic_joint_lag_ms, 0.0, 1000.0},
    };
    return fields;
  }

  static std::string Trim(const std::string& s) {
    const char* ws = " \t\r\n";
    auto b = s.find_first_not_of(ws);
    if (b == std::string::npos) { return ""; }
    auto e = s.find_last_not_of(ws);
    return s.substr(b, e - b + 1);
  }

  static double ParseDouble(const std::string& key, const std::string& value) {
    try {
      size_t used = 0;
      double v = std::stod(value, &used);
      if (used != value.size()) { throw std::invalid_argument("trailing characters"); }
      return v;
    } catch (const std::exception&) {
      throw std::runtime_error("invalid numeric value for " + key + ": '" + value + "'");
    }
  }
};

// ===========================================================================
// Rate-limited logging
// ===========================================================================

class RateLimitedLogger {
 public:
  explicit RateLimitedLogger(double interval_s = 1.0) : interval_s_(interval_s) {}

  /// Print `msg` at most once per interval per key; reports how many were suppressed.
  /// Returns true when the message was printed.
  bool Log(const std::string& key, double now_s, const std::string& msg) {
    auto& e = entries_[key];
    if (now_s - e.last_s < interval_s_) {
      ++e.suppressed;
      return false;
    }
    std::cout << "[TeleopSafety] " << msg;
    if (e.suppressed > 0) { std::cout << " (+" << e.suppressed << " suppressed)"; }
    std::cout << std::endl;
    e.last_s = now_s;
    e.suppressed = 0;
    return true;
  }

 private:
  struct Entry {
    double last_s = -1e18;
    int suppressed = 0;
  };
  double interval_s_;
  std::map<std::string, Entry> entries_;
};

// ===========================================================================
// Filters
// ===========================================================================

struct JointFilterState {
  JointArray lp1{};  ///< First low-pass stage.
  JointArray lp2{};  ///< Second low-pass stage (critically damped output).
  JointArray pos{};  ///< Limited reference position.
  JointArray vel{};  ///< Limited reference velocity.

  void Reset(const JointArray& q) {
    lp1 = q;
    lp2 = q;
    pos = q;
    vel.fill(0.0);
  }
};

struct StepLimits {
  JointArray vmax{};
  JointArray amax{};
  JointArray lo{};
  JointArray hi{};
};

struct StepStats {
  int vel_saturated = 0;
  int acc_saturated = 0;
  int pos_clamped = 0;
};

/**
 * One control step of the reference shaper for all joints.
 *
 * Stage 1: two cascaded first-order low-passes with time constant tau/2 each,
 * i.e. a critically damped second-order filter with ~tau total lag.
 * Stage 2: velocity command that never exceeds vmax, can always brake to the
 * target within amax (v <= sqrt(2·a·|e|)), and is acceleration limited.
 * Positions are clamped to [lo, hi]; hitting a bound zeroes the velocity.
 */
inline StepStats StepFilter(JointFilterState& s, const JointArray& target, double dt, double tau_s,
                            const StepLimits& lim) {
  StepStats st;
  const double alpha = (tau_s > 0.0) ? dt / (0.5 * tau_s + dt) : 1.0;
  for (int j = 0; j < kNumJoints; ++j) {
    s.lp1[j] += alpha * (target[j] - s.lp1[j]);
    s.lp2[j] += alpha * (s.lp1[j] - s.lp2[j]);

    const double e = s.lp2[j] - s.pos[j];
    const double ae = std::abs(e);
    const double vmax = lim.vmax[j];
    const double amax = lim.amax[j];
    double speed = std::min({vmax, std::sqrt(2.0 * amax * ae), ae / dt});
    if (speed >= vmax && ae / dt > vmax) { ++st.vel_saturated; }
    const double v_des = (e >= 0.0) ? speed : -speed;

    double dv = v_des - s.vel[j];
    const double dv_max = amax * dt;
    if (std::abs(dv) > dv_max) {
      dv = std::copysign(dv_max, dv);
      ++st.acc_saturated;
    }
    s.vel[j] = std::clamp(s.vel[j] + dv, -vmax, vmax);
    s.pos[j] += s.vel[j] * dt;

    if (s.pos[j] < lim.lo[j] || s.pos[j] > lim.hi[j]) {
      s.pos[j] = std::clamp(s.pos[j], lim.lo[j], lim.hi[j]);
      s.vel[j] = 0.0;
      ++st.pos_clamped;
    }
  }
  return st;
}

inline Quat QuatNormalize(const Quat& q) {
  double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (!(n > 1e-9)) { return {1.0, 0.0, 0.0, 0.0}; }
  return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

inline Quat QuatMul(const Quat& a, const Quat& b) {
  return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
          a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
          a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
          a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

inline Quat QuatConj(const Quat& q) { return {q[0], -q[1], -q[2], -q[3]}; }

inline Quat HeadingQuat(const Quat& q) {
  double yaw = std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
  return {std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
}

/// Angle (rad) between two unit quaternions.
inline double QuatAngle(const Quat& a, const Quat& b) {
  double d = std::abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
  return 2.0 * std::acos(std::min(1.0, d));
}

inline Quat QuatSlerp(const Quat& a, Quat b, double t) {
  double d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  if (d < 0.0) {
    for (auto& c : b) { c = -c; }
    d = -d;
  }
  if (d > 0.9995) {
    Quat r;
    for (int i = 0; i < 4; ++i) { r[i] = a[i] + t * (b[i] - a[i]); }
    return QuatNormalize(r);
  }
  double th = std::acos(d);
  double s = std::sin(th);
  double wa = std::sin((1.0 - t) * th) / s;
  double wb = std::sin(t * th) / s;
  return QuatNormalize({wa * a[0] + wb * b[0], wa * a[1] + wb * b[1], wa * a[2] + wb * b[2], wa * a[3] + wb * b[3]});
}

/// First-order slerp toward target with a hard angular-rate cap.  Returns true when rate limited.
inline bool StepQuat(Quat& q, const Quat& target, double dt, double tau_s, double wmax) {
  double angle = QuatAngle(q, target);
  if (angle < 1e-9) { return false; }
  double step = (tau_s > 0.0) ? angle * dt / (tau_s + dt) : angle;
  bool limited = false;
  if (step > wmax * dt) {
    step = wmax * dt;
    limited = true;
  }
  q = QuatSlerp(q, target, step / angle);
  return limited;
}

// ===========================================================================
// Motor-target guard (hardware order)
// ===========================================================================

struct OutputGuardStats {
  int clamped = 0;
  int jump_limited = 0;
  bool nonfinite = false;
  int worst_joint = -1;  ///< Hardware index of the last guarded joint (for logs).
};

/// Enforce URDF position limits and a per-cycle jump guard on policy motor
/// targets.  A non-finite target is reported and must be treated as a fault.
inline OutputGuardStats GuardMotorTargets(std::array<float, kNumJoints>& q_hw,
                                          const std::array<float, kNumJoints>& prev_hw, bool have_prev,
                                          double max_step_rad) {
  OutputGuardStats st;
  for (int i = 0; i < kNumJoints; ++i) {
    double q = q_hw[i];
    if (!std::isfinite(q)) {
      st.nonfinite = true;
      st.worst_joint = i;
      continue;
    }
    if (have_prev && std::isfinite(prev_hw[i])) {
      double dq = q - prev_hw[i];
      if (std::abs(dq) > max_step_rad) {
        q = prev_hw[i] + std::copysign(max_step_rad, dq);
        ++st.jump_limited;
        st.worst_joint = i;
      }
    }
    const auto& lim = kG1JointLimits[i];
    if (q < lim.lower || q > lim.upper) {
      q = std::clamp(q, lim.lower, lim.upper);
      ++st.clamped;
      st.worst_joint = i;
    }
    q_hw[i] = static_cast<float>(q);
  }
  return st;
}

// ===========================================================================
// Supervisor
// ===========================================================================

enum class Mode { kUninitialized, kHold, kRamp, kFollow };
enum class TempState { kNormal, kReduced, kStopped };

inline const char* ModeName(Mode m) {
  switch (m) {
    case Mode::kUninitialized: return "UNINIT";
    case Mode::kHold: return "HOLD";
    case Mode::kRamp: return "RAMP";
    case Mode::kFollow: return "FOLLOW";
  }
  return "?";
}

inline const char* TempStateName(TempState t) {
  switch (t) {
    case TempState::kNormal: return "normal";
    case TempState::kReduced: return "REDUCED";
    case TempState::kStopped: return "STOPPED";
  }
  return "?";
}

/// Per-tick inputs, filled by the controller before Supervisor::Update().
struct TickInput {
  double now_s = 0.0;
  bool streamed = false;          ///< Reference comes from live mocap.
  uint64_t source_id = 0;         ///< Changes when the reference source changes.
  bool new_frame = false;         ///< A new reference object arrived since last tick.
  std::vector<JointArray> raw_pos;  ///< Raw window, frame 0 = current cursor (IsaacLab order).
  std::vector<Quat> raw_quat;       ///< Raw root quaternion per window frame.
  JointArray measured_q{};        ///< Measured joint positions (IsaacLab order).
  double max_motor_temp_c = 0.0;
  double deadman_age_s = std::numeric_limits<double>::infinity();
  bool heading_reinit = false;    ///< Controller is about to recapture heading.
  std::optional<int64_t> live_frame_index;
};

struct TickOutput {
  std::vector<JointArray> pos;
  std::vector<JointArray> vel;
  std::vector<Quat> quat;
};

class Supervisor {
 public:
  explicit Supervisor(const Config& cfg) : cfg_(cfg), log_(cfg.log_interval_s) {
    for (int i = 0; i < kNumJoints; ++i) {
      const auto& lim = kG1JointLimits[HwIndexOfIsaac(i)];
      urdf_lo_[i] = lim.lower;
      urdf_hi_[i] = lim.upper;
      urdf_vel_[i] = lim.velocity;
      double lo = lim.lower + cfg.reference_limit_margin_rad;
      double hi = lim.upper - cfg.reference_limit_margin_rad;
      if (lo > hi) { lo = hi = 0.5 * (lim.lower + lim.upper); }
      ref_lo_[i] = lo;
      ref_hi_[i] = hi;
    }
    in_.raw_pos.reserve(kMaxWindowFrames);
    in_.raw_quat.reserve(kMaxWindowFrames);
    out_.pos.reserve(kMaxWindowFrames);
    out_.vel.reserve(kMaxWindowFrames);
    out_.quat.reserve(kMaxWindowFrames);
    last_valid_pos_.reserve(kMaxWindowFrames);
    last_valid_quat_.reserve(kMaxWindowFrames);
  }

  TickInput& input() { return in_; }
  const TickOutput& output() const { return out_; }
  Mode mode() const { return mode_; }
  TempState temp_state() const { return temp_state_; }
  const Config& config() const { return cfg_; }
  RateLimitedLogger& logger() { return log_; }

  /// Effective limits used for the most recent tick (IsaacLab order).
  const StepLimits& limits() const { return limits_; }

  /// Run one 50 Hz tick using input(); results in output().
  void Update() {
    const double dt = cfg_.ControlDt();
    const double now = in_.now_s;

    if (mode_ == Mode::kUninitialized) { Initialize(); }

    // ---- 1. Validate and latch the newest raw window ---------------------
    const bool source_changed = in_.source_id != source_id_;
    if (source_changed) {
      source_id_ = in_.source_id;
      has_valid_ = false;
      last_live_index_.reset();
    }
    if (in_.new_frame || source_changed || !in_.streamed) {
      std::string why;
      if (ValidateWindow(why)) {
        last_valid_pos_ = in_.raw_pos;
        last_valid_quat_ = in_.raw_quat;
        for (auto& q : last_valid_quat_) { q = QuatNormalize(q); }
        last_valid_time_s_ = now;
        has_valid_ = true;
        if (in_.streamed && in_.new_frame) { ++stats_.frames; }
      } else if (in_.new_frame || source_changed) {
        ++stats_.invalid_frames;
        log_.Log("invalid", now, "Rejected invalid mocap frame (" + why + "); keeping last safe target");
      }
      TrackLiveIndex(now);
    }

    const bool fresh = has_valid_ && (!in_.streamed || (now - last_valid_time_s_) * 1000.0 <= cfg_.mocap_timeout_ms);
    if (in_.streamed && has_valid_ && !fresh && was_fresh_) {
      ++stats_.stale_events;
      log_.Log("stale", now, "Mocap stale: no valid frame for " + Ms(now - last_valid_time_s_) +
                                 " (timeout " + Ms(cfg_.mocap_timeout_ms / 1000.0) + "); smoothly stopping and holding");
    }
    was_fresh_ = fresh;

    // ---- 2. Temperature ---------------------------------------------------
    UpdateTemperature(now);

    // ---- 3. Gate ----------------------------------------------------------
    const bool deadman_ok = in_.deadman_age_s * 1000.0 <= cfg_.deadman_timeout_ms;
    if (deadman_ok != deadman_ok_) {
      std::cout << "[TeleopSafety] Dead-man " << (deadman_ok ? "ENGAGED (space held)" : "RELEASED") << std::endl;
      deadman_ok_ = deadman_ok;
    }
    const bool want_follow = deadman_ok && fresh && temp_state_ != TempState::kStopped;

    if (in_.heading_reinit && has_valid_ && !last_valid_quat_.empty()) {
      // The controller recaptures heading against the reference's current yaw.
      // Adopt that yaw immediately (keeping smoothed tilt) so the robot does not
      // try to turn toward the operator's absolute heading.
      Quat tilt = QuatMul(QuatConj(HeadingQuat(quat_)), quat_);
      quat_ = QuatNormalize(QuatMul(HeadingQuat(last_valid_quat_[0]), tilt));
    }

    UpdateLimits();

    if (want_follow) {
      if (mode_ == Mode::kHold) {
        EnterRamp(now, "dead-man engaged and mocap valid");
      } else if (source_changed) {
        EnterRamp(now, "reference source changed");
      }
    } else if (mode_ != Mode::kHold) {
      std::string reason = !deadman_ok ? "dead-man released"
                           : temp_state_ == TempState::kStopped ? "motor temperature stop"
                           : !has_valid_ ? "no valid mocap frame"
                                         : "mocap timeout";
      EnterHold(now, reason);
    }

    // ---- 4. Roll the filter across the reference window -------------------
    const int frames = (mode_ == Mode::kHold || !has_valid_) ? 1 : static_cast<int>(last_valid_pos_.size());
    out_.pos.resize(frames);
    out_.vel.resize(frames);
    out_.quat.resize(frames);

    JointFilterState rollout = state_;
    Quat rollout_quat = quat_;
    const double tau = cfg_.smoothing_time_ms / 1000.0;
    const double root_tau = cfg_.root_smoothing_time_ms / 1000.0;
    const double wmax = cfg_.max_root_angular_velocity * TempScale();
    for (int k = 0; k < frames; ++k) {
      JointArray target;
      Quat target_quat;
      TargetForFrame(k, dt, target, target_quat);
      if (k == 0) {
        StepStats st = StepFilter(state_, target, dt, tau, limits_);
        stats_.vel_saturated += st.vel_saturated;
        stats_.acc_saturated += st.acc_saturated;
        stats_.pos_clamped += st.pos_clamped;
        if (StepQuat(quat_, target_quat, dt, root_tau, wmax)) { ++stats_.root_rate_limited; }
        rollout = state_;
        rollout_quat = quat_;
        last_target0_ = target;
      } else {
        StepFilter(rollout, target, dt, tau, limits_);
        StepQuat(rollout_quat, target_quat, dt, root_tau, wmax);
      }
      out_.pos[k] = rollout.pos;
      out_.vel[k] = rollout.vel;
      out_.quat[k] = rollout_quat;
    }

    if (mode_ == Mode::kRamp) {
      ramp_elapsed_s_ += dt;
      if (ramp_elapsed_s_ >= cfg_.startup_ramp_s) {
        mode_ = Mode::kFollow;
        std::cout << "[TeleopSafety] Ramp complete -> FOLLOW" << std::endl;
      }
    }

    if (stats_.vel_saturated + stats_.acc_saturated > 0) {
      log_.Log("ref_saturation", now, "Reference limiting active: " + std::to_string(stats_.vel_saturated) +
                                          " velocity / " + std::to_string(stats_.acc_saturated) +
                                          " acceleration saturations so far");
    }
    MaybeLogStatus(now);
  }

  /// Record controller loop period (seconds) for timing diagnostics.
  void NoteLoopPeriod(double now_s, double period_s) {
    const double dt = cfg_.ControlDt();
    worst_period_s_ = std::max(worst_period_s_, period_s);
    if (period_s > cfg_.loop_overrun_factor * dt) {
      ++stats_.loop_overruns;
      log_.Log("loop_overrun", now_s, "Control loop overrun: period " + Ms(period_s) + " (nominal " + Ms(dt) + ")");
    }
  }

  void NoteOutputGuard(double now_s, const OutputGuardStats& g) {
    stats_.out_clamped += g.clamped;
    stats_.out_jump_limited += g.jump_limited;
    if (g.clamped > 0 || g.jump_limited > 0) {
      std::string name = g.worst_joint >= 0 ? kG1JointLimits[g.worst_joint].name : "?";
      log_.Log("output_guard", now_s, "Motor target guard active: " + std::to_string(g.clamped) + " clamped to URDF limits, " +
                                          std::to_string(g.jump_limited) + " jump-limited this cycle (e.g. " + name + ")");
    }
  }

  // ---- CSV diagnostics -----------------------------------------------------
  bool OpenCsv(const std::string& path) {
    csv_ = std::make_unique<std::ofstream>(path);
    if (!*csv_) {
      csv_.reset();
      return false;
    }
    auto& f = *csv_;
    f << std::setprecision(10);
    // Comment header with the effective base limits (IsaacLab order) for offline analysis.
    f << "# dt=" << cfg_.ControlDt() << " startup_ramp_s=" << cfg_.startup_ramp_s
      << " mocap_timeout_ms=" << cfg_.mocap_timeout_ms << " deadman_timeout_ms=" << cfg_.deadman_timeout_ms << "\n";
    WriteLimitRow(f, "# vmax", [this](int i) { return BaseVmax(i); });
    WriteLimitRow(f, "# amax", [this](int i) { return BaseAmax(i); });
    WriteLimitRow(f, "# ref_lo", [this](int i) { return ref_lo_[i]; });
    WriteLimitRow(f, "# ref_hi", [this](int i) { return ref_hi_[i]; });
    f << "# urdf_hw_lo";
    for (const auto& l : kG1JointLimits) { f << "," << l.lower; }
    f << "\n# urdf_hw_hi";
    for (const auto& l : kG1JointLimits) { f << "," << l.upper; }
    f << "\n";
    f << "t,mode,temp_state,deadman_ok,fresh,streamed,max_temp_c";
    for (int i = 0; i < kNumJoints; ++i) { f << ",raw" << i; }
    for (int i = 0; i < kNumJoints; ++i) { f << ",ref" << i; }
    for (int i = 0; i < kNumJoints; ++i) { f << ",vel" << i; }
    for (int i = 0; i < kNumJoints; ++i) { f << ",motor_hw" << i; }
    f << ",root_w,root_x,root_y,root_z\n";
    return true;
  }

  void WriteCsvRow(const std::array<float, kNumJoints>& motor_q_hw) {
    if (!csv_) { return; }
    auto& f = *csv_;
    f << in_.now_s << "," << ModeName(mode_) << "," << TempStateName(temp_state_) << "," << deadman_ok_ << ","
      << was_fresh_ << "," << in_.streamed << "," << in_.max_motor_temp_c;
    for (int i = 0; i < kNumJoints; ++i) { f << "," << (has_valid_ ? last_valid_pos_[0][i] : NAN); }
    for (int i = 0; i < kNumJoints; ++i) { f << "," << state_.pos[i]; }
    for (int i = 0; i < kNumJoints; ++i) { f << "," << state_.vel[i]; }
    for (int i = 0; i < kNumJoints; ++i) { f << "," << motor_q_hw[i]; }
    f << "," << quat_[0] << "," << quat_[1] << "," << quat_[2] << "," << quat_[3] << "\n";
  }

  struct Stats {
    uint64_t frames = 0;
    uint64_t invalid_frames = 0;
    uint64_t dropped_frames = 0;
    uint64_t stale_events = 0;
    uint64_t vel_saturated = 0;
    uint64_t acc_saturated = 0;
    uint64_t pos_clamped = 0;
    uint64_t root_rate_limited = 0;
    uint64_t out_clamped = 0;
    uint64_t out_jump_limited = 0;
    uint64_t loop_overruns = 0;
  };
  const Stats& stats() const { return stats_; }

 private:
  void Initialize() {
    state_.Reset(ClampRef(in_.measured_q));
    hold_target_ = state_.pos;
    quat_ = (!in_.raw_quat.empty() && FiniteQuat(in_.raw_quat[0])) ? QuatNormalize(in_.raw_quat[0]) : Quat{1, 0, 0, 0};
    mode_ = Mode::kHold;
    source_id_ = in_.source_id;
    last_status_s_ = in_.now_s;
    std::cout << "[TeleopSafety] Reference initialised from measured joint positions; HOLD until dead-man is held"
              << std::endl;
  }

  bool ValidateWindow(std::string& why) const {
    if (in_.raw_pos.empty() || in_.raw_quat.size() != in_.raw_pos.size()) {
      why = "empty window";
      return false;
    }
    const double tol = cfg_.invalid_frame_tolerance_rad;
    for (size_t k = 0; k < in_.raw_pos.size(); ++k) {
      for (int i = 0; i < kNumJoints; ++i) {
        double q = in_.raw_pos[k][i];
        if (!std::isfinite(q)) {
          why = "non-finite joint " + std::string(kG1JointLimits[HwIndexOfIsaac(i)].name);
          return false;
        }
        if (q < urdf_lo_[i] - tol || q > urdf_hi_[i] + tol) {
          std::ostringstream os;
          os << kG1JointLimits[HwIndexOfIsaac(i)].name << "=" << q << " rad far outside URDF limits";
          why = os.str();
          return false;
        }
      }
      const Quat& qq = in_.raw_quat[k];
      double n2 = qq[0] * qq[0] + qq[1] * qq[1] + qq[2] * qq[2] + qq[3] * qq[3];
      if (!FiniteQuat(qq) || n2 < 0.25 || n2 > 2.25) {
        why = "invalid root quaternion";
        return false;
      }
    }
    return true;
  }

  static bool FiniteQuat(const Quat& q) {
    return std::isfinite(q[0]) && std::isfinite(q[1]) && std::isfinite(q[2]) && std::isfinite(q[3]);
  }

  void TrackLiveIndex(double now) {
    if (!in_.streamed || !in_.new_frame || !in_.live_frame_index) { return; }
    int64_t idx = *in_.live_frame_index;
    if (last_live_index_ && idx > *last_live_index_ + 1) {
      int64_t gap = idx - *last_live_index_ - 1;
      stats_.dropped_frames += static_cast<uint64_t>(gap);
      log_.Log("dropped", now, "Dropped/skipped " + std::to_string(gap) + " mocap frame(s) (index " +
                                   std::to_string(*last_live_index_) + " -> " + std::to_string(idx) + "); using newest");
    }
    last_live_index_ = idx;
  }

  void UpdateTemperature(double now) {
    const double t = in_.max_motor_temp_c;
    if (!std::isfinite(t)) { return; }
    TempState prev = temp_state_;
    const double h = cfg_.temperature_hysteresis_c;
    if (t >= cfg_.temperature_stop_c) {
      temp_state_ = TempState::kStopped;
    } else if (temp_state_ == TempState::kStopped) {
      if (t < cfg_.temperature_stop_c - h) { temp_state_ = (t >= cfg_.temperature_reduce_c) ? TempState::kReduced : TempState::kNormal; }
    } else if (t >= cfg_.temperature_reduce_c) {
      temp_state_ = TempState::kReduced;
    } else if (temp_state_ == TempState::kReduced && t < cfg_.temperature_reduce_c - h) {
      temp_state_ = TempState::kNormal;
    }
    if (temp_state_ != prev) {
      std::ostringstream os;
      os << "Motor temperature " << t << " C: " << TempStateName(prev) << " -> " << TempStateName(temp_state_);
      if (temp_state_ == TempState::kReduced) { os << " (limits scaled by " << cfg_.temperature_reduce_factor << ")"; }
      if (temp_state_ == TempState::kStopped) { os << " (motion stopped, holding pose)"; }
      std::cout << "[TeleopSafety] " << os.str() << std::endl;
    }
    (void)now;
  }

  double TempScale() const { return temp_state_ == TempState::kNormal ? 1.0 : cfg_.temperature_reduce_factor; }
  double BaseVmax(int i) const {
    return std::min(cfg_.max_velocity_fraction * urdf_vel_[i], cfg_.max_step_per_cycle_rad / cfg_.ControlDt());
  }
  double BaseAmax(int) const { return cfg_.max_acceleration_fraction * cfg_.nominal_acceleration_limit; }

  void UpdateLimits() {
    const double scale = TempScale();
    for (int i = 0; i < kNumJoints; ++i) {
      limits_.vmax[i] = BaseVmax(i) * scale;
      limits_.amax[i] = BaseAmax(i) * scale;
      limits_.lo[i] = ref_lo_[i];
      limits_.hi[i] = ref_hi_[i];
    }
  }

  JointArray ClampRef(const JointArray& q) const {
    JointArray r;
    for (int i = 0; i < kNumJoints; ++i) {
      r[i] = std::isfinite(q[i]) ? std::clamp(q[i], ref_lo_[i], ref_hi_[i]) : state_.pos[i];
    }
    return r;
  }

  static double SmoothStep(double w) {
    w = std::clamp(w, 0.0, 1.0);
    return w * w * (3.0 - 2.0 * w);
  }

  void TargetForFrame(int k, double dt, JointArray& target, Quat& target_quat) const {
    if (mode_ == Mode::kHold || !has_valid_) {
      target = hold_target_;
      target_quat = quat_;
      return;
    }
    const JointArray raw = ClampRef(last_valid_pos_[k]);
    target_quat = last_valid_quat_[k];
    if (mode_ == Mode::kRamp) {
      const double w = SmoothStep((ramp_elapsed_s_ + k * dt) / cfg_.startup_ramp_s);
      for (int i = 0; i < kNumJoints; ++i) { target[i] = ramp_start_[i] + w * (raw[i] - ramp_start_[i]); }
    } else {
      target = raw;
    }
  }

  void EnterRamp(double now, const std::string& reason) {
    mode_ = Mode::kRamp;
    ramp_start_ = state_.pos;
    ramp_elapsed_s_ = 0.0;
    state_.lp1 = state_.pos;
    state_.lp2 = state_.pos;
    std::cout << "[TeleopSafety] RAMP over " << cfg_.startup_ramp_s << " s (" << reason << ")" << std::endl;
    (void)now;
  }

  void EnterHold(double now, const std::string& reason) {
    mode_ = Mode::kHold;
    // Stop at the natural braking point rather than snapping back.
    for (int i = 0; i < kNumJoints; ++i) {
      double v = state_.vel[i];
      double a = std::max(limits_.amax[i], 1e-6);
      hold_target_[i] = std::clamp(state_.pos[i] + v * std::abs(v) / (2.0 * a), ref_lo_[i], ref_hi_[i]);
    }
    state_.lp1 = hold_target_;
    state_.lp2 = hold_target_;
    std::cout << "[TeleopSafety] HOLD (" << reason << "): decelerating smoothly and holding last safe target" << std::endl;
    (void)now;
  }

  void MaybeLogStatus(double now) {
    if (now - last_status_s_ < cfg_.status_interval_s) { return; }
    last_status_s_ = now;
    double age_ms = has_valid_ ? (now - last_valid_time_s_) * 1000.0 : -1.0;
    std::ostringstream os;
    os << std::fixed << std::setprecision(0) << "[TeleopSafety] mode=" << ModeName(mode_)
       << " deadman=" << (deadman_ok_ ? "held" : "released") << " src=" << (in_.streamed ? "mocap" : "preloaded")
       << " frame_age=" << age_ms << "ms";
    if (mode_ == Mode::kRamp) { os << " ramp=" << 100.0 * std::min(1.0, ramp_elapsed_s_ / cfg_.startup_ramp_s) << "%"; }
    os << " temp=" << in_.max_motor_temp_c << "C(" << TempStateName(temp_state_) << ")"
       << " frames=" << stats_.frames << " dropped=" << stats_.dropped_frames << " invalid=" << stats_.invalid_frames
       << " stale=" << stats_.stale_events << " ref_sat(v/a)=" << stats_.vel_saturated << "/" << stats_.acc_saturated
       << " out_clamp=" << stats_.out_clamped << " out_jump=" << stats_.out_jump_limited
       << " overruns=" << stats_.loop_overruns << std::setprecision(1) << " worst_period=" << worst_period_s_ * 1000.0
       << "ms";
    std::cout << os.str() << std::endl;
    worst_period_s_ = 0.0;
  }

  template <typename F>
  static void WriteLimitRow(std::ofstream& f, const char* label, F value) {
    f << label;
    for (int i = 0; i < kNumJoints; ++i) { f << "," << value(i); }
    f << "\n";
  }

  static std::string Ms(double s) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(0) << s * 1000.0 << " ms";
    return os.str();
  }

  Config cfg_;
  RateLimitedLogger log_;
  TickInput in_;
  TickOutput out_;

  JointArray urdf_lo_{}, urdf_hi_{}, urdf_vel_{}, ref_lo_{}, ref_hi_{};
  StepLimits limits_;

  Mode mode_ = Mode::kUninitialized;
  TempState temp_state_ = TempState::kNormal;
  JointFilterState state_;
  Quat quat_{1.0, 0.0, 0.0, 0.0};
  JointArray hold_target_{};
  JointArray ramp_start_{};
  JointArray last_target0_{};
  double ramp_elapsed_s_ = 0.0;

  uint64_t source_id_ = 0;
  bool has_valid_ = false;
  bool was_fresh_ = false;
  bool deadman_ok_ = false;
  double last_valid_time_s_ = -1e18;
  std::vector<JointArray> last_valid_pos_;
  std::vector<Quat> last_valid_quat_;
  std::optional<int64_t> last_live_index_;

  double last_status_s_ = 0.0;
  double worst_period_s_ = 0.0;
  Stats stats_;
  std::unique_ptr<std::ofstream> csv_;
};

}  // namespace teleop_safety

#endif  // TELEOP_SAFETY_HPP
