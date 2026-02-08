#include "g1_arm_control_hardware/g1_arm_hw.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <array>
#include <unordered_map>
#include <sstream>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

// Canonical joint-name order must match idx_left / idx_right in ArmUnitreeControlNode
static const std::array<const char*, 7> LEFT_NAMES = {
  "left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint",
  "left_elbow_joint","left_wrist_roll_joint","left_wrist_pitch_joint","left_wrist_yaw_joint"
};
static const std::array<const char*, 7> RIGHT_NAMES = {
  "right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint",
  "right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint"
};

// File-static permutations: canonical-14 <-> ros2_control (HW) order
static std::array<int, 14> g_perm_hw_to14;  // i14 -> iHW (pick from HW to build canonical 14)
static std::array<int, 14> g_perm_14_to_hw; // iHW -> i14 (place canonical 14 state into HW index)
static bool g_perm_ready = false;

inline void log_perm(const std::array<int,14>& a, const char* tag) {
  std::stringstream ss; ss << tag << ": ";
  for (int i=0;i<14;++i) { ss << a[i]; if (i+1<14) ss << ", "; }
  RCLCPP_INFO(rclcpp::get_logger("G1ArmPositionHardware"), "%s", ss.str().c_str());
}

} // anonymous namespace

namespace g1_platform
{

hardware_interface::return_type
G1ArmPositionHardware::configure(const hardware_interface::HardwareInfo &info)
{
  if (configure_default(info) != hardware_interface::return_type::OK)
    return hardware_interface::return_type::ERROR;

  hw_start_sec_ = std::stod(info_.hardware_parameters["hw_start_duration_sec"]);
  hw_stop_sec_  = std::stod(info_.hardware_parameters["hw_stop_duration_sec"]);
  hw_slowdown_  = std::stod(info_.hardware_parameters["hw_slowdown"]);

  // 14 DoF (7 left + 7 right)
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Validate interfaces
  for (const hardware_interface::ComponentInfo &joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("G1ArmPositionHardware"),
                   "Joint '%s' has %zu command interfaces; 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("G1ArmPositionHardware"),
                   "Joint '%s' command IF '%s'; '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }
    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("G1ArmPositionHardware"),
                   "Joint '%s' has %zu state interfaces; 1 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("G1ArmPositionHardware"),
                   "Joint '%s' state IF '%s'; '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }
  }

  // -------- Build name-based permutations (robust mapping) --------
  try {
    std::unordered_map<std::string, int> hw_index_by_name;
    hw_index_by_name.reserve(info_.joints.size());
    for (size_t i = 0; i < info_.joints.size(); ++i)
      hw_index_by_name.emplace(info_.joints[i].name, static_cast<int>(i));

    auto find_or_throw = [&](const std::string& n)->int {
      auto it = hw_index_by_name.find(n);
      if (it == hw_index_by_name.end())
        throw std::runtime_error("Joint name not found in ros2_control list: " + n);
      return it->second;
    };

    // Fill g_perm_hw_to14: canonical index -> HW index
    for (int j = 0; j < 7; ++j)  g_perm_hw_to14[j]     = find_or_throw(LEFT_NAMES[j]);
    for (int j = 0; j < 7; ++j)  g_perm_hw_to14[7 + j] = find_or_throw(RIGHT_NAMES[j]);

    // Inverse: HW index -> canonical index
    g_perm_14_to_hw.fill(-1);
    for (int i14 = 0; i14 < 14; ++i14) {
      const int iHW = g_perm_hw_to14[i14];
      if (iHW < 0 || iHW >= static_cast<int>(info_.joints.size()))
        throw std::runtime_error("Permutation out of HW bounds");
      g_perm_14_to_hw[iHW] = i14;
    }

    g_perm_ready = true;
    log_perm(g_perm_hw_to14, "perm_hw_to14");
    log_perm(g_perm_14_to_hw, "perm_14_to_hw");
  }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("G1ArmPositionHardware"),
                 "Failed to build joint-name permutations: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  // Initialize Unitree comms
  unitree_coms_ = std::make_shared<ArmUnitreeControlNode>();

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
G1ArmPositionHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
G1ArmPositionHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type
G1ArmPositionHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("G1ArmPositionHardware"), "Calibrating G1 Dual Arm Controller");

  for (int i = 0; i < hw_start_sec_; ++i) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("G1ArmPositionHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  }

  // Initialize states/commands
  for (size_t i = 0; i < hw_states_.size(); ++i) {
    if (std::isnan(hw_states_[i])) {
      hw_states_[i]   = 0.0;
      hw_commands_[i] = 0.0;
    } else {
      hw_commands_[i] = hw_states_[i];
    }
  }

  status_ = hardware_interface::status::STARTED;
  RCLCPP_INFO(rclcpp::get_logger("G1ArmPositionHardware"), "G1 Dual Arm System successfully started!");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
G1ArmPositionHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("G1ArmPositionHardware"), "Stopping G1 Dual Arm Controller");

  for (int i = 0; i < hw_stop_sec_; ++i) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("G1ArmPositionHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  }

  status_ = hardware_interface::status::STOPPED;

  if (unitree_coms_) unitree_coms_->arms_stop();

  RCLCPP_INFO(rclcpp::get_logger("G1ArmPositionHardware"), "System successfully stopped!");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
G1ArmPositionHardware::read()
{
  if (!g_perm_ready) {
    RCLCPP_ERROR(rclcpp::get_logger("G1ArmPositionHardware"),
                 "Permutations not ready in read(); skipping update.");
    return hardware_interface::return_type::ERROR;
  }

  // tracking_pos_: canonical [L0..L6, R0..R6]
  const auto& tp = unitree_coms_->tracking_pos_;

  // Place canonical 14 into HW order via g_perm_14_to_hw (inverse mapping)
  for (size_t iHW = 0; iHW < hw_states_.size() && iHW < 14; ++iHW) {
    const int i14 = g_perm_14_to_hw[iHW];
    if (i14 >= 0 && i14 < 14) {
      hw_states_[iHW] = static_cast<double>(tp[i14]);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
G1ArmPositionHardware::write()
{
  if (!g_perm_ready) {
    RCLCPP_ERROR(rclcpp::get_logger("G1ArmPositionHardware"),
                 "Permutations not ready in write(); aborting.");
    return hardware_interface::return_type::ERROR;
  }

  if (!startup_flag_) {
    unitree_coms_->arms_calibrate();
    rclcpp::sleep_for(std::chrono::seconds(4));
    startup_flag_ = true;
  }

  // Assemble canonical 14-command from HW order using g_perm_hw_to14
  std::array<float, 14> q14{};
  for (int i14 = 0; i14 < 14 && i14 < static_cast<int>(hw_commands_.size()); ++i14) {
    const int iHW = g_perm_hw_to14[i14];
    if (iHW >= 0 && iHW < static_cast<int>(hw_commands_.size())) {
      q14[i14] = static_cast<float>(hw_commands_[iHW]);
    }
  }

  unitree_coms_->arms_move_unrestricted(q14);
  return hardware_interface::return_type::OK;
}

} // namespace g1_platform

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(g1_platform::G1ArmPositionHardware, hardware_interface::SystemInterface)
