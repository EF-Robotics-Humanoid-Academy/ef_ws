#ifndef G1_ARM_HW_HPP_
#define G1_ARM_HW_HPP_

#include <memory>
#include <string>
#include <vector>

#include "g1_arm_control_hardware/g1_arm7_unitree.h"
#include "g1_arm_control_hardware/visibility_control.h"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"

namespace g1_platform
{
  class G1ArmPositionHardware
      : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(G1ArmPositionHardware);

    G1_ARM_CONTROL_HARDWARE_PUBLIC
    hardware_interface::return_type configure(const hardware_interface::HardwareInfo &info) override;

    G1_ARM_CONTROL_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    G1_ARM_CONTROL_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    G1_ARM_CONTROL_HARDWARE_PUBLIC
    hardware_interface::return_type start() override;

    G1_ARM_CONTROL_HARDWARE_PUBLIC
    hardware_interface::return_type stop() override;

    G1_ARM_CONTROL_HARDWARE_PUBLIC
    hardware_interface::return_type read() override;

    G1_ARM_CONTROL_HARDWARE_PUBLIC
    hardware_interface::return_type write() override;

  private:
    // Parameters for the G1
    double hw_start_sec_;
    double hw_stop_sec_;
    double hw_slowdown_;

    // Add one time startup flag
    bool startup_flag_{false};

    // Store the command for the G1 robot - now 14 DoF (7+7)
    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;

    std::array<int, 14> perm_hw_to14_{};
    std::array<int, 14> perm_14_to_hw_{};

    std::shared_ptr<ArmUnitreeControlNode> unitree_coms_;
  };

} // namespace g1_platform

#endif // G1_ARM_HW_HPP_