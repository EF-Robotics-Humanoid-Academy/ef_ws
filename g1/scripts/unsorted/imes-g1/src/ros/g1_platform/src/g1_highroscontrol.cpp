/*
@author    Salman Omar Sohail <support@mybotshop.de>
@copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of MYBOTSHOP GmbH nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

Redistribution and use in source and binary forms, with or without
modification, is not permitted without the express permission
of MYBOTSHOP GmbH.
*/

#include <chrono>
#include <iostream>
#include <thread>
#include <sstream>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "g1_interface/srv/g1_modes.hpp"

#include "unitree/robot/g1/loco/g1_loco_client.hpp"
#include "utils/colors.h"

// Helper function to parse space-separated floats
std::vector<float> stringToFloatVector(const std::string &str)
{
  std::vector<float> result;
  std::stringstream ss(str);
  float num;
  while (ss >> num)
  {
    result.push_back(num);
    ss.ignore();
  }
  return result;
}

class G1HighLevelRequest : public rclcpp::Node
{
public:
  G1HighLevelRequest(std::shared_ptr<unitree::robot::g1::LocoClient> client,
                     const std::string &eth_interface)
      : Node("g1_highroscontrol"), client_(client)
  {
    RCLCPP_INFO(this->get_logger(), "Using Ethernet interface: %s", eth_interface.c_str());
    RCLCPP_INFO(this->get_logger(), "\x1B[32mInitializing G1 Highlevel ROS Leg Hardware Communication\x1B[0m");

    sub_g1_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        "hardware/cmd_vel", 1, std::bind(&G1HighLevelRequest::callback_g1_cmd, this, std::placeholders::_1));

    srv_g1_modes = this->create_service<g1_interface::srv::G1Modes>(
        "hardware_modes", std::bind(&G1HighLevelRequest::callback_g1_modes, this,
                                    std::placeholders::_1, std::placeholders::_2));

    printAllModes();
  }

private:
  void callback_g1_cmd(geometry_msgs::msg::Twist::SharedPtr msg)
  {
    client_->Move(msg->linear.x, msg->linear.y, msg->angular.z);
  }

  void callback_g1_modes(const std::shared_ptr<g1_interface::srv::G1Modes::Request> request,
                         std::shared_ptr<g1_interface::srv::G1Modes::Response> response)
  {
    try
    {
      RCLCPP_INFO(get_logger(), BOLD(FORA("Service request: ")) FBLU("%s"), request->request_data.c_str());

      // Assume "key=value" format
      std::string input = request->request_data;
      size_t eq_pos = input.find('=');
      std::string key = (eq_pos != std::string::npos) ? input.substr(0, eq_pos) : input;
      std::string value = (eq_pos != std::string::npos) ? input.substr(eq_pos + 1) : "";

      bool success = processCommand(key, value);

      response->success = success;
      response->reason = success ? "Command executed successfully" : "Failed to execute command";
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Exception in callback_g1_modes: %s", e.what());
      response->success = false;
      response->reason = e.what();
    }
  }

  bool processCommand(const std::string &cmd, const std::string &param)
  {
    // Getters
    if (cmd == "get_fsm_id")
    {
      int fsm_id;
      client_->GetFsmId(fsm_id);
      RCLCPP_INFO(get_logger(), "Current fsm_id: %d", fsm_id);
      return true;
    }
    if (cmd == "get_fsm_mode")
    {
      int mode;
      client_->GetFsmMode(mode);
      RCLCPP_INFO(get_logger(), "Current fsm_mode: %d", mode);
      return true;
    }
    if (cmd == "get_balance_mode")
    {
      int mode;
      client_->GetBalanceMode(mode);
      RCLCPP_INFO(get_logger(), "Current balance_mode: %d", mode);
      return true;
    }
    if (cmd == "get_swing_height")
    {
      float val;
      client_->GetSwingHeight(val);
      RCLCPP_INFO(get_logger(), "Current swing_height: %.2f", val);
      return true;
    }
    if (cmd == "get_stand_height")
    {
      float val;
      client_->GetStandHeight(val);
      RCLCPP_INFO(get_logger(), "Current stand_height: %.2f", val);
      return true;
    }
    if (cmd == "get_phase")
    {
      std::vector<float> phase;
      client_->GetPhase(phase);
      std::ostringstream oss;
      for (auto &p : phase)
        oss << p << " ";
      RCLCPP_INFO(get_logger(), "Current phase: %s", oss.str().c_str());
      return true;
    }

    // Setters
    if (cmd == "set_fsm_id")
    {
      client_->SetFsmId(std::stoi(param));
      return true;
    }
    if (cmd == "set_balance_mode")
    {
      client_->SetBalanceMode(std::stoi(param));
      return true;
    }
    if (cmd == "set_swing_height")
    {
      client_->SetSwingHeight(std::stof(param));
      return true;
    }
    if (cmd == "set_stand_height")
    {
      client_->SetStandHeight(std::stof(param));
      return true;
    }
    if (cmd == "set_velocity")
    {
      auto vals = stringToFloatVector(param);
      if (vals.size() == 3)
        client_->SetVelocity(vals[0], vals[1], vals[2], 1.0f);
      else if (vals.size() == 4)
        client_->SetVelocity(vals[0], vals[1], vals[2], vals[3]);
      else
        return false;
      return true;
    }
    if (cmd == "move")
    {
      auto vals = stringToFloatVector(param);
      if (vals.size() == 3)
        client_->Move(vals[0], vals[1], vals[2]);
      else
        return false;
      return true;
    }
    if (cmd == "set_task_id")
    {
      client_->SetTaskId(std::stoi(param));
      return true;
    }
    if (cmd == "set_speed_mode")
    {
      client_->SetSpeedMode(std::stoi(param));
      return true;
    }

    // Actions
    if (cmd == "damp")
      return client_->Damp(), true;
    if (cmd == "start")
      return client_->Start(), true;
    if (cmd == "squat")
      return client_->Squat(), true;
    if (cmd == "sit")
      return client_->Sit(), true;
    if (cmd == "stand_up")
      return client_->StandUp(), true;
    if (cmd == "zero_torque")
      return client_->ZeroTorque(), true;
    if (cmd == "stop_move")
      return client_->StopMove(), true;
    if (cmd == "high_stand")
      return client_->HighStand(), true;
    if (cmd == "low_stand")
      return client_->LowStand(), true;
    if (cmd == "balance_stand")
      return client_->BalanceStand(), true;
    if (cmd == "shake_hand")
    {
      client_->ShakeHand(0);
      std::this_thread::sleep_for(std::chrono::seconds(10));
      client_->ShakeHand(1);
      return true;
    }
    if (cmd == "wave_hand")
      return client_->WaveHand(), true;
    if (cmd == "wave_hand_with_turn")
      return client_->WaveHand(true), true;

    // Toggles
    if (cmd == "continous_gait")
    {
      if (param == "true")
        client_->ContinuousGait(true);
      else if (param == "false")
        client_->ContinuousGait(false);
      else
        return false;
      return true;
    }
    if (cmd == "switch_move_mode")
    {
      if (param == "true")
        client_->SwitchMoveMode(true);
      else if (param == "false")
        client_->SwitchMoveMode(false);
      else
        return false;
      return true;
    }

    RCLCPP_WARN(this->get_logger(), "Unknown command: %s", cmd.c_str());
    return false;
  }

  void printAllModes()
  {
    RCLCPP_INFO(get_logger(), "Supported commands: get_fsm_id, get_fsm_mode, get_balance_mode, get_swing_height, "
                              "get_stand_height, get_phase, set_fsm_id, set_balance_mode, set_swing_height, "
                              "set_stand_height, set_velocity, move, set_task_id, set_speed_mode, damp, start, "
                              "squat, sit, stand_up, zero_torque, stop_move, high_stand, low_stand, balance_stand, "
                              "shake_hand, wave_hand, wave_hand_with_turn, continous_gait, switch_move_mode");
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_g1_cmd_vel;
  rclcpp::Service<g1_interface::srv::G1Modes>::SharedPtr srv_g1_modes;
  std::shared_ptr<unitree::robot::g1::LocoClient> client_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto temp_node = std::make_shared<rclcpp::Node>("temp_param_node");
  temp_node->declare_parameter<std::string>("eth", "none");
  std::string eth_interface = temp_node->get_parameter("eth").as_string();
  temp_node.reset();

  unitree::robot::ChannelFactory::Instance()->Init(0, eth_interface);

  auto client = std::make_shared<unitree::robot::g1::LocoClient>();
  client->Init();
  client->SetTimeout(10.0f);

  auto node = std::make_shared<G1HighLevelRequest>(client, eth_interface);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
