// Copyright 2024 Brenno Domingues <brennohdomingues@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <iostream>
#include <string>
#include "AdsLib.h"
#include "AdsVariable.h"
#include "ads_hardware/ads_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <chrono>


#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>






namespace ads_hardware
{
//10.1.82.18.1.1
const AmsNetId remoteNetId { 10, 1, 82, 18, 1, 1 };
const std::string remoteIpV4 = "192.168.0.31";
AdsHandler adsHandler(remoteNetId, remoteIpV4);



constexpr const char * kADSHardware = "ADSHardware";
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;
constexpr uint8_t kPresentPositionVelocityCurrentIndex = 0;
constexpr const char * kGoalPositionItem = "Goal_Position";
constexpr const char * kGoalVelocityItem = "Goal_Velocity";
constexpr const char * kMovingSpeedItem = "Moving_Speed";
constexpr const char * kPresentPositionItem = "Present_Position";
constexpr const char * kPresentVelocityItem = "Present_Velocity";
constexpr const char * kPresentSpeedItem = "Present_Speed";
constexpr const char * kPresentCurrentItem = "Present_Current";
constexpr const char * kPresentLoadItem = "Present_Load";
constexpr const char * const kExtraJointParameters[] = {
  "Profile_Velocity",
  "Profile_Acceleration",
  "Position_P_Gain",
  "Position_I_Gain",
  "Position_D_Gain",
  "Velocity_P_Gain",
  "Velocity_I_Gain",
};



CallbackReturn ADSHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  
  

  RCLCPP_DEBUG(rclcpp::get_logger(kADSHardware), "configure");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);



//Loop para identificar quantidade de juntas e interfaces
  for (uint i = 0; i < info_.joints.size(); i++) {
    
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.acceleration = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.acceleration = std::numeric_limits<double>::quiet_NaN();
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
    joints_[i].prev_command.acceleration = joints_[i].command.acceleration; 
    RCLCPP_INFO(rclcpp::get_logger(kADSHardware), "joint_id %d: %d", i, joint_ids_[i]);
  }


//inicializacao do controlador
  
  enable_torque(false); // inicializa com o torque desligado
  set_control_mode(ControlMode::Position, true); //seta o modo para posicao
  set_joint_params(); //seta os parametros de juntas como limites etc..
  enable_torque(true);//habilita o torque




  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ADSHardware::export_state_interfaces()
{
  //RCLCPP_DEBUG(rclcpp::get_logger(kADSHardware), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &joints_[i].state.acceleration));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ADSHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &joints_[i].command.acceleration));
  }

  return command_interfaces;
}

CallbackReturn ADSHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  for (uint i = 0; i < joints_.size(); i++) {
    if (use_dummy_ && std::isnan(joints_[i].state.position)) {
      joints_[i].state.position = 0.0;
      joints_[i].state.velocity = 0.0;
      joints_[i].state.acceleration = 0.0;
      joints_[i].state.effort = 0.0;
    }
  }
  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  reset_command();
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

CallbackReturn ADSHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kADSHardware), "stop");
  return CallbackReturn::SUCCESS;
}

return_type ADSHardware::read(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    return return_type::OK;
  }

  std::vector<uint8_t> ids(info_.joints.size(), 0);
  std::vector<int32_t> positions(info_.joints.size(), 0);
  std::vector<int32_t> velocities(info_.joints.size(), 0);
  std::vector<int32_t> accelerations(info_.joints.size(), 0);
  std::vector<int32_t> currents(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  const char * log = nullptr;


  joints_[0].state.position = adsHandler.getJointPos1();
  joints_[0].state.velocity = adsHandler.getJointVel1();
  joints_[0].state.acceleration = adsHandler.getJointAcc1();
  joints_[0].state.effort = 0.0;
  joints_[1].state.position = adsHandler.getJointPos2();
  joints_[1].state.velocity = adsHandler.getJointVel2();
  joints_[1].state.acceleration = adsHandler.getJointAcc2();
  joints_[1].state.effort = 0.0;
  joints_[2].state.position = adsHandler.getJointPos3();
  joints_[2].state.velocity = adsHandler.getJointVel3();
  joints_[2].state.acceleration = adsHandler.getJointAcc3();
  joints_[2].state.effort = 0.0;
  joints_[3].state.position = adsHandler.getJointPos4();
  joints_[3].state.velocity = adsHandler.getJointVel4();
  joints_[3].state.acceleration = adsHandler.getJointAcc4();
  joints_[3].state.effort = 0.0;
  joints_[4].state.position = adsHandler.getJointPos5();
  joints_[4].state.velocity = adsHandler.getJointVel5();
  joints_[4].state.acceleration = adsHandler.getJointAcc5();
  joints_[4].state.effort = 0.0;
  joints_[5].state.position = adsHandler.getJointPos6();
  joints_[5].state.velocity = adsHandler.getJointVel6();
  joints_[5].state.acceleration = adsHandler.getJointAcc6();
  joints_[5].state.effort = 0.0;
  joints_[6].state.position = adsHandler.getJointPos7();
  joints_[6].state.velocity = adsHandler.getJointVel7();
  joints_[6].state.acceleration = adsHandler.getJointAcc7();
  joints_[6].state.effort = 0.0;

  return return_type::OK;
}

return_type ADSHardware::write(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    for (auto & joint : joints_) {
      joint.prev_command.position = joint.command.position;
      joint.state.position = joint.command.position;
    }
    return return_type::OK;
  }

  // Velocity control
  if (std::any_of(
      joints_.cbegin(), joints_.cend(), [](auto j) {
        return j.command.velocity != j.prev_command.velocity;
      }))
  {
    set_control_mode(ControlMode::Velocity);
    if (mode_changed_) {
      set_joint_params();
    }
    set_joint_velocities();
    return return_type::OK;
  }

  // Position control
  if (std::any_of(
      joints_.cbegin(), joints_.cend(), [](auto j) {
        return j.command.position != j.prev_command.position;
      }))
  {
    set_control_mode(ControlMode::Position);
    if (mode_changed_) {
      set_joint_params();
    }
    set_joint_positions();
    return return_type::OK;
  }

  // Acceleration control
  if (std::any_of(
      joints_.cbegin(), joints_.cend(), [](auto j) {
        return j.command.acceleration != j.prev_command.acceleration;
      }))
  {
    set_control_mode(ControlMode::Acceleration);
    if (mode_changed_) {
      set_joint_params();
    }
    set_joint_accelerations();
    return return_type::OK;
  }

  // Effort control
  if (std::any_of(
      joints_.cbegin(), joints_.cend(), [](auto j) {return j.command.effort != 0.0;}))
  {
    RCLCPP_ERROR(rclcpp::get_logger(kADSHardware), "Effort control is not implemented");
    return return_type::ERROR;
  }

  // If all command values are unchanged, then remain in existing control mode and set
  // corresponding command values
  switch (control_mode_) {
    case ControlMode::Velocity:
      set_joint_velocities();
      return return_type::OK;
      break;
    case ControlMode::Position:
      set_joint_positions();
      return return_type::OK;
      break;
    case ControlMode::Acceleration:
      set_joint_accelerations();
      return return_type::OK;
      break;
    default:  // effort, etc
      RCLCPP_ERROR(rclcpp::get_logger(kADSHardware), "Control mode not implemented");
      return return_type::ERROR;
      break;
  }
}

return_type ADSHardware::enable_torque(const bool enabled)
{
  const char * log = nullptr;
  torque_enabled_ = adsHandler.getTorqueStatus();

  if (enabled && !torque_enabled_) {
    adsHandler.TorqueEnable(true);
    reset_command();
    RCLCPP_INFO(rclcpp::get_logger(kADSHardware), "Torque enabled");
  } else if (!enabled && torque_enabled_) {
    adsHandler.TorqueEnable(false);
    RCLCPP_INFO(rclcpp::get_logger(kADSHardware), "Torque disabled");
  }

 
  return return_type::OK;
}

return_type ADSHardware::set_control_mode(const ControlMode & mode, const bool force_set)
{
  const char * log = nullptr;
  mode_changed_ = false;

  if (mode == ControlMode::Velocity && (force_set || control_mode_ != ControlMode::Velocity)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
    }


    RCLCPP_INFO(rclcpp::get_logger(kADSHardware), "Velocity control");
    if (control_mode_ != ControlMode::Velocity) {
      mode_changed_ = true;
      control_mode_ = ControlMode::Velocity;
    }

    if (torque_enabled) {
      enable_torque(true);
    }
    return return_type::OK;
  }

  if (mode == ControlMode::Position && (force_set || control_mode_ != ControlMode::Position)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
    }

    
    RCLCPP_INFO(rclcpp::get_logger(kADSHardware), "Position control");
    if (control_mode_ != ControlMode::Position) {
      mode_changed_ = true;
      control_mode_ = ControlMode::Position;
    }

    if (torque_enabled) {
      enable_torque(true);
    }
    return return_type::OK;
  }

  if (mode == ControlMode::Acceleration && (force_set || control_mode_ != ControlMode::Acceleration)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
    }

    
    RCLCPP_INFO(rclcpp::get_logger(kADSHardware), "Acceleration control");
    if (control_mode_ != ControlMode::Acceleration) {
      mode_changed_ = true;
      control_mode_ = ControlMode::Acceleration;
    }

    if (torque_enabled) {
      enable_torque(true);
    }
    return return_type::OK;
  }

  if (control_mode_ != ControlMode::Velocity && control_mode_ != ControlMode::Position && control_mode_ != ControlMode::Acceleration) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kADSHardware), "Only position/velocity control are implemented");
    return return_type::ERROR;
  }

  return return_type::OK;
}

return_type ADSHardware::reset_command()
{
  for (uint i = 0; i < joints_.size(); i++) {
    joints_[i].command.position = joints_[i].state.position;
    joints_[i].command.velocity = 0.0;
    joints_[i].command.acceleration = 0.0;
    joints_[i].command.effort = 0.0;
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.acceleration = joints_[i].command.acceleration;
    joints_[i].prev_command.effort = joints_[i].command.effort;
  }

  return return_type::OK;
}

CallbackReturn ADSHardware::set_joint_positions()
{
  const char * log = nullptr;
  std::vector<int32_t> commands(info_.joints.size(), 0);
  std::vector<uint8_t> ids(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());

  
  for (uint i = 0; i < ids.size(); i++) {
    joints_[i].prev_command.position = joints_[i].command.position;
    commands[i] = joints_[i].command.position;
  }
  adsHandler.SetPosition1(joints_[0].command.position);
  adsHandler.SetPosition2(joints_[1].command.position);
  adsHandler.SetPosition3(joints_[2].command.position);
  adsHandler.SetPosition4(joints_[3].command.position);
  adsHandler.SetPosition5(joints_[4].command.position);
  adsHandler.SetPosition6(joints_[5].command.position);
  adsHandler.SetPosition7(joints_[6].command.position);
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn ADSHardware::set_joint_velocities()
{
  const char * log = nullptr;
  std::vector<int32_t> commands(info_.joints.size(), 0);
  std::vector<uint8_t> ids(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());

  
  for (uint i = 0; i < ids.size(); i++) {
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    commands[i] = joints_[i].command.velocity;
  }
  adsHandler.SetVelocity1(joints_[0].command.velocity);
  adsHandler.SetVelocity2(joints_[1].command.velocity);
  adsHandler.SetVelocity3(joints_[2].command.velocity);
  adsHandler.SetVelocity4(joints_[3].command.velocity);
  adsHandler.SetVelocity5(joints_[4].command.velocity);
  adsHandler.SetVelocity6(joints_[5].command.velocity);
  adsHandler.SetVelocity7(joints_[6].command.velocity);

  return CallbackReturn::SUCCESS;
}

CallbackReturn ADSHardware::set_joint_accelerations()
{
  const char * log = nullptr;
  std::vector<int32_t> commands(info_.joints.size(), 0);
  std::vector<uint8_t> ids(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());

  
  for (uint i = 0; i < ids.size(); i++) {
    joints_[i].prev_command.acceleration = joints_[i].command.acceleration;
    commands[i] = joints_[i].command.acceleration;
  }
  adsHandler.SetAcceleration1(joints_[0].command.acceleration);
  adsHandler.SetAcceleration2(joints_[1].command.acceleration);
  adsHandler.SetAcceleration3(joints_[2].command.acceleration);
  adsHandler.SetAcceleration4(joints_[3].command.acceleration);
  adsHandler.SetAcceleration5(joints_[4].command.acceleration);
  adsHandler.SetAcceleration6(joints_[5].command.acceleration);
  adsHandler.SetAcceleration7(joints_[6].command.acceleration);

  return CallbackReturn::SUCCESS;
}

CallbackReturn ADSHardware::set_joint_params()
{
  const char * log = nullptr;
  for (uint i = 0; i < info_.joints.size(); ++i) {
    for (auto paramName : kExtraJointParameters) {
      if (info_.joints[i].parameters.find(paramName) != info_.joints[i].parameters.end()) {
        auto value = std::stoi(info_.joints[i].parameters.at(paramName));

      }
    }
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ads_hardware::ADSHardware, hardware_interface::SystemInterface)
