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



#ifndef ADS_HARDWARE__ADS_HARDWARE_HPP_
#define ADS_HARDWARE__ADS_HARDWARE_HPP_

#include "AdsLib.h"
#include "AdsVariable.h"

#include <map>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "ads_hardware/visiblity_control.h"
#include "rclcpp/macros.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;



namespace ads_hardware
{
  
struct AdsVariables
{
    AdsVariables() = delete;

    explicit AdsVariables(AdsDevice& route)
        : activateMotion{route, "MAIN.bActivateMotion"}
        , SetPosition1{route, "MAIN.SetPos[1]"}
        , GetPosition1{route, "MAIN.ActPos[1]"}
        , SetPosition2{route, "MAIN.SetPos[2]"}
        , GetPosition2{route, "MAIN.ActPos[2]"}
        , SetPosition3{route, "MAIN.SetPos[3]"}
        , GetPosition3{route, "MAIN.ActPos[3]"}
        , SetPosition4{route, "MAIN.SetPos[4]"}
        , GetPosition4{route, "MAIN.ActPos[4]"}
        , SetPosition5{route, "MAIN.SetPos[5]"}
        , GetPosition5{route, "MAIN.ActPos[5]"}
        , SetPosition6{route, "MAIN.SetPos[6]"}
        , GetPosition6{route, "MAIN.ActPos[6]"}
        , SetPosition7{route, "MAIN.SetPos[7]"}
        , GetPosition7{route, "MAIN.ActPos[7]"}

        , SetVelocity1{route, "MAIN.SetVel[1]"}
        , GetVelocity1{route, "MAIN.ActVel[1]"}
        , SetVelocity2{route, "MAIN.SetVel[2]"}
        , GetVelocity2{route, "MAIN.ActVel[2]"}
        , SetVelocity3{route, "MAIN.SetVel[3]"}
        , GetVelocity3{route, "MAIN.ActVel[3]"}
        , SetVelocity4{route, "MAIN.SetVel[4]"}
        , GetVelocity4{route, "MAIN.ActVel[4]"}
        , SetVelocity5{route, "MAIN.SetVel[5]"}
        , GetVelocity5{route, "MAIN.ActVel[5]"}
        , SetVelocity6{route, "MAIN.SetVel[6]"}
        , GetVelocity6{route, "MAIN.ActVel[6]"}
        , SetVelocity7{route, "MAIN.SetVel[7]"}
        , GetVelocity7{route, "MAIN.ActVel[7]"}

        , SetAcceleration1{route, "MAIN.SetAcc[1]"}
        , GetAcceleration1{route, "MAIN.ActAcc[1]"}
        , SetAcceleration2{route, "MAIN.SetAcc[2]"}
        , GetAcceleration2{route, "MAIN.ActAcc[2]"}
        , SetAcceleration3{route, "MAIN.SetAcc[3]"}
        , GetAcceleration3{route, "MAIN.ActAcc[3]"}
        , SetAcceleration4{route, "MAIN.SetAcc[4]"}
        , GetAcceleration4{route, "MAIN.ActAcc[4]"}
        , SetAcceleration5{route, "MAIN.SetAcc[5]"}
        , GetAcceleration5{route, "MAIN.ActAcc[5]"}
        , SetAcceleration6{route, "MAIN.SetAcc[6]"}
        , GetAcceleration6{route, "MAIN.ActAcc[6]"}
        , SetAcceleration7{route, "MAIN.SetAcc[7]"}
        , GetAcceleration7{route, "MAIN.ActAcc[7]"}
    {
        // Do nothing.
    }

    AdsVariable<bool> activateMotion;
    AdsVariable<double> SetPosition1;
    AdsVariable<double> GetPosition1;
    AdsVariable<double> SetPosition2;
    AdsVariable<double> GetPosition2;
    AdsVariable<double> SetPosition3;
    AdsVariable<double> GetPosition3;
    AdsVariable<double> SetPosition4;
    AdsVariable<double> GetPosition4;
    AdsVariable<double> SetPosition5;
    AdsVariable<double> GetPosition5;
    AdsVariable<double> SetPosition6;
    AdsVariable<double> GetPosition6;
    AdsVariable<double> SetPosition7;
    AdsVariable<double> GetPosition7;

    AdsVariable<double> SetVelocity1;
    AdsVariable<double> GetVelocity1;
    AdsVariable<double> SetVelocity2;
    AdsVariable<double> GetVelocity2;
    AdsVariable<double> SetVelocity3;
    AdsVariable<double> GetVelocity3;
    AdsVariable<double> SetVelocity4;
    AdsVariable<double> GetVelocity4;
    AdsVariable<double> SetVelocity5;
    AdsVariable<double> GetVelocity5;
    AdsVariable<double> SetVelocity6;
    AdsVariable<double> GetVelocity6;
    AdsVariable<double> SetVelocity7;
    AdsVariable<double> GetVelocity7;

    AdsVariable<double> SetAcceleration1;
    AdsVariable<double> GetAcceleration1;
    AdsVariable<double> SetAcceleration2;
    AdsVariable<double> GetAcceleration2;
    AdsVariable<double> SetAcceleration3;
    AdsVariable<double> GetAcceleration3;
    AdsVariable<double> SetAcceleration4;
    AdsVariable<double> GetAcceleration4;
    AdsVariable<double> SetAcceleration5;
    AdsVariable<double> GetAcceleration5;
    AdsVariable<double> SetAcceleration6;
    AdsVariable<double> GetAcceleration6;
    AdsVariable<double> SetAcceleration7;
    AdsVariable<double> GetAcceleration7;
};

class AdsHandler{
  public:
    explicit AdsHandler(const AmsNetId remoteNetId, const std::string remoteIpV4)
      : remoteNetId_(remoteNetId)
      , remoteIpV4_(remoteIpV4)
      , route_{remoteIpV4_, remoteNetId_, AMSPORT_R0_PLC_TC3}
      , ads_(route_) { }

    //AdsHandler() : AdsHandler({172, 20, 0, 2,  1, 1}, "172.19.0.2") { }
    AdsHandler() : AdsHandler({10, 1, 82, 18,  1, 1}, "192.168.0.31") { }


    
    void TorqueEnable(bool value)
    {
        ads_.activateMotion = value;
    }
    bool getTorqueStatus()
    {
        return ads_.activateMotion;
    }

    double getJointPos1()
    {
        return ads_.GetPosition1 * 0.01745;
    }
    double getJointPos2()
    {
        return ads_.GetPosition2 * 0.01745;
    }
    double getJointPos3()
    {
        return ads_.GetPosition3 * 0.01745;
    }
    double getJointPos4()
    {
        return ads_.GetPosition4 * 0.01745;
    }
    double getJointPos5()
    {
        return ads_.GetPosition5 * 0.01745;
    }
    double getJointPos6()
    {
        return ads_.GetPosition6 * 0.01745;
    }
    double getJointPos7()
    {
        return ads_.GetPosition7 * 0.01745;
    }
    void SetPosition1(double value)
    {
        ads_.SetPosition1 = value * 57.2958;
    }
    void SetPosition2(double value)
    {
        ads_.SetPosition2 = value * 57.2958;
    }
    void SetPosition3(double value)
    {
        ads_.SetPosition3 = value * 57.2958;
    }
    void SetPosition4(double value)
    {
        ads_.SetPosition4 = value * 57.2958;
    }
    void SetPosition5(double value)
    {
        ads_.SetPosition5 = value * 57.2958;
    }
    void SetPosition6(double value)
    {
        ads_.SetPosition6 = value * 57.2958;
    }
    void SetPosition7(double value)
    {
        ads_.SetPosition7 = value * 57.2958;
    }

    double getJointVel1()
    {
        return ads_.GetVelocity1 * 0.01745;
    }
    double getJointVel2()
    {
        return ads_.GetVelocity2 * 0.01745;
    }
    double getJointVel3()
    {
        return ads_.GetVelocity3 * 0.01745;
    }
    double getJointVel4()
    {
        return ads_.GetVelocity4 * 0.01745;
    }
    double getJointVel5()
    {
        return ads_.GetVelocity5 * 0.01745;
    }
    double getJointVel6()
    {
        return ads_.GetVelocity6 * 0.01745;
    }
    double getJointVel7()
    {
        return ads_.GetVelocity7 * 0.01745;
    }
    void SetVelocity1(double value)
    {
        ads_.SetVelocity1 = value * 57.2958;
    }
    void SetVelocity2(double value)
    {
        ads_.SetVelocity2 = value * 57.2958;
    }
    void SetVelocity3(double value)
    {
        ads_.SetVelocity3 = value * 57.2958;
    }
    void SetVelocity4(double value)
    {
        ads_.SetVelocity4 = value * 57.2958;
    }
    void SetVelocity5(double value)
    {
        ads_.SetVelocity5 = value * 57.2958;
    }
    void SetVelocity6(double value)
    {
        ads_.SetVelocity6 = value * 57.2958;
    }
    void SetVelocity7(double value)
    {
        ads_.SetVelocity7 = value * 57.2958;
    }

    double getJointAcc1()
    {
        return ads_.GetAcceleration1 * 0.01745;
    }
    double getJointAcc2()
    {
        return ads_.GetAcceleration2 * 0.01745;
    }
    double getJointAcc3()
    {
        return ads_.GetAcceleration3 * 0.01745;
    }
    double getJointAcc4()
    {
        return ads_.GetAcceleration4 * 0.01745;
    }
    double getJointAcc5()
    {
        return ads_.GetAcceleration5 * 0.01745;
    }
    double getJointAcc6()
    {
        return ads_.GetAcceleration6 * 0.01745;
    }
    double getJointAcc7()
    {
        return ads_.GetAcceleration7 * 0.01745;
    }

    void SetAcceleration1(double value)
    {
        ads_.SetAcceleration1 = value * 57.2958;
    }
    void SetAcceleration2(double value)
    {
        ads_.SetAcceleration2 = value * 57.2958;
    }
    void SetAcceleration3(double value)
    {
        ads_.SetAcceleration3 = value * 57.2958;
    }
    void SetAcceleration4(double value)
    {
        ads_.SetAcceleration4 = value * 57.2958;
    }
    void SetAcceleration5(double value)
    {
        ads_.SetAcceleration5 = value * 57.2958;
    }
    void SetAcceleration6(double value)
    {
        ads_.SetAcceleration6 = value * 57.2958;
    }
    void SetAcceleration7(double value)
    {
        ads_.SetAcceleration7 = value * 57.2958;
    }
  private:
    const AmsNetId remoteNetId_;
    const std::string remoteIpV4_;
    AdsDevice route_;
    AdsVariables ads_;
};
struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  double effort{0.0};
};

struct Joint
{
  JointValue state{};
  JointValue command{};
  JointValue prev_command{};
};

enum class ControlMode
{
  Position,
  Velocity,
  Acceleration,
  Torque,
  Currrent,
  ExtendedPosition,
  MultiTurn,
  CurrentBasedPosition,
  PWM,
};

class ADSHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ADSHardware)

  ADS_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ADS_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ADS_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ADS_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ADS_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ADS_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ADS_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  return_type enable_torque(const bool enabled);

  return_type set_control_mode(const ControlMode & mode, const bool force_set = false);

  return_type reset_command();

  CallbackReturn set_joint_positions();
  CallbackReturn set_joint_velocities();
  CallbackReturn set_joint_accelerations();
  CallbackReturn set_joint_params();

//  ads_workbench_ ads_workbench_;
  //std::map<const char * const, const ControlItem *> control_items_;
  std::vector<Joint> joints_;
  std::vector<uint8_t> joint_ids_;
  bool torque_enabled_{false};
  ControlMode control_mode_{ControlMode::Position};
  bool mode_changed_{false};
  bool use_dummy_{false};

 
};


}  // namespace ads_hardware

#endif  // ADS_HARDWARE__ADS_HARDWARE_HPP_
