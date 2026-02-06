// Copyright 2021 ros2_control Development Team
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

#include "avc_car/avc_car_system.hpp"
#include "avc_car/SerialTransfer.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace avc_car
{
hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info) 
  // HardwareInfo (udrf/ros2_control_bic.xacro) is where we can retrieve all the parameters

{
  // If the hardware interface doesn't connect with the system interface, return an error  
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // corresponds to parameters in udrf/ros2_control_bic.xacro
  // cfg_ and all the .rear_wheel_name, .front_wheel_name, etc. are declared as variables in carlikebot_system.hpp
  // cfg_ is like an object of CarlikeBotSystemHardware

  // std::stof converts string to float (this need to be done to any number)
  // and you just change the last letter to whatever data type (e.g. f for float, d for double, etc.)
  cfg_.rear_wheel_name = info_.hardware_parameters["rear_wheel_name"];
  cfg_.front_wheel_name = info_.hardware_parameters["front_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

  // calling the setup function in robot_description/hardware/include/bicdrive_arduino/steering.hpp and traction.hpp
  // traction_ and steering_ are objects of the created Traction and Steering classes, which have functions setup

  // basically CONNECTING the joint declared in the udrf file to the PHYSICAL joint 
  // traction_.setup(cfg_.rear_wheel_name); 
  // steering_.setup(cfg_.front_wheel_name);

  // Check if the number of joints is correct based on the mode of operation
  // we want 2: 1 for steering, 1 for traction (like a bicycle!)
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("CarlikeBotSystemHardware"),
      "CarlikeBotSystemHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  // checking if the command and state interfaces are correct
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("front") != std::string::npos;

    // Steering joints have a position command interface and a position state interface
    if (joint_is_steering)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' is a steering joint.",
        joint.name.c_str());

      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' is a drive joint.",
        joint.name.c_str());

      // Drive joints have a velocity command interface and a velocity state interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}
// declares the state interfaces so rest of ros2_control knows what to read 
std::vector<hardware_interface::StateInterface> CarlikeBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      steering_.name, hardware_interface::HW_IF_POSITION, &steering_.pos));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      traction_.name, hardware_interface::HW_IF_VELOCITY, &traction_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      traction_.name, hardware_interface::HW_IF_POSITION, &traction_.pos));
    
    RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "State interfaces exported");

  return state_interfaces;
}

// declares the command interfaces so rest of ros2_control knows what to write to
// this is where 
std::vector<hardware_interface::CommandInterface>
CarlikeBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    steering_.name, hardware_interface::HW_IF_POSITION,
    &steering_.cmd)); //CHANGED from o.g. code to .cmd because this is overwriting the state interface .pos

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    traction_.name, hardware_interface::HW_IF_VELOCITY,
    &traction_.cmd));

  RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Command interfaces exported");

  return command_interfaces;
}
// testing if the jetson is connected to the pico right
// TODO: replace with new comms
hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

  serial_transfer_ = new SerialTransfer()
  
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// disconnecting jetson from pico
// TODO: replace with new comms
hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}
//TODO: however the new comms thing is initialized/connected
hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Activating ...please wait...");

  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

//TODO: however the new comms thing is ended/disconnected
hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Deactivating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}
//TODO: read stuff using the comms function
// will need to at least read encoder for joint state interface probably if we're gonna do that 
hardware_interface::return_type CarlikeBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }
  int num = 0;
  comms_.read_encoder_value(num);
  // TODO: create 3 read functions to update the 3 values below
  steering_.pos = 0.0;
  traction_.vel = 0.0;
  traction_.pos = 0.0;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type bicdrive_arduino ::CarlikeBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }
  //steering_.pos = M_PI_2 + steering_.pos;
  //double steering_pos_deg = steering_.pos * (180.0 / M_PI);
  // std::cout << "Steering pos is " << steering_.pos << std::endl;

  double steering_pos_deg = steering_.cmd * (180.0/M_PI);
  double max_steering_angle = 180.0; 
  double min_steering_angle = -180.0; 
  double clamped_steering_angle = std::max(min_steering_angle, std::min(steering_pos_deg, max_steering_angle));
  // if (steering_.pos < -10.0) {
  //   steering_pos_deg = -180.0;
  // } 
  // else if (steering_.pos > 10.0) {
  //   steering_pos_deg = 180.0;
  // }

  double max_velocity = 1000.0;
  double min_velocity = -1000.0;

  double clamped_velocity = std::max(min_velocity, std::min(traction_.cmd, max_velocity));
  clamped_velocity = clamped_velocity / 3; 
  
  comms_.set_steering_motor_value(steering_pos_deg); // change this to clamped_steer
  comms_.set_drive_motor_value(clamped_velocity);

  return hardware_interface::return_type::OK;
}

}  // namespace bicdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  bicdrive_arduino::CarlikeBotSystemHardware, hardware_interface::SystemInterface)