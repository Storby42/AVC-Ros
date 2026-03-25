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
#include "avc_car/pico_structs.hpp"


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
SerialTransfer::SerialTransfer* PicoTransfer;
// TODO: Move this stuff to the avc_car_system header file

hardware_interface::CallbackReturn avc_carSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(get_logger(), info_.hardware_parameters["loop_rate"].c_str());
  
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.avc_car"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // Check if the number of joints is correct based on the mode of operation
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      get_logger(),
      "avc_carSystemHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("front") != std::string::npos;
    bool joint_is_traction = joint.name.find("rear") != std::string::npos;


    // Steering joints have a position command interface and a position state interface
    if (joint_is_steering)
    {
      RCLCPP_INFO(get_logger(), "Joint '%s' is a steering joint.", joint.name.c_str());

      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else if(joint_is_traction)
    {
      RCLCPP_INFO(get_logger(), "Joint '%s' is a drive joint.", joint.name.c_str());

      // Drive joints have a velocity command interface and a velocity state interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

    //Get sensor state interfaces
  for (const hardware_interface::ComponentInfo & sensor : info_.sensors)
  {
    if (sensor.name.find("fused_odom") != std::string::npos)
    {
      pose_sensor_ = sensor.name;
    }
  }



  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production
  // code
  hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  hw_interfaces_["steering"] = Joint("virtual_front_wheel_joint");

  hw_interfaces_["traction"] = Joint("virtual_rear_wheel_joint");


  servo_vel = std::stof(info_.hardware_parameters["servo_vel"]);
  servo_offset = std::stof(info_.hardware_parameters["servo_offset"]);
  traction_cal = std::stof(info_.hardware_parameters["traction_cal"]);
  steering_cal = std::stof(info_.hardware_parameters["steering_cal"]);
  wheel_radius = std::stof(info_.hardware_parameters["wheel_radius"]);



  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> avc_carSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.second.joint_name, hardware_interface::HW_IF_POSITION, &joint.second.state.position));

    if (joint.first == "traction")
    {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
          &joint.second.state.velocity));
    }
  }

  RCLCPP_INFO(get_logger(), "Exported %zu state interfaces.", state_interfaces.size());

  for (auto s : state_interfaces)
  {
    RCLCPP_INFO(get_logger(), "Exported state interface '%s'.", s.get_name().c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
avc_carSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    if (joint.first == "steering")
    {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint.second.joint_name, hardware_interface::HW_IF_POSITION,
          &joint.second.command.position));
    }
    else if (joint.first == "traction")
    {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
          &joint.second.command.velocity));
    }
  }

  RCLCPP_INFO(get_logger(), "Exported %zu command interfaces.", command_interfaces.size());

  for (auto i = 0u; i < command_interfaces.size(); i++)
  {
    RCLCPP_INFO(
      get_logger(), "Exported command interface '%s'.", command_interfaces[i].get_name().c_str());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn avc_carSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }

  for (auto & joint : hw_interfaces_)
  {
    joint.second.state.position = 0.0;

    if (joint.first == "traction")
    {
      joint.second.state.velocity = 0.0;
      joint.second.command.velocity = 0.0;
    }

    else if (joint.first == "steering")
    {
      joint.second.command.position = 0.0;
    }
  }

  
  PicoTransfer = new SerialTransfer::SerialTransfer(info_.hardware_parameters["device"].c_str(), std::stoi(info_.hardware_parameters["baud_rate"]));
  rclcpp::Time current_time = get_clock()->now();
  prev_time = current_time.seconds();



  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn avc_carSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

    
  PicoTransfer = nullptr;


  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");



  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type avc_carSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

    // set_state(
  //   steering_joint_ + "/" + hardware_interface::HW_IF_POSITION,
  //   get_command(steering_joint_ + "/" + hardware_interface::HW_IF_POSITION));

  // set_state(
  //   traction_joint_ + "/" + hardware_interface::HW_IF_VELOCITY,
  //   get_command(traction_joint_ + "/" + hardware_interface::HW_IF_VELOCITY));
  // set_state(
  //   traction_joint_ + "/" + hardware_interface::HW_IF_POSITION,
  //   get_state(traction_joint_ + "/" + hardware_interface::HW_IF_POSITION) +
  //     get_command(traction_joint_ + "/" + hardware_interface::HW_IF_VELOCITY) * period.seconds());
  
  // ----------------------  receive state data from pico ---------------------- 
  // Measure how long the receive + state update section takes.
  auto section_start = std::chrono::high_resolution_clock::now();

  if (PicoTransfer -> available())
  {
    uint16_t recSizePico = 0;
    recSizePico = PicoTransfer -> rxObj(odometry, recSizePico);
  }

  // calc approximated servo position (not given by pico)
  rclcpp::Time current_time = get_clock()->now();
  double deltaTime = current_time.seconds() - prev_time;

  float diff = jetsonCommands.steering_angle - sim_servo_pos;
  float direction = 0.0f;
  if (diff != 0.0f) {
    direction = diff / std::fabs(diff);
  }
  float step = std::min(static_cast<float>(std::fabs(diff)), static_cast<float>(deltaTime * servo_vel));
  sim_servo_pos += direction * step;




  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

  hw_interfaces_["steering"].state.position = (((static_cast<double>(sim_servo_pos))-servo_offset)/steering_cal);

  hw_interfaces_["traction"].state.velocity = ((static_cast<double>(odometry.motorVelocity))/traction_cal);
  hw_interfaces_["traction"].state.position = ((static_cast<double>(odometry.motorPosition))/traction_cal);

  //PORT ME!!!
    auto section_end = std::chrono::high_resolution_clock::now();
  auto section_us = std::chrono::duration_cast<std::chrono::microseconds>(section_end - section_start).count();
  
  // TODO: do quat bullshit here
  // funni reverse radius thing with traction_cal and wheel_radius
  // Publish odometry to sensor state interfaces (only touch sensors that exist).
  
  pose_sensor_ + "/position.x", static_cast<double>(odometry.x) / traction_cal);
  set_state(pose_sensor_ + "/position.y", static_cast<double>(odometry.y) / traction_cal);
  // If the Pico provides a z value, replace the 0.0 below with odometry.z
  set_state(pose_sensor_ + "/position.z", 0.0);

  // Publish a default (identity) quaternion for orientation sensors unless you
  // have a yaw/quat from the Pico (then compute/use that instead).
  set_state(pose_sensor_ + "/orientation.x", 0.0);
  set_state(pose_sensor_ + "/orientation.y", 0.0);
  set_state(pose_sensor_ + "/orientation.z", static_cast<double>(sin(odometry.theta/2)));
  set_state(pose_sensor_ + "/orientation.w", static_cast<double>(cos(odometry.theta/2)));
  
  
  double section_ms = static_cast<double>(section_us) / 1000.0;
  
  // Throttle the timing log to avoid flooding the console.
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "avc_car_system: odometry rx + state update took %.3f ms (%lld us)",
    section_ms, (long long)section_us);
  //PORT ME!!!!


  std::stringstream ss;
  ss << "Reading states:";

  ss << std::fixed << std::setprecision(2) << std::endl
     << "\t"
     << "position: " << hw_interfaces_["steering"].state.position << " for joint '"
     << hw_interfaces_["steering"].joint_name.c_str() << "'" << std::endl
     << "\t"
     << "position: " << hw_interfaces_["traction"].state.position << " for joint '"
     << hw_interfaces_["traction"].joint_name.c_str() << "'" << std::endl
     << "\t"
     << "velocity: " << hw_interfaces_["traction"].state.velocity << " for joint '"
     << hw_interfaces_["traction"].joint_name.c_str() << "'";

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());


  

  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type avc_car ::avc_carSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Writing commands:";

  ss << std::fixed << std::setprecision(2) << std::endl
     << "\t"
     << "position: " << hw_interfaces_["steering"].command.position << " for joint '"
     << hw_interfaces_["steering"].joint_name.c_str() << "'" << std::endl
     << "\t"
     << "velocity: " << hw_interfaces_["traction"].command.velocity << " for joint '"
     << hw_interfaces_["traction"].joint_name.c_str() << "'";

  //PORT ME!!!
    // ---------------------- send data to pico ----------------------
  // use this variable to keep track of how many bytes we're stuffing in the transmit buffer
  jetsonCommands.onoff = false;
  jetsonCommands.steering_angle = (static_cast<float>((hw_interfaces_["steering"].command.position)*steering_cal)+servo_offset);
  jetsonCommands.target_velocity = (static_cast<float>(hw_interfaces_["traction"].command.velocity)*traction_cal);

  uint16_t sendSizeCmd = 0;
  // Stuff buffer with struct
  sendSizeCmd = PicoTransfer -> txObj(jetsonCommands, sendSizeCmd);
  // Send buffer
  PicoTransfer -> sendData(sendSizeCmd);
  //PORT ME!!!

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace avc_car

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  avc_car::avc_carSystemHardware, hardware_interface::SystemInterface)
