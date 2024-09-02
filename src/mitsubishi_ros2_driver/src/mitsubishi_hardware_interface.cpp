/*
Copyright 2024 ADIRO Automatisierungstechnik GmbH
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
-- END LICENSE BLOCK ------------------------------------------------

Author: Timo Schwarzer, Stefan Knoblauch
Maintainer: Alexander Feuchter
Last-Updated: January 22, 2024
*/

#include "mitsubishi_hardware_interface.hpp"
#include <string>
#include <vector>

namespace mitsubishi_hardware_interface
{
  CallbackReturn MitsubishiHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // robot has 6 joints and 2 interfaces
    joint_position_.assign(6, 0);
    joint_position_command_.assign(6, 0);

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> MitsubishiHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
                                                                       hardware_interface::HW_IF_POSITION, &joint_position_[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> MitsubishiHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (uint i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_command_[i]));
    }

    return command_interfaces;
  }

  CallbackReturn MitsubishiHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_INFO(rclcpp::get_logger("MitsubishiHardwareInterface"), "Starting the realtime interface");

    robotIpAddress = info_.hardware_parameters["robot_ip"];
    robotUdpPort = stoi(info_.hardware_parameters["robot_udp_port"]);

    // Socket creation
    destSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (destSocket_ < 0)
    {
      RCLCPP_FATAL(rclcpp::get_logger("MitsubishiHardwareInterface"), "ERROR: socket unsuccessful");
      return CallbackReturn::ERROR;
    }

    destSockAddr_.sin_family = AF_INET;
    destSockAddr_.sin_port = htons(robotUdpPort);
    destSockAddr_.sin_addr.s_addr = inet_addr(robotIpAddress.c_str());

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    if (int err = setsockopt(destSocket_, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)))
    {
      RCLCPP_FATAL(
          rclcpp::get_logger("MitsubishiHardwareInterface"), "ERROR: setsockopt unsuccessful : %i ", err);
      return CallbackReturn::ERROR;
    }

    if (send_first_null_package() == false)
    {
      RCLCPP_ERROR(rclcpp::get_logger("MitsubishiHardwareInterface"), "System setup failed!");
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("MitsubishiHardwareInterface"), "System Successfully started!");
    return CallbackReturn::SUCCESS;
  }

  bool MitsubishiHardwareInterface::send_first_null_package()
  {
    memset(&MXTsend_, 0, sizeof(MXTsend_));

    MXTsend_.Command = MXT_CMD_NULL;
    MXTsend_.SendType = MXT_TYP_NULL;
    MXTsend_.RecvType = MXT_TYP_JOINT;
    MXTsend_.SendIOType = MXT_IO_NULL;
    MXTsend_.RecvIOType = MXT_IO_NULL;
    MXTsend_.CCount = counter_;

    socket_status_ = sendto(destSocket_, (char *)&MXTsend_, sizeof(MXTCMD), 0, (struct sockaddr *)&destSockAddr_, sizeof(destSockAddr_));
    if (socket_status_ != sizeof(MXTCMD))
    {
      RCLCPP_FATAL(rclcpp::get_logger("MitsubishiHardwareInterface"),
                   "ERROR: sendto unsuccessful in first send.");
      return false;
    }

    socket_status_ = recvfrom(destSocket_, (char *)&MXTrecv_, sizeof(MXTrecv_), 0, NULL, NULL);
    if (socket_status_ < 0)
    {
      RCLCPP_FATAL(
          rclcpp::get_logger("MitsubishiHardwareInterface"),
          "ERROR: recvfrom unsuccessful in first recv.");
      return false;
    }

    joint_position_[0] = MXTrecv_.dat.jnt.j1;
    joint_position_[1] = MXTrecv_.dat.jnt.j2;
    joint_position_[2] = MXTrecv_.dat.jnt.j3;
    joint_position_[3] = MXTrecv_.dat.jnt.j4;
    joint_position_[4] = MXTrecv_.dat.jnt.j5;
    joint_position_[5] = MXTrecv_.dat.jnt.j6;
    joint_position_command_ = joint_position_;

    return true;
  }

  CallbackReturn MitsubishiHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_INFO(rclcpp::get_logger("MitsubishiHardwareInterface"), "Stopping the realtime interface");
    close(destSocket_);
    RCLCPP_INFO(rclcpp::get_logger("MitsubishiHardwareInterface"), "System successfully stopped!");

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type MitsubishiHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    if (counter_ <= 1) 
    {
      return hardware_interface::return_type::OK;
    }

    memset(&MXTrecv_, 0, sizeof(MXTrecv_));
    socket_status_ = recvfrom(destSocket_, (char *)&MXTrecv_, sizeof(MXTrecv_), 0, NULL, NULL);
    if (socket_status_ < 0)
    {
      // RCLCPP_ERROR(rclcpp::get_logger("MitsubishiHardwareInterface"), "Receive from UDP socket failed");
      // return hardware_interface::return_type::ERROR;
    }

    joint_position_[0] = MXTrecv_.dat.jnt.j1;
    joint_position_[1] = MXTrecv_.dat.jnt.j2;
    joint_position_[2] = MXTrecv_.dat.jnt.j3;
    joint_position_[3] = MXTrecv_.dat.jnt.j4;
    joint_position_[4] = MXTrecv_.dat.jnt.j5;
    joint_position_[5] = MXTrecv_.dat.jnt.j6;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type MitsubishiHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
  {    
    memset(&MXTsend_, 0, sizeof(MXTsend_));

    // Second and following times
    MXTsend_.Command = MXT_CMD_MOVE;

    MXTsend_.SendType = MXT_TYP_JOINT;
    MXTsend_.RecvType = MXT_TYP_JOINT;
    MXTsend_.SendIOType = MXT_IO_NULL;
    MXTsend_.RecvIOType = MXT_IO_IN;
    MXTsend_.BitTop = 16;
    MXTsend_.BitMask = 0xffff;
    MXTsend_.IoData = 0;
    MXTsend_.CCount = counter_;

    MXTsend_.dat.jnt.j1 = (float)joint_position_command_[0];
    MXTsend_.dat.jnt.j2 = (float)joint_position_command_[1];
    MXTsend_.dat.jnt.j3 = (float)joint_position_command_[2];
    MXTsend_.dat.jnt.j4 = (float)joint_position_command_[3];
    MXTsend_.dat.jnt.j5 = (float)joint_position_command_[4];
    MXTsend_.dat.jnt.j6 = (float)joint_position_command_[5];

    socket_status_ = sendto(destSocket_, (char *)&MXTsend_, sizeof(MXTCMD), 0, (struct sockaddr *)&destSockAddr_, sizeof(destSockAddr_));
    if (socket_status_ != sizeof(MXTCMD))
    {
      RCLCPP_ERROR(rclcpp::get_logger("MitsubishiHardwareInterface"), "Send Joint positions to controller was not successful");
      return hardware_interface::return_type::ERROR;
    }

    counter_++;
    return hardware_interface::return_type::OK;
  }

} // namespace mitsubishi_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    mitsubishi_hardware_interface::MitsubishiHardwareInterface, hardware_interface::SystemInterface)