#ifndef MITSUBIHI_HARDWARE_HPP_
#define MITSUBIHI_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <limits>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <arpa/inet.h>

#include "strdef.hpp"

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mitsubishi_hardware_interface
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  class HARDWARE_INTERFACE_PUBLIC MitsubishiHardwareInterface : public hardware_interface::SystemInterface
  {
  public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

  protected:
    bool send_first_null_package();

    std::string robotIpAddress;
    int robotUdpPort;

    // Communication
    sockaddr_in destSockAddr_;
    int destSocket_;
    MXTCMD MXTsend_, MXTrecv_;
    unsigned int counter_ = 0;
    int socket_status_;

    std::vector<double> joint_position_command_;
    std::vector<double> joint_position_;

    std::unordered_map<std::string, std::vector<std::string>> joint_interfaces =
        {{"position", {}}};
  };
} // namespace mitsubishi_hardware_interface

#endif // MITSUBIHI_HARDWARE_HPP_