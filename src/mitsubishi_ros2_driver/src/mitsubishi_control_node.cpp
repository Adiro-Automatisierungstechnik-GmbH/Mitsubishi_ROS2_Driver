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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "mitsubishi_hardware_interface.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MitsubishiControlNode : public rclcpp::Node
{
  public:
    MitsubishiControlNode()
    : Node("mitsubishi_control_node"), count_(0)
    {      
      // Set the desired spin rate (3.5 ms)
      spin_rate_ = std::make_shared<rclcpp::Rate>(1000000 / 3500);
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&MitsubishiControlNode::timer_callback, this));
      
      memset(&jointTarget, 0, sizeof(JOINT));
      jointTarget.j1 = 0;
      jointTarget.j2 = 0;
      jointTarget.j3 = 1.55;
      jointTarget.j4 = 0;
      jointTarget.j5 = 1.55;
      jointTarget.j6 = 0;
      jointTarget.j7 = 0;
      jointTarget.j8 = 0;

      robot.connect_realtime();
      robot.send_first_null_package();
    }

  private:
    void timer_callback()
    {
      if (robot.bRealtimeConnected)
      {
        robot.receive_data();
        robot.send_data(jointTarget);
      }
      
      // Sleep to achieve the desired spin rate
      spin_rate_->sleep();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;    
    std::shared_ptr<rclcpp::Rate> spin_rate_;
    size_t count_;
    MitsubishiHardware robot;
    JOINT jointTarget;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MitsubishiControlNode>());
  rclcpp::shutdown();
  return 0;
}