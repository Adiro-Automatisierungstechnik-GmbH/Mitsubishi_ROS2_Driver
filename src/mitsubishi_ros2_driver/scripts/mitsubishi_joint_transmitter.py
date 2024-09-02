#!/usr/bin/env python3
#-- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2024 ADIRO Automatisierungstechnik GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#-- END LICENSE BLOCK ------------------------------------------------

# Author: Timo Schwarzer, Stefan Knoblauch
# Maintainer: Alexander Feuchter
# Last-Updated: January 22, 2024
# Description: Launch a Mitsubishi RV-2FR robot URDF file using Rviz.
# https://adiro.com/en/
# https://www.software-byts.de/ 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('my_joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'target_joint_data', 10)        
        self.joint_state_msg = JointState()

        # Set the rate at which to publish the JointState message
        self.timer = self.create_timer(0.0035, self.publish_joint_state)  # 3.5 ms period

    def publish_joint_state(self):
        # Populate the JointState message with dummy joint values
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()        
        self.joint_state_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Publish the JointState message
        self.publisher.publish(self.joint_state_msg)        

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()

    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        pass

    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()