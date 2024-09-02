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
# Last-Updated: July 31, 2024
# Description: Script to establish a TCP connection with a Melfa robot
# https://adiro.com/en/
# https://www.software-byts.de/ 

import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import Int32

import os
import socket 
from time import sleep

from enum import IntEnum

class ControllerStatus(IntEnum):
    UNKNOWN = -99
    STARTING = 1
    MXT_MODE_INITIALIZED = 10
    UDP_CONN_ESTABLISHED = 11
    ERROR = 900

class MitsubishiTCP(Node):

    def __init__(self):
        super().__init__('mitsubishi_tcp')
        self.publisher_ = self.create_publisher(Int32, 'mitsubishi_controller_status', 10)
        self.declare_parameter('robot_ip', '192.168.0.20')
        self.declare_parameter('robot_tcp_port', 10003)
        self.robot_ip = self.get_parameter('robot_ip').value
        self.robot_tcp_port = self.get_parameter('robot_tcp_port').value
        if len(sys.argv) == 3:
            self.robot_ip = sys.argv[1]
            self.robot_tcp_port = int(sys.argv[2])
        elif len(sys.argv) == 2:
            self.robot_ip = sys.argv[1]
        elif os.getenv('ROBOT_IP'):
            self.robot_ip = os.getenv('ROBOT_IP')
#**********************************************
# Node main entry point
#********************************************** 
def main(args=None):
    rclpy.init(args=args)

    mitsubishi_tcp = MitsubishiTCP()
    #mitsubishi_tcp.log_status('MitsubishiTCP is getting shutdown!')
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mitsubishi_tcp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
