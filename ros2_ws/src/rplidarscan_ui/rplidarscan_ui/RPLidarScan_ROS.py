# Copyright 2026 Francis James Franklin
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

import sys
import math

if sys.version_info.minor >= 11:
    from typing import Self
else:
    from typing_extensions import Self

from typing import Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from .base import Lidar2D_DataManager

class RPLidarScan_ROS(Node, Lidar2D_DataManager):
    """ROS2 subscriber to listen for 2D lidar scan data.
    """

    @staticmethod
    def create_data_manager(ros_args) -> Self:
        """ROS2 setup and create node."""
        rclpy.init(args=ros_args)
        return RPLidarScan_ROS(), False

    def __init__(self) -> None:
        """Initiate, and subscribe to /scan messages."""
        Node.__init__(self, 'rplidarscan_ui')
        Lidar2D_DataManager.__init__(self, 1.0, True) # radians and metres
        self.scan = self.create_subscription(LaserScan, '/scan', self.__scan_callback, 10)

    def __scan_callback(self, msg: LaserScan) -> None:
        """Handle /scan messages through ROS2."""
        rslen = len(msg.ranges)
        data_distance = {}
        data_intensity = {}
        for i in range(rslen):
            theta = msg.angle_min + (msg.angle_max - msg.angle_min) * i / (rslen - 1.0)
            distance = msg.ranges[i]
            if not math.isfinite(distance):
                continue
            data_distance[theta] = distance
            if len(msg.intensities):
                data_intensity[theta] = msg.intensities[i]
        self.L2D_data = data_distance, data_intensity

    def L2D_update(self) -> bool:
        """Check to see if new lidar data is available."""
        if not rclpy.ok():
            return False

        rclpy.spin_once(self, timeout_sec=0)
        return True

    def L2D_app_will_end(self) -> None:
        """Notification that the application is ending."""
        pass
