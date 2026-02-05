# Copyright 2026 Francis James Franklin
#
# With thanks to: https://github.com/m2-farzan/ros2-asyncio
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

import math
import asyncio

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

class RPLidarScan_UI(Node):

    def __init__(self):
        super().__init__('rplidarscan_ui')
        self.scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        count = int(msg.scan_time / msg.time_increment)
        print("[SLLIDAR INFO]: I heard a laser scan " + msg.header.frame_id + "[{c}]:".format(c=count))
        print("[SLLIDAR INFO]: angle_range : [{n}, {x}]".format(n=math.radians(msg.angle_min), x=math.radians(msg.angle_max)))

        for i in range(count):
            degree = math.radians(msg.angle_min + msg.angle_increment * i);
            print("[SLLIDAR INFO]: angle-distance : [{d}, {r}]".format(d=degree, r=msg.ranges[i]))

async def ros_loop(ui):
    while rclpy.ok():
        rclpy.spin_once(ui, timeout_sec=0)
        await asyncio.sleep(1e-4)

def main(args=None):
    rclpy.init(args=args)

    ui = RPLidarScan_UI()

    print("Running RPLidarScan UI... ", end="", flush=True)
    try:
        future = asyncio.wait([ros_loop(ui)])
        asyncio.get_event_loop().run_until_complete(future)
    except KeyboardInterrupt:
        print(" (interrupt) ", end="")
    finally:
        print("Exiting.")

    ui.destroy_node()
    #rclpy.shutdown() # This causes a shutdown within a shutdown

if __name__ == '__main__':
    main()
