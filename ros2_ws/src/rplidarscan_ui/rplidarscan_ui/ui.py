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

import os

from .UI_Controller import UI_Controller

opt_device  = '/dev/rplidar'
opt_display = 'HyperPixel'
opt_admin   = False
opt_fullscr = False

ros2_client_mode = True
shutdown_on_exit = False

def main(ros_args = None):
    if ros2_client_mode: # TODO: Integrate with save/load settings, etc.
        from ament_index_python.packages import get_package_share_directory
        share_folder = get_package_share_directory('rplidarscan_ui')
    else:
        share_folder = 'rplidarscan_ui'

    cmd_opts = {}
    cmd_opts["device"    ] = opt_device
    cmd_opts["display"   ] = opt_display
    cmd_opts["admin"     ] = opt_admin
    cmd_opts["fullscreen"] = opt_fullscr
    cmd_opts["shutdown"  ] = shutdown_on_exit
    cmd_opts["share"     ] = share_folder

    if ros2_client_mode:
        from .RPLidarScan_ROS import RPLidarScan_ROS
        lidar, can_play_pause = RPLidarScan_ROS.create_data_manager(ros_args)
    else:
        from .RPLidarScan_Serial import RPLidarScan_Serial
        lidar, can_play_pause = RPLidarScan_Serial.create_data_manager(opt_device)

    cmd_opts["lidar"] = lidar
    cmd_opts["can_play_pause"] = can_play_pause

    ui = UI_Controller(cmd_opts)
    ui.setup()
    ui.run()

    if ros2_client_mode:
        lidar.destroy_node()

    if ui.opts["admin"] and ui.opts["shutdown"]:
        os.system("shutdown -h now")
