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

import rplidarscan_ui.ui as ui

import argparse

parser = argparse.ArgumentParser(description="Touchscreen GUI.")

parser.add_argument('--device',     help='RPLider C1 device [/dev/rplidar].',               default='/dev/rplidar', type=str)
parser.add_argument('--target',     help='Target display (Elecrow, HyperPixel) [Elecrow].', default='Elecrow', choices=['Elecrow', 'HyperPixel'])
parser.add_argument('--fullscreen', help='Enter fullscreen display mode.',                  action='store_true')
parser.add_argument('--devicetest', help='Test connection to RPLidar C1 device.',           action='store_true')
parser.add_argument('--admin',      help='Enable admin features.',                          action='store_true')

args = parser.parse_args()

if args.devicetest:
    from RPLidarScan_Serial import L2D_device_test
    L2D_device_test(args.device) # runs test and exits
else:
    ui.ros2_client_mode = False

    ui.opt_device  = args.device
    ui.opt_display = args.target
    ui.opt_admin   = args.admin
    ui.opt_fullscr = args.fullscreen

    ui.main()
