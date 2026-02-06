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

import sys
import os.path
import time
import math
import asyncio

if sys.version_info.minor >= 11:
    from asyncio import run, TaskGroup
else:
    from taskgroup import run, TaskGroup

import dearpygui.dearpygui as dpg

from rplidarc1.scanner import RPLidar

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

opt_device  = '/dev/rplidar'
opt_display = 'HyperPixel'
opt_admin   = False
opt_fullscr = False

ros2_client_mode = True
shutdown_on_exit = False

class RPLidarScan_Node(Node):

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

class UI_Lidar(object):
    @staticmethod
    def __callback_play_pause(sender, app_data, user_data):
        if user_data.playing():
            user_data.pause()
        else:
            user_data.play()

    def __lidar_begin(self):
        if not os.path.exists(args.device):
            print("RPLidarScan: RPLidar C1 device path '" + args.device + "' does not exist.")
        else:
            # Initialize the RPLidar with the appropriate port and baudrate
            try:
                self.lidar = RPLidar(args.device, 460800)
            except ConnectionError:
                print("RPLidarScan: Unable to connect to RPLidar C1 device.")
                self.lidar = None

        if self.lidar is not None:
            print("RPLidarScan: Connected to RPLidar C1 at device path '" + args.device + "'.")
            self.task_group.create_task(self.lidar.simple_scan(make_return_dict=True))
            return True

        return False

    def playing(self):
        return self.lidar is not None

    def play(self):
        if self.lidar is None:
            if self.__lidar_begin():
                dpg.configure_item(self.pp_btag, texture_tag=self.ui.icon_texture["Pause"])

    def pause(self):
        if self.lidar is None:
            return

        self.lidar.stop_event.set()
        self.lidar.reset()
        self.lidar = None
        dpg.configure_item(self.pp_btag, texture_tag=self.ui.icon_texture["Play"])

    def set_zoom(self, zoom):
        self.zoom = zoom

        if self.zoom < 12:
            dpg.enable_item(self.zo_btag)
        else:
            self.zoom = 12
            dpg.disable_item(self.zo_btag)

        if self.zoom > 1:
            dpg.enable_item(self.zi_btag)
        else:
            self.zoom = 1
            dpg.disable_item(self.zi_btag)

        dpg.set_value(self.zz_btag, "Range: {r}m".format(r=self.zoom))

    @staticmethod
    def __callback_zoom_in(sender, app_data, user_data):
        user_data.zoom_in()

    def zoom_in(self):
        if self.zoom > 1:
            self.set_zoom(self.zoom - 1)

    @staticmethod
    def __callback_zoom_out(sender, app_data, user_data):
        user_data.zoom_out()

    def zoom_out(self):
        if self.zoom < 12:
            self.set_zoom(self.zoom + 1)

    def __init__(self, ui, subwin):
        self.ui = ui
        self.subwin = subwin
        self.zoom = 12
        self.lidar = None

        inset = dpg.add_group(parent=subwin, pos=(self.ui.subwin_inset_x, self.ui.subwin_inset_y))

        self.pp_ttag = self.ui.icon_texture["Play"]
        self.pp_btag = 'b' + self.pp_ttag
        dpg.add_image_button(self.pp_ttag, label="Play", user_data=self, callback=UI_Lidar.__callback_play_pause,
                             width=self.ui.button_width, height=self.ui.button_height, tag=self.pp_btag, parent=inset)

        dpg.add_spacer(height=self.ui.button_height/2, parent=inset)

        self.zi_ttag = self.ui.icon_texture["Zoom In"]
        self.zi_btag = 'b' + self.zi_ttag
        dpg.add_image_button(self.zi_ttag, label="Zoom In", user_data=self, callback=UI_Lidar.__callback_zoom_in,
                             width=self.ui.button_width, height=self.ui.button_height, tag=self.zi_btag, parent=inset)

        self.zz_btag = "bRangeText"
        dpg.add_text("Range: XXm", parent=inset, tag=self.zz_btag)
        dpg.bind_item_font(self.zz_btag, self.ui.font_small)

        self.zo_ttag = self.ui.icon_texture["Zoom Out"]
        self.zo_btag = 'b' + self.zo_ttag
        dpg.add_image_button(self.zo_ttag, label="Zoom Out", user_data=self, callback=UI_Lidar.__callback_zoom_out,
                             width=self.ui.button_width, height=self.ui.button_height, tag=self.zo_btag, parent=inset)

        self.set_zoom(12)

        with dpg.drawlist(parent=self.subwin, width=self.ui.subwin_width, height=self.ui.subwin_height-20) as dl: # FIXME: Horizontal scrolling issue - here?
            self.dl = dl
            self.draw_grid()
            self.line = dpg.draw_line((10, 10), (100, 100), color=(255, 0, 0, 255), thickness=1)

    def m_to_px(self, m):
        return m / float(self.zoom) * self.ui.canvas_min
        
    def draw_grid(self):
        px_o = (self.ui.canvas_x, self.ui.canvas_y)
        gray = (191, 191, 191, 255)
        for ir in range(self.zoom):
            px_r = self.m_to_px(ir + 1)
            dpg.draw_circle(px_o, px_r, color=gray, thickness=2)
        if self.zoom <= 2:
            gray = (127, 127, 127, 255)
            if self.zoom > 1:
                for ir in range(9):
                    px_r = self.m_to_px((ir + 1) * 0.1 + 1)
                    dpg.draw_circle(px_o, px_r, color=gray, thickness=1)
            for ir in range(9):
                px_r = self.m_to_px((ir + 1) * 0.1)
                dpg.draw_circle(px_o, px_r, color=gray, thickness=1)

    def set_task_group(self, tg):
        self.task_group = tg

    def update(self):
        # Remove any data in the queue (dist vs angle gets saved to the dict internally)
        if self.lidar is not None:
            while not self.lidar.output_queue.empty():
                data = self.lidar.output_queue.get_nowait()
                #print(f"Angle: {data['a_deg']}°, Distance: {data['d_mm']}mm, Quality: {data['q']}")

        dpg.delete_item(self.dl)

        with dpg.drawlist(parent=self.subwin, width=self.ui.subwin_width, height=self.ui.subwin_height-20) as dl:
            self.dl = dl
            self.draw_grid()

            if self.lidar is not None:
                da = math.pi * 2 / 500 # angular resolution
                for angle_deg, distance_mm in self.lidar.output_dict.items():
                    if distance_mm is None:
                        continue
                    r = self.m_to_px(0.001 * distance_mm)
                    a = math.radians(angle_deg - 90)
                    c = math.cos(a)
                    s = math.sin(a)

                    cx = self.ui.canvas_x + r * c
                    cy = self.ui.canvas_y + r * s
                    dx = - r * da * s / 2
                    dy =   r * da * c / 2
                    dpg.draw_line((cx-dx, cy-dy), (cx+dx, cy+dy), color=(255, 0, 0, 255), thickness=1)

    def app_will_end(self):
        if self.lidar is not None:
            self.pause()

class SideMenuButton(object):
    def __init__(self, ui, name):
        self.ui = ui
        self.name = name

        self.btag = "button_" + name
        self.stag = "subwin_" + name
        self.ttag = self.ui.icon_texture[name]

        self.subwin = None
        self.sub_ui = None

        if self.name == "Home":
            self.ui.smb_Home = self

    def app_will_end(self):
        if self.sub_ui is not None:
            self.sub_ui.app_will_end()

    def deselect(self):
        dpg.hide_item(self.subwin)
        self.ui.smb_active = None

    def select(self):
        if self.ui.smb_active is not None:
            self.ui.smb_active.deselect()

        dpg.show_item(self.subwin)
        self.ui.smb_active = self

    @staticmethod
    def __callback_exit():
        dpg.stop_dearpygui()

    @staticmethod
    def __callback_shutdown():
        dpg.stop_dearpygui()

        global shutdown_on_exit
        shutdown_on_exit = True

    @staticmethod
    def __callback(sender, app_data, user_data):
        user_data.select()

    def __add_large_button(self, label, callback):
        button = dpg.add_button(label=label, callback=callback, width=self.ui.subbtn_width, height=self.ui.subbtn_height)
        dpg.bind_item_font(button, self.ui.font_large)
        return button

    def add_button_and_window(self):
        dpg.add_image_button(self.ttag, label=self.name, user_data=self, callback=SideMenuButton.__callback,
                             width=self.ui.button_width, height=self.ui.button_height, tag=self.btag)

        with dpg.window(tag=self.stag, pos=(self.ui.subwin_x, self.ui.subwin_y), width=self.ui.subwin_width, height=self.ui.subwin_height,
                        no_title_bar=True, show=False) as subwin:
            self.subwin = subwin

            if self.name == "Lidar":
                self.sub_ui = UI_Lidar(self.ui, self.subwin)

            if self.name == "Shutdown":
                ui_exit = self.__add_large_button("Exit", SideMenuButton.__callback_exit)
                if args.admin:
                    ui_shut = self.__add_large_button("Shutdown", SideMenuButton.__callback_shutdown)
                    dpg.bind_item_theme(ui_shut, self.ui.theme_caution)

    def set_task_group(self, tg):
        if self.sub_ui is not None:
            self.sub_ui.set_task_group(tg)

    def update(self):
        if self.sub_ui is not None:
            self.sub_ui.update()

class UI():
    def __init__(self):
        # Can we get this padding from DearPyGui somehow?
        padding = 10

        if opt_display == 'Elecrow':
            self.display_title  = 'Elecrow 7\" Display'
            self.display_width  = 1024
            self.display_height =  600
            self.button_width   =  100
            self.fontsize_small =   25
        else: # opt_display = 'HyperPixel'
            self.display_title  = 'HyperPixel 4.0 Display'
            self.display_width  =  800
            self.display_height =  480
            self.button_width   =   80
            self.fontsize_small =   20

        # Square buttons, so
        self.button_height  = self.button_width

        # Main window for application
        self.subwin_x       = self.button_width + 2 * padding
        self.subwin_y       = padding
        self.subwin_width   = self.display_width  - self.subwin_x - padding
        self.subwin_height  = self.display_height - self.subwin_y - padding

        # Font for, e.g., large buttons
        self.fontsize_large = self.fontsize_small * 2
        self.font_small = None
        self.font_large = None

        # Side sub-menu
        self.subwin_inset_x = self.subwin_width - self.button_width - 2 * padding
        self.subwin_inset_y = padding

        # Main application window, not including inset
        self.canvas_width   = self.subwin_inset_x
        self.canvas_height  = self.subwin_height - 2 * padding
        self.canvas_x   = int(self.canvas_width  / 2)
        self.canvas_y   = int(self.canvas_height / 2)
        self.canvas_min = min(self.canvas_x, self.canvas_y)

        # Large button
        self.subbtn_width   = self.subwin_width - 2 * padding
        self.subbtn_height  = self.fontsize_large + padding

        self.icon_prefix = "icons/png/"
        self.icon_suffix = "-{w}x{h}.png".format(w=self.button_width, h=self.button_height)

        self.icon_defs = [
            [ "Home",     "home" ],
            [ "Back",     "arrow-left" ],
            [ "Lidar",    "clock" ],
            [ "Settings", "settings" ],
            [ "Shutdown", "system-shut" ],
            [ "Play",     "play" ],
            [ "Pause",    "pause" ],
            [ "Zoom In",  "zoom-in" ],
            [ "Zoom Out", "zoom-out" ]]
        self.icon_texture = {}

        self.sidemenu_button_defs = [ "Home", "Lidar", "Settings", "Shutdown" ]
        self.sidemenu_buttons = []

        self.theme_caution = None

        self.ui_sidebar = None

        self.smb_active = None
        self.smb_Home = None

        self.node = None

    def setup(self):
        # Start by creating DearPyGui context
        dpg.create_context()

        # Load icon images as textures; crossref with label-ttag dictionary
        with dpg.texture_registry(show=False):
            for icon_def in self.icon_defs:
                label, icon = icon_def

                filename = self.icon_prefix + icon + self.icon_suffix
                image = dpg.load_image(filename)
                if image is None:
                    print("Error: failed to load image from '" + filename + "'")
                width, height, channels, data = image

                ttag = "tt_" + label
                dpg.add_static_texture(width=width, height=height, default_value=data, tag=ttag)

                self.icon_texture[label] = ttag

        # Add scaled fonts
        with dpg.font_registry():
            self.font_small = dpg.add_font("fonts/NotoSerifCJKjp-Medium.otf", self.fontsize_small)
            self.font_large = dpg.add_font("fonts/NotoSerifCJKjp-Medium.otf", self.fontsize_large)

        # Create custom theme
        with dpg.theme() as tc:
            self.theme_caution = tc

            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (255, 0, 0), category=dpg.mvThemeCat_Core)

        # Create side menu buttons and associated subwin panes
        for bdef in self.sidemenu_button_defs:
            self.sidemenu_buttons.append(SideMenuButton(self, bdef))

        with dpg.window(tag="Primary Window"):
            #dpg.bind_font(font_large)
            with dpg.group() as sidebar:
                self.ui_sidebar = sidebar
                for b in self.sidemenu_buttons:
                    b.add_button_and_window()

        # Create viewport and setup
        dpg.create_viewport(title=self.display_title, width=self.display_width, height=self.display_height)
        dpg.setup_dearpygui()

        # Display the application-level window that everything sits inside
        dpg.show_viewport()
        if opt_fullscr:
            dpg.toggle_viewport_fullscreen()

        # The primary window fills the whole viewport
        dpg.set_primary_window("Primary Window", True)

        # Default to Home
        self.smb_Home.select()

    def ros_init(self, args):
        rclpy.init(args=args)

        self.node = RPLidarScan_Node()

    def run(self):
        async def main_loop(ui):
            while dpg.is_dearpygui_running():
                if self.node is not None:
                    if not rclpy.ok():
                        break
                    rclpy.spin_once(ui, timeout_sec=0)
                for b in ui.sidemenu_buttons:
                    b.update()
                dpg.render_dearpygui_frame()
                await asyncio.sleep(0.001)

        async def main_async(ui):
            async with TaskGroup() as tg:
                for b in ui.sidemenu_buttons:
                    b.set_task_group(tg)
                tg.create_task(main_loop(ui))

        print("Running RPLidarScan UI... ", end="", flush=True)
        try:
            future = asyncio.wait([main_async(self)])
            asyncio.get_event_loop().run_until_complete(future)
        except KeyboardInterrupt:
            print(" (interrupt) ", end="")
        finally:
            print("Exiting.")

        # tidy up and exit cleanly
        for b in self.sidemenu_buttons:
            b.app_will_end()

        dpg.destroy_context()

        if self.node is not None:
            self.node.destroy_node()

        if opt_admin and shutdown_on_exit:
            os.system("shutdown -h now")

def main(args=None):
    ui = UI()
    ui.setup()

    if ros2_client_mode:
        ui.ros_init(args)

    ui.run()

def device_test():
    lidar = None
    if not os.path.exists(args.device):
        print("RPLidarScan: RPLidar C1 device path '" + args.device + "' does not exist.")
    else:
        # Initialize the RPLidar with the appropriate port and baudrate
        try:
            lidar = RPLidar(args.device, 460800)
        except ConnectionError:
            print("RPLidarScan: Unable to connect to RPLidar C1 device.")
            lidar = None

    if lidar is not None:
        print("RPLidarScan: Connected to RPLidar C1 at device path '" + args.device + "'.")

    if lidar is None:
        sys.exit(0)

    # Test code from https://github.com/dsaadatmandi/rplidarc1

    async def process_scan_data():
        # Start a scan with dictionary output
        async with TaskGroup() as tg:
            # Create a task to stop scanning after 5 seconds
            tg.create_task(wait_and_stop(5, lidar.stop_event))

            # Create a task to process data from the queue
            tg.create_task(process_queue(lidar.output_queue, lidar.stop_event))

            # Start the scan with dictionary output
            tg.create_task(lidar.simple_scan(make_return_dict=True))

        # Access the scan data dictionary
        print(lidar.output_dict)

        # Reset the device
        lidar.reset()

    async def wait_and_stop(seconds, event):
        await asyncio.sleep(seconds)
        event.set()

    async def process_queue(queue, stop_event):
        while not stop_event.is_set():
            if not queue.empty():
                data = await queue.get()
                # Process the data
                print(f"Angle: {data['a_deg']}°, Distance: {data['d_mm']}mm, Quality: {data['q']}")
            else:
                await asyncio.sleep(0.1)

    # Run the example
    try:
        run(process_scan_data())
    except KeyboardInterrupt:
        lidar.reset()

    sys.exit(0)

if __name__ == '__main__':
    import argparse

    ros2_client_mode = False

    parser = argparse.ArgumentParser(description="Touchscreen GUI.")

    parser.add_argument('--device',     help='RPLider C1 device [/dev/rplidar].',               default='/dev/rplidar', type=str)
    parser.add_argument('--target',     help='Target display (Elecrow, HyperPixel) [Elecrow].', default='Elecrow', choices=['Elecrow', 'HyperPixel'])
    parser.add_argument('--fullscreen', help='Enter fullscreen display mode.',                  action='store_true')
    parser.add_argument('--devicetest', help='Test connection to RPLidar C1 device.',           action='store_true')
    parser.add_argument('--admin',      help='Enable admin features.',                          action='store_true')

    args = parser.parse_args()

    if args.devicetest:
        device_test() # runs test and exits

    opt_device  = args.device
    opt_display = args.target
    opt_admin   = args.admin
    opt_fullscr = args.fullscreen

    main()
