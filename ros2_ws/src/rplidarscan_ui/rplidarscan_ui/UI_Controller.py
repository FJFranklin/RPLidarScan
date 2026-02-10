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
import math

if sys.version_info.minor >= 11:
    from typing import Self
else:
    from typing_extensions import Self

from typing import Dict

import asyncio

if sys.version_info.minor >= 11:
    from asyncio import TaskGroup
else:
    from taskgroup import TaskGroup

from .base import Lidar2D_DataManager
from .UI_SideMenuButton import UI_SideMenuButton

import dearpygui.dearpygui as dpg

class UI_Controller():
    """User interface settings and control.
    """

    def __init__(self, opts: Dict) -> None:
        """Initiate, with options."""

        self.opts = opts

        display = self.opts["display"]
        share_folder = self.opts["share"]

        # Can we get this padding from DearPyGui somehow?
        padding = 10

        if share_folder is not None:
            self.prefix = share_folder + '/'
        else:
            self.prefix = ""

        if display == 'Elecrow':
            self.display_title  = 'Elecrow 7\" Display'
            self.display_width  = 1024
            self.display_height =  600
            self.button_width   =  100
            self.fontsize_small =   25
        else: # display = 'HyperPixel'
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

        self.font_prefix = self.prefix + "fonts/"
        self.icon_prefix = self.prefix + "icons/png/"
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

    def setup(self) -> None:
        """Create the main user interface and its components."""
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
            self.font_small = dpg.add_font(self.font_prefix + "NotoSerifCJKjp-Medium.otf", self.fontsize_small)
            self.font_large = dpg.add_font(self.font_prefix + "NotoSerifCJKjp-Medium.otf", self.fontsize_large)

        # Create custom theme
        with dpg.theme() as tc:
            self.theme_caution = tc

            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (255, 0, 0), category=dpg.mvThemeCat_Core)

        # Create side menu buttons and associated subwin panes
        for bdef in self.sidemenu_button_defs:
            self.sidemenu_buttons.append(UI_SideMenuButton(self, bdef))

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
        if self.opts["fullscreen"]:
            dpg.toggle_viewport_fullscreen()

        # The primary window fills the whole viewport
        dpg.set_primary_window("Primary Window", True)

        # Default to Home
        self.smb_Home.select()

    def update(self) -> bool:
        """Update all tabs/panes."""
        for b in self.sidemenu_buttons:
            if not b.update():
                return False

        return True

    def run(self) -> None:
        """Create the asynchronous event loop."""

        async def main_loop(ui):
            while dpg.is_dearpygui_running():
                if not ui.update(): # returns false if should quit
                    break
                dpg.render_dearpygui_frame()
                await asyncio.sleep(0.001)

            # tidy up and exit cleanly; this *should* finish any other asynchronous tasks in the group
            for b in ui.sidemenu_buttons:
                b.app_will_end()

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
            # tidy up and exit cleanly
            for b in self.sidemenu_buttons:
                b.app_will_end()
        finally:
            print("Exiting.")

        dpg.destroy_context()
