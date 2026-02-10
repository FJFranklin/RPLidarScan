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

if sys.version_info.minor >= 11:
    from asyncio import TaskGroup
else:
    from taskgroup import TaskGroup

from .base import Lidar2D_DataManager, UI_Component

import dearpygui.dearpygui as dpg

class UI_Lidar(UI_Component):
    """User interface to play/pause the lidar, if possible, and to display the data.
    """

    @staticmethod
    def __callback_play_pause(sender, app_data, user_data: Self) -> None:
        """Callback: The Play/Pause button was pressed."""
        if user_data.lidar.playing:
            user_data.pause()
        else:
            user_data.play()

    def play(self) -> None:
        """Try to start the lidar."""
        if self.playable:
            if self.lidar.play():
                dpg.configure_item(self.pp_btag, texture_tag=self.ui.icon_texture["Pause"])

    def pause(self) -> None:
        """Try to stop the lidar."""
        if self.playable:
            self.lidar.pause()
            dpg.configure_item(self.pp_btag, texture_tag=self.ui.icon_texture["Play"])

    def set_zoom(self, zoom: int) -> None:
        """Change the zoom setting (1-12)."""
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
        self.redraw = True

    @staticmethod
    def __callback_zoom_in(sender, app_data, user_data: Self) -> None:
        """Callback: Zoom in."""
        user_data.zoom_in()

    def zoom_in(self) -> None:
        """Zoom in."""
        if self.zoom > 1:
            self.set_zoom(self.zoom - 1)

    @staticmethod
    def __callback_zoom_out(sender, app_data, user_data: Self) -> None:
        """Callback: Zoom out."""
        user_data.zoom_out()

    def zoom_out(self) -> None:
        """Zoom out."""
        if self.zoom < 12:
            self.set_zoom(self.zoom + 1)

    def __init__(self, ui, subwin) -> None:
        """Initialise, and build the lidar UI."""
        self.ui = ui
        self.subwin = subwin
        self.zoom: int = 12
        self.lidar: Lidar2D_DataManager = self.ui.opts["lidar"]
        self.playable: bool = self.ui.opts.get("can_play_pause", True)
        self.redraw: bool = False

        inset = dpg.add_group(parent=subwin, pos=(self.ui.subwin_inset_x, self.ui.subwin_inset_y))

        self.pp_ttag = self.ui.icon_texture["Play"]
        self.pp_btag = 'b' + self.pp_ttag
        dpg.add_image_button(self.pp_ttag, label="Play", user_data=self, callback=UI_Lidar.__callback_play_pause,
                             width=self.ui.button_width, height=self.ui.button_height, tag=self.pp_btag, parent=inset)
        if not self.playable:
            dpg.disable_item(self.pp_btag)

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
            self.__draw_grid()

    def __m_to_px(self, m: float) -> float:
        """Convert from metres to pixels on the drawing canvas."""
        return m / float(self.zoom) * self.ui.canvas_min
        
    def __draw_grid(self) -> None:
        """Draw the grid for the current zoom level."""
        px_o = (self.ui.canvas_x, self.ui.canvas_y)
        gray = (191, 191, 191, 255)
        for ir in range(self.zoom):
            px_r = self.__m_to_px(ir + 1)
            dpg.draw_circle(px_o, px_r, color=gray, thickness=2)
        if self.zoom <= 2:
            gray = (127, 127, 127, 255)
            if self.zoom > 1:
                for ir in range(9):
                    px_r = self.__m_to_px((ir + 1) * 0.1 + 1)
                    dpg.draw_circle(px_o, px_r, color=gray, thickness=1)
            for ir in range(9):
                px_r = self.__m_to_px((ir + 1) * 0.1)
                dpg.draw_circle(px_o, px_r, color=gray, thickness=1)

    def update(self) -> bool:
        """Update lidar data and redraw if necessary."""

        if not self.lidar.L2D_update():
            return False

        if not self.redraw and not self.lidar.L2D_updated:
            return True # no need to update

        self.redraw = False
        distances, intensities = self.lidar.L2D_data
        dpg.delete_item(self.dl)

        with dpg.drawlist(parent=self.subwin, width=self.ui.subwin_width, height=self.ui.subwin_height-20) as dl:
            self.dl = dl
            self.__draw_grid()

            da = math.pi * 2 / 500 # angular resolution
            for angle, distance in distances.items():
                if distance is None:
                    continue
                r = self.__m_to_px(self.lidar.L2D_scaling * distance)
                if self.lidar.L2D_radians: # TODO: handle this better
                    a = math.pi / 2 - angle_rad
                else:
                    a = math.radians(angle - 90)
                c = math.cos(a)
                s = math.sin(a)

                cx = self.ui.canvas_x + r * c
                cy = self.ui.canvas_y + r * s
                dx = - r * da * s / 2
                dy =   r * da * c / 2
                dpg.draw_line((cx-dx, cy-dy), (cx+dx, cy+dy), color=(255, 0, 0, 255), thickness=1)

        return True

    def set_task_group(self, task_group: TaskGroup) -> None:
        """Set the current asyncio event loop controller."""
        if self.playable:
            self.lidar.set_task_group(task_group)

    def app_will_end(self) -> None:
        """Notification that the application is ending."""
        self.lidar.L2D_app_will_end() # pass it on
