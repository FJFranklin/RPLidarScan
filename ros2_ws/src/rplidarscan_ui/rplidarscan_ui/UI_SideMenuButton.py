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

if sys.version_info.minor >= 11:
    from typing import Self
else:
    from typing_extensions import Self

from typing import Callable

if sys.version_info.minor >= 11:
    from asyncio import TaskGroup
else:
    from taskgroup import TaskGroup

from .base import UI_Component
from .UI_Lidar import UI_Lidar

import dearpygui.dearpygui as dpg

class UI_SideMenuButton(UI_Component):
    """The UI consists of 4-5 buttons at the side, each of which has an associated tab / pane.
       This object associates a button and pane; details within panes are managed separately.
    """

    def __init__(self, ui, name: str) -> None:
        """Initiate, with name of button for reference."""
        self.ui = ui
        self.name: str = name

        self.btag = "button_" + name
        self.stag = "subwin_" + name
        self.ttag = self.ui.icon_texture[name]

        self.subwin = None
        self.sub_ui: UI_Component = None

        if self.name == "Home":
            self.ui.smb_Home = self

    def deselect(self):
        """Deselect / Hide the pane."""
        dpg.hide_item(self.subwin)
        self.ui.smb_active = None

    def select(self):
        """Select / Show the pane."""
        if self.ui.smb_active is not None:
            self.ui.smb_active.deselect()

        dpg.show_item(self.subwin)
        self.ui.smb_active = self

    @staticmethod
    def __callback_exit():
        """Callback: The application exit button has been pressed."""
        dpg.stop_dearpygui()

    @staticmethod
    def __callback_shutdown():
        """Callback: The shutdown button has been pressed."""
        self.ui.opts["shutdown"] = True
        dpg.stop_dearpygui()

    @staticmethod
    def __callback(sender, app_data, user_data: Self) -> None:
        """Callback: Side-button pressed."""
        user_data.select()

    def __add_large_button(self, label: str, callback: Callable):
        """Add a button at the side."""
        button = dpg.add_button(label=label, callback=callback, width=self.ui.subbtn_width, height=self.ui.subbtn_height)
        dpg.bind_item_font(button, self.ui.font_large)
        return button

    def add_button_and_window(self):
        """Add a button at the side with an associated tab / pane."""
        dpg.add_image_button(self.ttag, label=self.name, user_data=self, callback=UI_SideMenuButton.__callback,
                             width=self.ui.button_width, height=self.ui.button_height, tag=self.btag)

        with dpg.window(tag=self.stag, pos=(self.ui.subwin_x, self.ui.subwin_y), width=self.ui.subwin_width, height=self.ui.subwin_height,
                        no_title_bar=True, show=False) as subwin:
            self.subwin = subwin

            # TODO: Home
            # TODO: Settings

            if self.name == "Lidar": # TODO: This whole section needs a re-think; just create the pane here, add the sub_ui component later.
                self.sub_ui = UI_Lidar(self.ui, self.subwin)

            if self.name == "Shutdown":
                ui_exit = self.__add_large_button("Exit", UI_SideMenuButton.__callback_exit)
                if self.ui.opts.get("admin", False):
                    ui_shut = self.__add_large_button("Shutdown", UI_SideMenuButton.__callback_shutdown)
                    dpg.bind_item_theme(ui_shut, self.ui.theme_caution)

    def update(self):
        """Update if necessary."""
        if self.sub_ui is not None:
            return self.sub_ui.update()
        return True

    def set_task_group(self, tg):
        """Set the current asyncio event loop controller."""
        if self.sub_ui is not None:
            self.sub_ui.set_task_group(tg)

    def app_will_end(self):
        """Notification that the application is ending."""
        if self.sub_ui is not None:
            self.sub_ui.app_will_end()
