import argparse

import dearpygui.dearpygui as dpg

parser = argparse.ArgumentParser(description="Touchscreen GUI.")

parser.add_argument('--target',     help='Target display (Elecrow, HyperPixel) [Elecrow].', default='Elecrow', choices=['Elecrow', 'HyperPixel'])
parser.add_argument('--fullscreen', help='Enter fullscreen display mode.',                  action='store_true')

args = parser.parse_args()

if args.target == 'Elecrow':
    display_title  = 'Elecrow 7\" Display'
    display_width  = 1024
    display_height =  600
    button_width   =  100
    button_height  =  100
    subwin_x       =  125
    subwin_y       =    9
    subwin_width   =  890
    subwin_height  =  582
else:
    display_title  = 'HyperPixel 4.0 Display'
    display_width  =  800
    display_height =  480
    button_width   =   80
    button_height  =   80
    subwin_x       =  105
    subwin_y       =    9
    subwin_width   =  686
    subwin_height  =  462

icon_prefix = "icons/png/"
icon_suffix = "-{w}x{h}.png".format(w=button_width, h=button_height)

def save_callback():
    print("Save Clicked")

def exit_callback():
    dpg.stop_dearpygui()
    print("Exit Clicked")

dpg.create_context()

class ImageButton(object):
    active_IB = None

    def __init__(self, name, icon):
        self.name = name
        self.icon = icon

        self.btag = "button_"  + name
        self.stag = "subwin_"  + name
        self.ttag = "texture_" + name

        filename = icon_prefix + icon + icon_suffix
        image = dpg.load_image(filename)
        if image is None:
            print("ImageButton: failed to load image from '" + filename + "'")
        width, height, channels, data = image

        dpg.add_static_texture(width=width, height=height, default_value=data, tag=self.ttag)

        self.subwin = None

    def deselect(self):
        print("Image button '" + self.name + "' deselected.")
        dpg.hide_item(self.subwin)
        ImageButton.active_IB = None

    def select(self):
        print("Image button '" + self.name + "' pressed.")
        if ImageButton.active_IB is not None:
            ImageButton.active_IB.deselect()

        dpg.show_item(self.subwin)
        ImageButton.active_IB = self

    @staticmethod
    def __callback(sender, app_data, user_data):
        user_data.select()

    def add_button_and_window(self):
        dpg.add_image_button(self.ttag, label=self.name, user_data=self, callback=ImageButton.__callback,
                             width=button_width, height=button_height, tag=self.btag)

        with dpg.window(tag=self.stag, pos=(subwin_x, subwin_y), width=subwin_width, height=subwin_height,
                        no_title_bar=True, show=False) as subwin:
            self.subwin = subwin

            if self.name == "Shutdown":
                ui_save = dpg.add_button(label="Save", callback=save_callback)
                ui_exit = dpg.add_button(label="Exit", callback=exit_callback)
                dpg.add_text("Hello world")
                dpg.add_input_text(label="string")
                dpg.add_slider_float(label="float")

button_defs = [
    [ "Home",     "home" ],
    [ "Back",     "arrow-left" ],
    [ "Clock",    "clock" ],
    [ "Settings", "settings" ],
    [ "Shutdown", "system-shut" ]]
buttons = []

with dpg.texture_registry(show=False):
    for bdef in button_defs:
        buttons.append(ImageButton(*bdef))

with dpg.window(tag="Primary Window"):
    with dpg.group() as ui_sidebar:
        for b in buttons:
            b.add_button_and_window()

dpg.create_viewport(title=display_title, width=display_width, height=display_height)
dpg.setup_dearpygui()

#Display the application-level window that everything sits inside
dpg.show_viewport()
if args.fullscreen:
    dpg.toggle_viewport_fullscreen()

#The primary window fills the whole viewport
dpg.set_primary_window("Primary Window", True)

buttons[0].select()

#Render Loop
dpg.start_dearpygui() # to exit loop, use dpg.stop_dearpygui()
#Equivalently:
#while dpg.is_dearpygui_running():
#    dpg.render_dearpygui_frame()

dpg.destroy_context()
