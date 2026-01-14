import dearpygui.dearpygui as dpg

def save_callback():
    print("Save Clicked")

def exit_callback():
    dpg.stop_dearpygui()
    print("Exit Clicked")

dpg.create_context()

class ImageButton(object):
    def __init__(self, name, icon):
        self.name = name
        self.icon = icon

        self.btag = "button_"  + name
        self.ttag = "texture_" + name

        width, height, channels, data = dpg.load_image(icon)

        dpg.add_static_texture(width=width, height=height, default_value=data, tag=self.ttag)

    def select(self):
        print("Image button '" + self.name + "' pressed.")

    @staticmethod
    def __callback(sender, app_data, user_data):
        user_data.select()

    def add_button(self, width, height):
        dpg.add_image_button(self.ttag, label=self.name, user_data=self, callback=ImageButton.__callback, width=width, height=height, tag=self.btag)

button_width  = 100
button_height = 100
button_defs = [
    [ "Home",     "icons/png/home-100x100.png" ],
    [ "Back",     "icons/png/arrow-left-100x100.png" ],
    [ "Clock",    "icons/png/clock-100x100.png" ],
    [ "Settings", "icons/png/settings-100x100.png" ],
    [ "Shutdown", "icons/png/system-shut-100x100.png" ]]
buttons = []

with dpg.texture_registry(show=False):
    for bdef in button_defs:
        buttons.append(ImageButton(*bdef))

with dpg.window(tag="Primary Window"):
    with dpg.group(horizontal=True):
        with dpg.group() as ui_sidebar:
            for b in buttons:
                b.add_button(button_width, button_height)
        with dpg.group():
            ui_save = dpg.add_button(label="Save", callback=save_callback)
            ui_exit = dpg.add_button(label="Exit", callback=exit_callback)
            dpg.add_text("Hello world")
            dpg.add_input_text(label="string")
            dpg.add_slider_float(label="float")

dpg.create_viewport(title='Elecrow 7\" Display', width=1024, height=600)
dpg.setup_dearpygui()

#Display the application-level window that everything sits inside
dpg.show_viewport()
#dpg.toggle_viewport_fullscreen()

#The primary window fills the whole viewport
dpg.set_primary_window("Primary Window", True)

#Render Loop
dpg.start_dearpygui() # to exit loop, use dpg.stop_dearpygui()
#Equivalently:
#while dpg.is_dearpygui_running():
#    dpg.render_dearpygui_frame()

dpg.destroy_context()
