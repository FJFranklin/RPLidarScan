import time
import math
import argparse

import dearpygui.dearpygui as dpg

parser = argparse.ArgumentParser(description="Touchscreen GUI.")

parser.add_argument('--target',     help='Target display (Elecrow, HyperPixel) [Elecrow].', default='Elecrow', choices=['Elecrow', 'HyperPixel'])
parser.add_argument('--fullscreen', help='Enter fullscreen display mode.',                  action='store_true')

args = parser.parse_args()

# Can we get this padding from DearPyGui somehow?
padding = 10

if args.target == 'Elecrow':
    display_title  = 'Elecrow 7\" Display'
    display_width  = 1024
    display_height =  600
    button_width   =  100
    fontsize_small =   25
else:
    display_title  = 'HyperPixel 4.0 Display'
    display_width  =  800
    display_height =  480
    button_width   =   80
    fontsize_small =   20

# Square buttons, so
button_height  = button_width

# Main window for application
subwin_x       = button_width + 25
subwin_y       = padding
subwin_width   = display_width  - subwin_x - padding
subwin_height  = display_height - subwin_y - padding

# Font for, e.g., large buttons
fontsize_large = fontsize_small * 2

# Side sub-menu
subwin_inset_x = subwin_width - button_width - 2 * padding
subwin_inset_y = padding

# Main application window, not including inset
canvas_width  = subwin_inset_x
canvas_height = subwin_height - 2 * padding
canvas_x   = int(canvas_width  / 2)
canvas_y   = int(canvas_height / 2)
canvas_min = min(canvas_x, canvas_y)

# Large button
subbtn_width  =  subwin_width - 2 * padding
subbtn_height =  fontsize_large + padding

icon_prefix = "icons/png/"
icon_suffix = "-{w}x{h}.png".format(w=button_width, h=button_height)

class UI_Lidar(object):
    @staticmethod
    def __callback_play_pause(sender, app_data, user_data):
        if user_data.playing:
            user_data.pause()
        else:
            user_data.play()

    def play(self):
        if self.playing:
            return
        self.playing = True # TODO
        if self.playing:
            dpg.configure_item(self.pp_btag, texture_tag=icon_texture["Pause"])

    def pause(self):
        if not self.playing:
            return
        self.playing = False # TODO
        if not self.playing:
            dpg.configure_item(self.pp_btag, texture_tag=icon_texture["Play"])

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

    def __init__(self, subwin):
        self.subwin = subwin
        self.zoom = 12
        self.playing = False

        inset = dpg.add_group(parent=subwin, pos=(subwin_inset_x, subwin_inset_y))

        self.pp_ttag = icon_texture["Play"]
        self.pp_btag = 'b' + self.pp_ttag
        dpg.add_image_button(self.pp_ttag, label="Play", user_data=self, callback=UI_Lidar.__callback_play_pause,
                             width=button_width, height=button_height, tag=self.pp_btag, parent=inset)

        dpg.add_spacer(height=button_height/2, parent=inset)

        self.zi_ttag = icon_texture["Zoom In"]
        self.zi_btag = 'b' + self.zi_ttag
        dpg.add_image_button(self.zi_ttag, label="Zoom In", user_data=self, callback=UI_Lidar.__callback_zoom_in,
                             width=button_width, height=button_height, tag=self.zi_btag, parent=inset)

        self.zz_btag = "bRangeText"
        dpg.add_text("Range: XXm", parent=inset, tag=self.zz_btag)
        dpg.bind_item_font(self.zz_btag, font_small)

        self.zo_ttag = icon_texture["Zoom Out"]
        self.zo_btag = 'b' + self.zo_ttag
        dpg.add_image_button(self.zo_ttag, label="Zoom Out", user_data=self, callback=UI_Lidar.__callback_zoom_out,
                             width=button_width, height=button_height, tag=self.zo_btag, parent=inset)

        self.set_zoom(12)

        with dpg.drawlist(parent=self.subwin, width=subwin_width, height=subwin_height-20) as dl:
            self.dl = dl
            self.draw_grid()
            self.line = dpg.draw_line((10, 10), (100, 100), color=(255, 0, 0, 255), thickness=1)

    def m_to_px(self, m):
        return m / float(self.zoom) * canvas_min
        
    def draw_grid(self):
        px_o = (canvas_x, canvas_y)
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

    def update(self):
        cr = canvas_min
        tp = 10 * time.process_time()
        cx = canvas_x + cr * math.cos(tp)
        cy = canvas_y + cr * math.sin(tp)

        #How to move an existing line:
        #dpg.configure_item(self.line, p1=(x0,y0), p2=(cx,cy))

        #How to redo a whole drawing:
        dpg.delete_item(self.dl)

        with dpg.drawlist(parent=self.subwin, width=subwin_width, height=subwin_height-20) as dl:
            self.dl = dl
            self.draw_grid()
            self.line = dpg.draw_line((canvas_x, canvas_y), (cx, cy), color=(255, 0, 0, 255), thickness=1)

class SideMenuButton(object):
    smb_active = None
    smb_Home = None

    def __init__(self, name):
        self.name = name

        self.btag = "button_" + name
        self.stag = "subwin_" + name
        self.ttag = icon_texture[name]

        self.subwin = None
        self.sub_ui = None

        if self.name == "Home":
            SideMenuButton.smb_Home = self

    def deselect(self):
        dpg.hide_item(self.subwin)
        SideMenuButton.smb_active = None

    def select(self):
        if SideMenuButton.smb_active is not None:
            SideMenuButton.smb_active.deselect()

        dpg.show_item(self.subwin)
        SideMenuButton.smb_active = self

    @staticmethod
    def __callback_save(): # demo code
        print("Saving...")

    @staticmethod
    def __callback_exit():
        dpg.stop_dearpygui()

    @staticmethod
    def __callback(sender, app_data, user_data):
        user_data.select()

    def __add_large_button(self, label, callback):
        button = dpg.add_button(label=label, callback=callback, width=subbtn_width, height=subbtn_height)
        dpg.bind_item_font(button, font_large)
        return button

    def add_button_and_window(self):
        dpg.add_image_button(self.ttag, label=self.name, user_data=self, callback=SideMenuButton.__callback,
                             width=button_width, height=button_height, tag=self.btag)

        with dpg.window(tag=self.stag, pos=(subwin_x, subwin_y), width=subwin_width, height=subwin_height,
                        no_title_bar=True, show=False) as subwin:
            self.subwin = subwin

            if self.name == "Home":
                # demo code
                ui_save = dpg.add_button(label="Save", callback=SideMenuButton.__callback_save)
                dpg.add_text("Hello world")
                dpg.add_input_text(label="string")
                dpg.add_slider_float(label="float")

            if self.name == "Lidar":
                self.sub_ui = UI_Lidar(self.subwin)

            if self.name == "Shutdown":
                ui_exit = self.__add_large_button("Exit", SideMenuButton.__callback_exit)
                dpg.bind_item_theme(ui_exit, theme_caution)

    def update(self):
        if self.sub_ui is not None:
            self.sub_ui.update()

# Start by creating DearPyGui context
dpg.create_context()

# Load icon images as textures; crossref with label-ttag dictionary
icon_defs = [
    [ "Home",     "home" ],
    [ "Back",     "arrow-left" ],
    [ "Lidar",    "clock" ],
    [ "Settings", "settings" ],
    [ "Shutdown", "system-shut" ],
    [ "Play",     "play" ],
    [ "Pause",    "pause" ],
    [ "Zoom In",  "zoom-in" ],
    [ "Zoom Out", "zoom-out" ]]
icon_texture = {}

with dpg.texture_registry(show=False):
    for icon_def in icon_defs:
        label, icon = icon_def

        filename = icon_prefix + icon + icon_suffix
        image = dpg.load_image(filename)
        if image is None:
            print("Error: failed to load image from '" + filename + "'")
        width, height, channels, data = image

        ttag = "tt_" + label
        dpg.add_static_texture(width=width, height=height, default_value=data, tag=ttag)

        icon_texture[label] = ttag

# Add scaled fonts
with dpg.font_registry():
    font_small = dpg.add_font("fonts/NotoSerifCJKjp-Medium.otf", fontsize_small)
    font_large = dpg.add_font("fonts/NotoSerifCJKjp-Medium.otf", fontsize_large)

# Create custom theme
with dpg.theme() as theme_caution:
    with dpg.theme_component(dpg.mvAll):
        dpg.add_theme_color(dpg.mvThemeCol_Button, (255, 0, 0), category=dpg.mvThemeCat_Core)

# Create side menu buttons and associated subwin panes
sidemenu_button_defs = [ "Home", "Lidar", "Settings", "Shutdown" ]
sidemenu_buttons = []

for bdef in sidemenu_button_defs:
    sidemenu_buttons.append(SideMenuButton(bdef))

with dpg.window(tag="Primary Window"):
    #dpg.bind_font(font_large)
    with dpg.group() as ui_sidebar:
        for b in sidemenu_buttons:
            b.add_button_and_window()

# Create viewport and setup
dpg.create_viewport(title=display_title, width=display_width, height=display_height)
dpg.setup_dearpygui()

#Display the application-level window that everything sits inside
dpg.show_viewport()
if args.fullscreen:
    dpg.toggle_viewport_fullscreen()

#The primary window fills the whole viewport
dpg.set_primary_window("Primary Window", True)

#Default to Home
SideMenuButton.smb_Home.select()

#Render Loop
#dpg.start_dearpygui() # to exit loop, use dpg.stop_dearpygui()
#Equivalently:
while dpg.is_dearpygui_running():
    for b in sidemenu_buttons:
        b.update()
    dpg.render_dearpygui_frame()
    time.sleep(0.001)

dpg.destroy_context()
