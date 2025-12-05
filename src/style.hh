/*
 * MIT License
 * 
 * Copyright (c) 2022 Evan Pezent & ImPlot Community
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __RPLIDAR_STYLE_HH__
#define __RPLIDAR_STYLE_HH__

#include "Fonts/Fonts.h"

static void s_style_and_colors() {
  static const ImVec4 bg_dark      = ImVec4(0.150f, 0.160f, 0.210f, 1.0f);
  static const ImVec4 bg_mid       = ImVec4(0.200f, 0.210f, 0.270f, 1.0f);
  static const ImVec4 accent_dark  = ImVec4(0.292f, 0.360f, 0.594f, 1.0f);
  static const ImVec4 accent_light = ImVec4(0.409f, 0.510f, 0.835f, 1.0f);
  static const ImVec4 active       = ImVec4(0.107f, 0.118f, 0.157f, 1.0f);
  static const ImVec4 attention    = ImVec4(0.821f, 1.000f, 0.000f, 1.0f);

  auto& style = ImGui::GetStyle();
  style.WindowPadding    = {6, 6};
  style.FramePadding     = {6, 3};
  style.CellPadding      = {6, 3};
  style.ItemSpacing      = {6, 6};
  style.ItemInnerSpacing = {6, 6};
  style.ScrollbarSize    = 16;
  style.GrabMinSize      = 8;
  style.WindowBorderSize = style.ChildBorderSize = style.PopupBorderSize = style.TabBorderSize = 0;
  style.FrameBorderSize  = 1;
  style.WindowRounding = style.ChildRounding = style.PopupRounding = style.ScrollbarRounding = style.GrabRounding = style.TabRounding = 4;

  ImVec4* colors = ImGui::GetStyle().Colors;
  colors[ImGuiCol_Text]                  = ImVec4(0.89f, 0.89f, 0.92f, 1.00f);
  colors[ImGuiCol_TextDisabled]          = ImVec4(0.38f, 0.45f, 0.64f, 1.00f);
  colors[ImGuiCol_WindowBg]              = bg_mid;
  colors[ImGuiCol_ChildBg]               = ImVec4(0.20f, 0.21f, 0.27f, 0.00f);
  colors[ImGuiCol_PopupBg]               = bg_mid;
  colors[ImGuiCol_Border]                = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
  colors[ImGuiCol_BorderShadow]          = ImVec4(0.00f, 0.00f, 0.00f, 0.06f);
  colors[ImGuiCol_FrameBg]               = ImVec4(1.00f, 1.00f, 1.00f, 0.02f);
  colors[ImGuiCol_FrameBgHovered]        = accent_light;
  colors[ImGuiCol_FrameBgActive]         = active;
  colors[ImGuiCol_TitleBg]               = accent_dark;
  colors[ImGuiCol_TitleBgActive]         = accent_dark;
  colors[ImGuiCol_TitleBgCollapsed]      = accent_dark;
  colors[ImGuiCol_MenuBarBg]             = accent_dark;
  colors[ImGuiCol_ScrollbarBg]           = bg_mid;
  colors[ImGuiCol_ScrollbarGrab]         = ImVec4(0.89f, 0.89f, 0.93f, 0.27f);
  colors[ImGuiCol_ScrollbarGrabHovered]  = accent_light;
  colors[ImGuiCol_ScrollbarGrabActive]   = active;
  colors[ImGuiCol_CheckMark]             = accent_dark;
  colors[ImGuiCol_SliderGrab]            = accent_dark;
  colors[ImGuiCol_SliderGrabActive]      = accent_light;
  colors[ImGuiCol_Button]                = accent_dark;
  colors[ImGuiCol_ButtonHovered]         = accent_light;
  colors[ImGuiCol_ButtonActive]          = active;
  colors[ImGuiCol_Header]                = accent_dark;
  colors[ImGuiCol_HeaderHovered]         = accent_light;
  colors[ImGuiCol_HeaderActive]          = active;
  colors[ImGuiCol_Separator]             = accent_dark;
  colors[ImGuiCol_SeparatorHovered]      = accent_light;
  colors[ImGuiCol_SeparatorActive]       = active;
  colors[ImGuiCol_ResizeGrip]            = accent_dark;
  colors[ImGuiCol_ResizeGripHovered]     = accent_light;
  colors[ImGuiCol_ResizeGripActive]      = active;
  colors[ImGuiCol_Tab]                   = ImVec4(1.00f, 1.00f, 1.00f, 0.02f);
  colors[ImGuiCol_TabHovered]            = accent_light;
  colors[ImGuiCol_TabActive]             = accent_dark;
  colors[ImGuiCol_TabUnfocused]          = ImVec4(0.24f, 0.23f, 0.29f, 1.00f);
  colors[ImGuiCol_TabUnfocusedActive]    = active;
  colors[ImGuiCol_PlotLines]             = accent_light;
  colors[ImGuiCol_PlotLinesHovered]      = active;
  colors[ImGuiCol_PlotHistogram]         = accent_light;
  colors[ImGuiCol_PlotHistogramHovered]  = active;
  colors[ImGuiCol_TableHeaderBg]         = accent_dark;
  colors[ImGuiCol_TableBorderStrong]     = ImVec4(1.00f, 1.00f, 1.00f, 0.10f);
  colors[ImGuiCol_TableBorderLight]      = ImVec4(1.00f, 1.00f, 1.00f, 0.02f);
  colors[ImGuiCol_TableRowBg]            = ImVec4(0, 0, 0, 0);
  colors[ImGuiCol_TableRowBgAlt]         = ImVec4(1.00f, 1.00f, 1.00f, 0.02f);
  colors[ImGuiCol_TextSelectedBg]        = accent_light;
  colors[ImGuiCol_DragDropTarget]        = attention;
  colors[ImGuiCol_NavHighlight]          = ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
  colors[ImGuiCol_NavWindowingHighlight] = ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
  colors[ImGuiCol_NavWindowingDimBg]     = ImVec4(0.80f, 0.80f, 0.80f, 0.20f);
  colors[ImGuiCol_ModalWindowDimBg]      = ImVec4(1.00f, 0.98f, 0.95f, 0.73f);
#ifdef IMGUI_HAS_DOCK
  colors[ImGuiCol_DockingPreview]        = ImVec4(0.85f, 0.85f, 0.85f, 0.28f);
  colors[ImGuiCol_DockingEmptyBg]        = ImVec4(0.38f, 0.38f, 0.38f, 1.00f);
#endif

  ImPlot::StyleColorsAuto();

  ImVec4* pcolors = ImPlot::GetStyle().Colors;
  pcolors[ImPlotCol_PlotBg]     = ImVec4(0, 0, 0, 0);
  pcolors[ImPlotCol_PlotBorder] = ImVec4(0, 0, 0, 0);
  pcolors[ImPlotCol_Selection]  = attention;
  pcolors[ImPlotCol_Crosshairs] = colors[ImGuiCol_Text];

  ImPlot::GetStyle().DigitalBitHeight = 20;

  auto& pstyle = ImPlot::GetStyle();
  pstyle.PlotPadding  = pstyle.LegendPadding      = {12, 12};
  pstyle.LabelPadding = pstyle.LegendInnerPadding = {6, 6};
  pstyle.LegendSpacing     = {10, 2};
  pstyle.AnnotationPadding = {4,2};

  const ImU32 Dracula[] = {4288967266, 4285315327, 4286315088, 4283782655, 4294546365, 4287429361, 4291197439, 4294830475, 4294113528, 4284106564 };
  ImPlot::GetStyle().Colormap = ImPlot::AddColormap("Dracula", Dracula, 10);
}

static void s_load_fonts(std::map<std::string,ImFont*>& Fonts, bool im_style) {
  ImGuiIO& io = ImGui::GetIO();

  // add fonts
  io.Fonts->Clear();

  ImFontConfig font_cfg;
  font_cfg.FontDataOwnedByAtlas = false;

  ImFontConfig icons_config;
  icons_config.MergeMode = true;
  icons_config.PixelSnapH = true;
  icons_config.GlyphMinAdvanceX = 14.0f;
  icons_config.GlyphOffset = ImVec2(0, 0);
  icons_config.OversampleH = 1;
  icons_config.OversampleV = 1;
  icons_config.FontDataOwnedByAtlas = false;

  static const ImWchar fa_ranges[] = {ICON_MIN_FA, ICON_MAX_FA, 0};

  if (im_style) {
    io.Fonts->AddFontDefault();
    io.Fonts->AddFontFromMemoryTTF(fa_solid_900_ttf, fa_solid_900_ttf_len, 14.0f, &icons_config, fa_ranges);
  }

  ImStrncpy(font_cfg.Name, "Roboto Bold", 40);
  Fonts[font_cfg.Name] = io.Fonts->AddFontFromMemoryTTF(Roboto_Bold_ttf, Roboto_Bold_ttf_len, 15.0f, &font_cfg);
  io.Fonts->AddFontFromMemoryTTF(fa_solid_900_ttf, fa_solid_900_ttf_len, 14.0f, &icons_config, fa_ranges);

  ImStrncpy(font_cfg.Name, "Roboto Italic", 40);
  Fonts[font_cfg.Name] = io.Fonts->AddFontFromMemoryTTF(Roboto_Italic_ttf, Roboto_Italic_ttf_len, 15.0f, &font_cfg);
  io.Fonts->AddFontFromMemoryTTF(fa_solid_900_ttf, fa_solid_900_ttf_len, 14.0f, &icons_config, fa_ranges);

  ImStrncpy(font_cfg.Name, "Roboto Regular", 40);
  Fonts[font_cfg.Name] = io.Fonts->AddFontFromMemoryTTF(Roboto_Regular_ttf, Roboto_Regular_ttf_len, 15.0f, &font_cfg);
  io.Fonts->AddFontFromMemoryTTF(fa_solid_900_ttf, fa_solid_900_ttf_len, 14.0f, &icons_config, fa_ranges);

  ImStrncpy(font_cfg.Name, "Roboto Mono Bold", 40);
  Fonts[font_cfg.Name] = io.Fonts->AddFontFromMemoryTTF(RobotoMono_Bold_ttf, RobotoMono_Bold_ttf_len, 15.0f, &font_cfg);
  io.Fonts->AddFontFromMemoryTTF(fa_solid_900_ttf, fa_solid_900_ttf_len, 14.0f, &icons_config, fa_ranges);

  ImStrncpy(font_cfg.Name, "Roboto Mono Italic", 40);
  Fonts[font_cfg.Name] = io.Fonts->AddFontFromMemoryTTF(RobotoMono_Italic_ttf, RobotoMono_Italic_ttf_len, 15.0f, &font_cfg);
  io.Fonts->AddFontFromMemoryTTF(fa_solid_900_ttf, fa_solid_900_ttf_len, 14.0f, &icons_config, fa_ranges);

  ImStrncpy(font_cfg.Name, "Roboto Mono Regular", 40);
  Fonts[font_cfg.Name] = io.Fonts->AddFontFromMemoryTTF(RobotoMono_Regular_ttf, RobotoMono_Regular_ttf_len, 15.0f, &font_cfg);
  io.Fonts->AddFontFromMemoryTTF(fa_solid_900_ttf, fa_solid_900_ttf_len, 14.0f, &icons_config, fa_ranges);
}

#endif /* ! __RPLIDAR_STYLE_HH__ */
