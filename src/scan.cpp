/*
 * MIT License
 * 
 * Copyright (c) 2025 Francis James Franklin
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

#include <cstdio>
#include <cstring>
#include <cmath>
#include <csignal>

#include <string>
#include <map>

#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_stdlib.h>
#include "imgui_internal.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <implot.h>

#include "style.hh"
#include "lidar.hh"

class App {
private:
  ImVec4 ClearColor;                    // background clear color
  GLFWwindow* Window;                   // GLFW window handle
  std::map<std::string,ImFont*> Fonts;  // font map

public:
  App(std::string title, int w, int h, bool bImDemoStyle);

  virtual ~App();

  // Called at top of run
  virtual void Start() { }

  // Update, called once per frame.
  virtual void Update() { }

  // Runs the app.
  void Run();

  // Get window size
  ImVec2 GetWindowSize() const {
    int w, h;
    glfwGetWindowSize(Window, &w, &h);
    return ImVec2(w, h);
  }
};

static void s_glfw_error_callback(int error, const char* description) {
  fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

App::App(std::string title, int w, int h, bool bImDemoStyle) {
  // Setup window
  glfwSetErrorCallback(s_glfw_error_callback);
  if (!glfwInit())
    abort();

  // Create window with graphics context
  Window = glfwCreateWindow(w, h, title.c_str(), NULL, NULL);
  if (Window == NULL) {
    fprintf(stderr, "Failed to initialize GLFW window!\n");
    abort();
  }
  glfwMakeContextCurrent(Window);
  glfwSwapInterval(1);

  const GLubyte* renderer = glGetString(GL_RENDERER); 

  title +=  " - ";
  title += reinterpret_cast<char const *>(renderer);
  glfwSetWindowTitle(Window, title.c_str());

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGui_ImplGlfw_InitForOpenGL(Window, true);
  ImGui_ImplOpenGL3_Init();

  if (!bImDemoStyle) {
    ImGui::StyleColorsDark();
    ImPlot::StyleColorsDark();
  } else {
    ClearColor = ImVec4(0.15f, 0.16f, 0.21f, 1.00f);
    s_style_and_colors();
  }
  s_load_fonts(Fonts, !bImDemoStyle);
}

App::~App() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();
  glfwDestroyWindow(Window);
  glfwTerminate();
}

static bool ctrl_c_pressed = false;

void ctrlc(int) {
  ctrl_c_pressed = true;
}

void App::Run() {
  if (lidar_connect() < 0)
    return;

  ctrl_c_pressed = false;
  signal(SIGINT, ctrlc);

  lidar_scan_start();

  Start();

  // Main loop
  while (!glfwWindowShouldClose(Window) && !ctrl_c_pressed) {
    lidar_scan_once();
    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    Update();

    // Rendering
    ImGui::Render();

    int display_w, display_h;
    glfwGetFramebufferSize(Window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(ClearColor.x, ClearColor.y, ClearColor.z, ClearColor.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(Window);
  }
  lidar_scan_stop();
  lidar_disconnect();
}

class RPLidarScan : public App {
public:
  bool bLogarithmic;

  RPLidarScan(std::string title, int w, int h, bool bImDemoStyle) :
    App(title, w, h, bImDemoStyle),
    bLogarithmic(false)
  {
    // ...
  }

  virtual ~RPLidarScan();

  void Start() override {
    //
  }

  void Update() override;
};

static void draw_circle(const ImVec4& color, double radius, int points, const char* name = "##item") {
  auto circle = [](int idx, void* data) {
    double* params = (double*) data;
    double divisor = params[0];
    double radius  = params[1];
    double theta   = 2.0 * M_PI * idx / divisor;
    double x = radius * cos(theta);
    double y = radius * sin(theta);
    return ImPlotPoint(x, y);
  };

  double params[2] = { (double) (points - 1), radius };
  ImPlot::SetNextLineStyle(color);
  ImPlot::PlotLineG(name, circle, params, points);
}

void RPLidarScan::Update() {
  static const ImVec4 red(1, 0, 0, 1);
  static const ImVec4 amber(1, 0.75f, 0, 1);
  static const ImVec4 green(0, 1, 0, 1);
  static const ImVec4 blue(0, 0, 1, 1);

  ImGui::SetNextWindowSize(GetWindowSize());
  ImGui::SetNextWindowPos({0,0});

  ImGui::Begin("rplidar-scan", nullptr, ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize);

  int measurements = lidar_scan_once();

  if (ImPlot::BeginPlot("##Plot", ImVec2(-1,-1))) { // title(?) and frame size(?)
    ImPlot::SetupAxisLimits(ImAxis_X1, -13.0, 13.0, ImPlotCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, -13.0, 13.0, ImPlotCond_Always);

    draw_circle(green, 12, 2000);
    draw_circle(amber,  8, 1500);
    draw_circle(red,    4, 1000);

    for (int p = 0; p < measurements; p++) {
      float theta = 0;
      float radius = 0;
      lidar_result(p, theta, radius);
      if (radius < 0.05) continue; // LiDAR effective range is from 5cm to 12m
      if (bLogarithmic) {
	radius = 8 + 4 * log10(radius); // logarithmic scaling
      }
      if (radius > 12) {
	radius = 13;
      }
      ImVec4 color(radius < 6 ? 1 : 0, radius > 4 ? (radius > 12 ? 0 : 1) : 0, radius > 12 ? 1 : 0, 1);
      double coord[2];
      coord[0] = radius * sin(theta); // compass orientation
      coord[1] = radius * cos(theta);
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 3.0f, color, 0.0f); // fill, no edge
      ImPlot::PlotScatterG("##item", 
			   [](int idx, void* data) {
			     double* coord = (double*) data;
			     return ImPlotPoint(coord[0], coord[1]);
			   }, coord, 1);
    }
    ImPlot::EndPlot();
  }
  ImGui::End();
}

RPLidarScan::~RPLidarScan() {
  // ...
}

int main(int argc, char const *argv[]) {
  bool bLogarithmic = false;
  bool bImDemoStyle = false;

  for (int arg = 1; arg < argc; arg++) {
    if (strcmp(argv[arg], "--help") == 0 || strcmp(argv[arg], "-h") == 0) {
      fprintf(stdout, "RPLidarScan: usage: scan [-h|--help] [-l|--logarithmic] [-i|--implot-demo-style]\n");
      return 0;
    }
    if (strcmp(argv[arg], "--logarithmic") == 0 || strcmp(argv[arg], "-l") == 0) {
      bLogarithmic = true;
    }
    if (strcmp(argv[arg], "--implot-demo-style") == 0 || strcmp(argv[arg], "-i") == 0) {
      bImDemoStyle = true;
    }
  }

  RPLidarScan app("RPLidar Scan", 800, 800, bImDemoStyle); // provides window title

  app.bLogarithmic = bLogarithmic;
  app.Run();

  return 0;
}
