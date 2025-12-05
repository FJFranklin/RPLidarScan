cmake_minimum_required(VERSION 3.11.0)
project(RPLidarScan VERSION 0.1.0)

find_package(OpenGL REQUIRED)

set(RPLIDARSDK_HEADERS
  ../rplidar_sdk/sdk/include/rplidar_cmd.h
  ../rplidar_sdk/sdk/include/rplidar_driver.h
  ../rplidar_sdk/sdk/include/rplidar.h
  ../rplidar_sdk/sdk/include/rplidar_protocol.h
  ../rplidar_sdk/sdk/include/rptypes.h
  ../rplidar_sdk/sdk/include/sl_crc.h
  ../rplidar_sdk/sdk/include/sl_lidar_cmd.h
  ../rplidar_sdk/sdk/include/sl_lidar_driver.h
  ../rplidar_sdk/sdk/include/sl_lidar_driver_impl.h
  ../rplidar_sdk/sdk/include/sl_lidar.h
  ../rplidar_sdk/sdk/include/sl_lidar_protocol.h
  ../rplidar_sdk/sdk/include/sl_types.h
)
set(RPLIDARSDK_SRC
  ../rplidar_sdk/sdk/src/rplidar_driver.cpp
  ../rplidar_sdk/sdk/src/sdkcommon.h
  ../rplidar_sdk/sdk/src/sl_async_transceiver.cpp
  ../rplidar_sdk/sdk/src/sl_async_transceiver.h
  ../rplidar_sdk/sdk/src/sl_crc.cpp
  ../rplidar_sdk/sdk/src/sl_lidar_driver.cpp
  ../rplidar_sdk/sdk/src/sl_lidarprotocol_codec.cpp
  ../rplidar_sdk/sdk/src/sl_lidarprotocol_codec.h
  ../rplidar_sdk/sdk/src/sl_serial_channel.cpp
  ../rplidar_sdk/sdk/src/sl_tcp_channel.cpp
  ../rplidar_sdk/sdk/src/sl_udp_channel.cpp
)
set(RPLIDARSDK_ARCH
  ../rplidar_sdk/sdk/src/arch/linux/arch_linux.h
  ../rplidar_sdk/sdk/src/arch/linux/net_serial.cpp
  ../rplidar_sdk/sdk/src/arch/linux/net_serial.h
  ../rplidar_sdk/sdk/src/arch/linux/net_socket.cpp
  ../rplidar_sdk/sdk/src/arch/linux/thread.hpp
  ../rplidar_sdk/sdk/src/arch/linux/timer.cpp
  ../rplidar_sdk/sdk/src/arch/linux/timer.h
)
set(RPLIDARSDK_DATA
  ../rplidar_sdk/sdk/src/dataunpacker/dataunnpacker_commondef.h
  ../rplidar_sdk/sdk/src/dataunpacker/dataunnpacker_internal.h
  ../rplidar_sdk/sdk/src/dataunpacker/dataunpacker.cpp
  ../rplidar_sdk/sdk/src/dataunpacker/dataunpacker.h
  ../rplidar_sdk/sdk/src/dataunpacker/dataupacker_namespace.h
  ../rplidar_sdk/sdk/src/dataunpacker/unpacker/handler_capsules.cpp
  ../rplidar_sdk/sdk/src/dataunpacker/unpacker/handler_capsules.h
  ../rplidar_sdk/sdk/src/dataunpacker/unpacker/handler_hqnode.cpp
  ../rplidar_sdk/sdk/src/dataunpacker/unpacker/handler_hqnode.h
  ../rplidar_sdk/sdk/src/dataunpacker/unpacker/handler_normalnode.cpp
  ../rplidar_sdk/sdk/src/dataunpacker/unpacker/handler_normalnode.h
)
set(RPLIDARSDK_HAL
  ../rplidar_sdk/sdk/src/hal/abs_rxtx.h
  ../rplidar_sdk/sdk/src/hal/assert.h
  ../rplidar_sdk/sdk/src/hal/byteops.h
  ../rplidar_sdk/sdk/src/hal/byteorder.h
  ../rplidar_sdk/sdk/src/hal/event.h
  ../rplidar_sdk/sdk/src/hal/locker.h
  ../rplidar_sdk/sdk/src/hal/socket.h
  ../rplidar_sdk/sdk/src/hal/thread.cpp
  ../rplidar_sdk/sdk/src/hal/thread.h
  ../rplidar_sdk/sdk/src/hal/types.h
  ../rplidar_sdk/sdk/src/hal/util.h
  ../rplidar_sdk/sdk/src/hal/waiter.h
)
include_directories(../rplidar_sdk/sdk/src/ ../rplidar_sdk/sdk/include)

add_library(rplidar_sdk ${RPLIDARSDK_HEADERS} ${RPLIDARSDK_SRC} ${RPLIDARSDK_ARCH} ${RPLIDARSDK_DATA} ${RPLIDARSDK_HAL})

set(IMGUI_HEADERS
  ../imgui/imconfig.h
  ../imgui/imgui_internal.h
  ../imgui/imgui.h
  ../imgui/imstb_rectpack.h
  ../imgui/imstb_textedit.h
  ../imgui/imstb_truetype.h    
  ../imgui/backends/imgui_impl_glfw.h
  ../imgui/backends/imgui_impl_opengl3.h
  ../imgui/backends/imgui_impl_opengl3_loader.h
  ../imgui/misc/cpp/imgui_stdlib.h
)
set(IMGUI_SRC
  ../imgui/imgui_demo.cpp
  ../imgui/imgui_draw.cpp
  ../imgui/imgui_tables.cpp
  ../imgui/imgui_widgets.cpp
  ../imgui/imgui.cpp
  ../imgui/backends/imgui_impl_glfw.cpp
  ../imgui/backends/imgui_impl_opengl3.cpp
  ../imgui/misc/cpp/imgui_stdlib.cpp
)
include_directories(../imgui/ ../imgui/backends ../imgui/misc/cpp)

add_library(imgui ${IMGUI_HEADERS} ${IMGUI_SRC})
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)
target_compile_definitions(imgui PRIVATE IMGUI_DLL_EXPORT)

set(IMPLOT_HEADERS
  ../implot/implot.h
  ../implot/implot_internal.h
)
set(IMPLOT_SRC
  ../implot/implot.cpp
  ../implot/implot_items.cpp
  ../implot/implot_demo.cpp
)
include_directories(../implot)

#if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../implot/backends")
#list(APPEND IMPLOT_HEADERS ../implot/backends/implot_backend.h ../implot/backends/implot_impl_opengl3.h)
#list(APPEND IMPLOT_SRC ../implot/backends/implot_backend.h ../implot/backends/implot_impl_opengl3.cpp)
#endif()

add_library(implot ${IMPLOT_HEADERS} ${IMPLOT_SRC})
target_link_libraries(implot PUBLIC imgui)
target_compile_definitions(implot PUBLIC IMPLOT_DEBUG IMPLOT_DLL_EXPORT IMPLOT_BACKEND_ENABLE_OPENGL3)
#set_property(TARGET implot PROPERTY CXX_STANDARD 17)
target_compile_options(implot PRIVATE -Wall -Wextra -pedantic -Werror -Ofast)

add_library(robotofonts
  ../implot_demos/common/Fonts/Fonts.h
  ../implot_demos/common/Fonts/FontAwesome5.cpp
  ../implot_demos/common/Fonts/RobotoRegular.cpp
  ../implot_demos/common/Fonts/RobotoBold.cpp
  ../implot_demos/common/Fonts/RobotoItalic.cpp
  ../implot_demos/common/Fonts/RobotoMonoRegular.cpp
  ../implot_demos/common/Fonts/RobotoMonoBold.cpp
  ../implot_demos/common/Fonts/RobotoMonoItalic.cpp
)
include_directories(../implot_demos/common)
#target_compile_features(robotofonts PRIVATE cxx_std_17)

add_executable(scan
  src/scan.cpp
  src/style.hh
  src/lidar.hh
  src/lidar.cpp
)
target_link_libraries(scan robotofonts implot rplidar_sdk)
