# RPLidarScan UI

RPLidarScan is a tool for visualising scanning data from SLAMTEC's C1 LiDAR.

The UI operates both as a standalone tool that communicates directly with the lidar over serial:
```bash
python3 ui_serial.py
```
or as a ROS2 module that listens for '/scan' messages:
```bash
ros2 run rplidarscan_ui ui
```
For the latter, you can use [SLAMTEC's ROS2 lidar module](https://github.com/Slamtec/sllidar_ros2):
```bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py
```

The UI is currently a proof-of-concept application intended for small touchscreens.
There are 4 tabs / panes, and clicking the clock icon will show the lidar scan.
To exit, press the power icon and select exit.

**WARNING:** The application freezes on exit when using WSL2.
I believe this is a bug caused by DearPyGui trying to close the window on Windows.

## Requirements

You will need:
* [RPLidarC1](https://github.com/dsaadatmandi/rplidarc1)
```bash
pip install rplidarc1
```
* [DearPyGui](https://github.com/hoffstadt/DearPyGui) - availability depends on platform, but you can try:
```bash
pip install dearpygui
```
    * For the Jetson Orin Nano, you can build DearPyGui from source - instructions
[here](https://github.com/hoffstadt/DearPyGui/wiki/Local-Wheel) - but for the Jetson:
```bash
git clone --recursive https://github.com/hoffstadt/DearPyGui
cd DearPyGui
python3 -m setup bdist_wheel --plat-name manylinux_2_35_aarch64 --dist-dir dist
pip install dist/dearpygui-2.1.1-cp310-cp310-manylinux_2_35_aarch64.whl
```

* If using Python 3.10 (as is the case on the Jetson Orin Nano), you also need to install [taskgroup](https://github.com/graingert/taskgroup):
```bash
pip install taskgroup
```

## Build & Install

The rplidarscan source should be placed in your ROS2 workspace, i.e., 'ros2_ws/src/rplidarscan' and built using colcon:
```bash
colcon build --symlink-install
```
After building for the first time, don't forget to update the environment:
```bash
source install/setup.bash
```
