# RPLidarScan
RPLidarScan is a tool for visualising scanning data from SLAMTEC's C1 LiDAR.
It uses OpenGL via [Dear ImGui](https://github.com/ocornut/imgui) and [ImPlot](https://github.com/epezent/implot) for graphics and plotting,
and SLAMTEC's [RPLidar SDK](https://github.com/Slamtec/rplidar_sdk) for interfacing with the LiDAR.
RPLidarScan has been developed and tested on a Jetson Orin Nano (Ubuntu 22.04) and on Raspberry Pi 5 (Pi OS / debian Bookworm).

## Installation

In the same directory as the RPLidarScan source:

```bash
git clone https://github.com/ocornut/imgui.git
git clone https://github.com/epezent/implot.git
git clone https://github.com/epezent/implot_demos.git
git clone https://github.com/Slamtec/rplidar_sdk.git
```

You will need the GLFW's developer package too. For debian-based distros:

```bash
sudo apt-get install libglfw3-dev
```

and if this the first time you've used the LiDAR, you will need to copy the udev rule and restart udev:

```bash
sudo cp RPLidarScan/etc/01-rplidar.rules /etc/udev/rules.d/
sudo service udev reload
sudo service udev restart
```

The next time you plug in the LiDAR, it should appear as `/dev/ttyUSB0`, and also appear as `/dev/rplidar`.

Finally, to build RPLidarScan:

```bash
mkdir -p RPLidarScan/build
cd RPLidarScan/build
cmake ..
cmake --build . --config Release
```

If all goes well, you should have an application called `scan`.
