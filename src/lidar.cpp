/*
 * Copyright (c) 2025 Francis James Franklin
 *   This code is derived from ultra_simple in SLAMTEC's RPLidar SDK:
 *     https://github.com/Slamtec/rplidar_sdk
 */
/*
 *  SLAMTEC LIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <unistd.h>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

#include "lidar.hh"

using namespace sl;

static ILidarDriver* drv = 0;

static bool checkSLAMTECLIDARHealth() {
  sl_lidar_response_device_health_t healthinfo;

  sl_result op_result = drv->getHealth(healthinfo);
  if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
    fprintf(stdout, "lidar_connect: health check: SLAMTEC Lidar health status: %d\n", healthinfo.status);

    if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
      fprintf(stderr, "lidar_connect: health check: SLAMTEC Lidar internal error detected. Please reboot the device to retry.\n");
      // enable the following code if you want slamtec lidar to be reboot by software
      // drv->reset();
      return false;
    }
  } else {
    fprintf(stderr, "lidar_connect: health check: Unable to retrieve the lidar health code: %x\n", op_result);
    return false;
  }
  return true;
}

int lidar_connect(const char* device, unsigned long baud) {
  if (drv) {
    fprintf(stderr, "lidar_connect: Already connected.\n");
    return -1;
  }

  if (!device) {
    device = "/dev/ttyUSB0";
    fprintf(stdout, "lidar_connect: Trying device %s\n", device);
  }

  // create the driver instance
  drv = *createLidarDriver();
  if (!drv) {
    fprintf(stderr, "lidar_connect: Unable to create driver (insufficent memory).\n");
    return -2;
  }

  sl_lidar_response_device_info_t devinfo;

  bool connectSuccess = false;

  IChannel* _channel = (*createSerialPortChannel(device, (sl_u32) baud));
  if (SL_IS_OK((drv)->connect(_channel))) {
    sl_result op_result = drv->getDeviceInfo(devinfo);
	
    if (SL_IS_OK(op_result)) {
      connectSuccess = true;
    } else {
      delete drv;
      drv = NULL;
    }
  }

  if (!connectSuccess) {
    fprintf(stderr, "lidar_connect: Unable to bind to the specified serial port %s.\n", device);
    if(drv) {
        delete drv;
        drv = NULL;
    }
    return -3;
  }

  // print out the device serial number, firmware and hardware version number..
  fprintf(stdout, "lidar_connect: device info:\n  SLAMTEC LIDAR S/N: ");
  for (int pos = 0; pos < 16 ;++pos) {
    fprintf(stdout, "%02X", devinfo.serialnum[pos]);
  }
  fprintf(stdout, "\n  Firmware Ver: %d.%02d\n  Hardware Rev: %d\n",
	  devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF, (int) devinfo.hardware_version);

  // check health...
  if (!checkSLAMTECLIDARHealth()) {
    fprintf(stderr, "lidar_connect: Failed health check!\n");
    if(drv) {
        delete drv;
        drv = NULL;
    }
    return -4;
  }
  return 0;
}

const size_t count_max = 520;
static sl_lidar_response_measurement_node_hq_t nodes[count_max];

int lidar_scan_once() {
  size_t count = count_max;

  sl_result op_result = drv->grabScanDataHq(nodes, count);
  if (SL_IS_OK(op_result)) {
    //fprintf(stdout, "lidar_scan_once: read %lu/%lu\n", (unsigned long) count, (unsigned long) count_max);
  } else {
    count = 0;
  }
  return (int) count;
}

void lidar_result(int index, float& angle, float& distance) {
  if (index < (int) count_max) {
    angle    = nodes[index].angle_z_q14 * M_PI_2 / 16384.f;
    distance = nodes[index].dist_mm_q2 / 4000.0f;
  } else {
    angle = 0;
    distance = 0;
  }
}

bool lidar_scan_start() {
  if (drv) {
    drv->setMotorSpeed();
    drv->startScan(0, 1);
    return true;
  }
  return false;
}

void lidar_scan_stop() {
  if (drv) {
    drv->stop();
    usleep(200000);
    drv->setMotorSpeed(0);
  }
}

void lidar_disconnect() {
  if (drv) {
    lidar_scan_stop();

    delete drv;
    drv = 0;
  }
}
