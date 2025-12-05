#ifndef __RPLIDAR_LIDAR_HH__
#define __RPLIDAR_LIDAR_HH__

extern int  lidar_connect(const char* device = "/dev/rplidar", unsigned long baud = 460800); // returns 0 on success
extern void lidar_disconnect();
extern bool lidar_scan_start();
extern void lidar_scan_stop();
extern int  lidar_scan_once();
extern void lidar_result(int index, float& angle, float& distance);

#endif /* ! __RPLIDAR_LIDAR_HH__ */
