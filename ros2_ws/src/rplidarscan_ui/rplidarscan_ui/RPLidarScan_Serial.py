# Copyright 2026 Francis James Franklin
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import os.path

if sys.version_info.minor >= 11:
    from typing import Self
else:
    from typing_extensions import Self

from typing import Tuple

import asyncio

if sys.version_info.minor >= 11:
    from asyncio import TaskGroup, run
else:
    from taskgroup import TaskGroup, run

from rplidarc1.scanner import RPLidar

from .base import Lidar2D_DataManager

class RPLidarScan_Serial(Lidar2D_DataManager):
    """Class to handle direct serial control of the RPLidar C1
    """

    @staticmethod
    def create_data_manager(device: str) -> Tuple[Self, bool]:
        """RPLidar C1 serial class creator."""
        return RPLidarScan_Serial(device), True

    def __init__(self, device: str) -> None:
        """Initiation, with the device name of the lidar."""
        super().__init__(0.001, False) # degrees and mm
        self.device: str = device
        self.task_group: TaskGroup = None
        self.lidar: RPLidar = None

        self.__quality = {}
        self.__distance = {}
        self.__last_angle = 0.0

    def set_task_group(self, task_group: TaskGroup) -> None:
        """Set the current asyncio event loop controller."""
        self.task_group = task_group

    def __lidar_begin(self) -> bool:
        """Try to connect to the lidar and start asynchronous data collection."""
        if not os.path.exists(self.device):
            print("RPLidarScan: RPLidar C1 device path '" + self.device + "' does not exist.")
        else:
            # Initialize the RPLidar with the appropriate port and baudrate
            try:
                self.lidar = RPLidar(self.device, 460800)
            except ConnectionError:
                print("RPLidarScan: Unable to connect to RPLidar C1 device.")
                self.lidar = None

        if self.lidar is not None:
            print("RPLidarScan: Connected to RPLidar C1 at device path '" + self.device + "'.")
            self.task_group.create_task(self.lidar.simple_scan(make_return_dict=False))
            return True

        return False

    @property
    def playing(self) -> bool:
        """True if the lidar is operating."""
        return self.lidar is not None

    def play(self) -> bool:
        """Start the lidar; returns False on error."""
        if self.lidar is None:
            return self.__lidar_begin()
        return True # already playing

    def pause(self) -> None:
        if self.lidar is not None:
            self.lidar.stop_event.set()
            self.lidar.reset()
            self.lidar = None

    def L2D_update(self) -> bool:
        """Check to see if new lidar data is available."""
        # Remove any data in the queue (dist vs angle gets saved to the dict internally)
        data_received = False
        if self.lidar is not None:
            while not self.lidar.output_queue.empty():
                measurement = self.lidar.output_queue.get_nowait()
                a = measurement["a_deg"]
                #print("{a}, ".format(a=a), end="")
                if a < 5.0 and self.__last_angle > 355.0 and len(self.__distance) > 2:
                    #print(len(self.__distance))
                    self.L2D_data = self.__distance, self.__quality
                    self.__quality = {}
                    self.__distance = {}
                self.__last_angle = a
                self.__quality[a]  = measurement["q"]
                self.__distance[a] = measurement["d_mm"]

        return True

    def L2D_app_will_end(self) -> None:
        """Notification that the application is ending."""
        self.pause()

def L2D_device_test(device: str) -> None:
    """Diagnostic test-run for RPLidar C1 at specified device path."""
    lidar = None
    if not os.path.exists(device):
        print("RPLidarScan: RPLidar C1 device path '" + device + "' does not exist.")
    else:
        # Initialize the RPLidar with the appropriate port and baudrate
        try:
            lidar = RPLidar(device, 460800)
        except ConnectionError:
            print("RPLidarScan: Unable to connect to RPLidar C1 device.")
            lidar = None

    if lidar is not None:
        print("RPLidarScan: Connected to RPLidar C1 at device path '" + device + "'.")

    if lidar is None:
        return

    # Test code from https://github.com/dsaadatmandi/rplidarc1

    async def process_scan_data():
        # Start a scan with dictionary output
        async with TaskGroup() as tg:
            # Create a task to stop scanning after 5 seconds
            tg.create_task(wait_and_stop(5, lidar.stop_event))

            # Create a task to process data from the queue
            tg.create_task(process_queue(lidar.output_queue, lidar.stop_event))

            # Start the scan with dictionary output
            tg.create_task(lidar.simple_scan(make_return_dict=True))

        # Access the scan data dictionary
        print(lidar.output_dict)

        # Reset the device
        lidar.reset()

    async def wait_and_stop(seconds, event):
        await asyncio.sleep(seconds)
        event.set()

    async def process_queue(queue, stop_event):
        while not stop_event.is_set():
            if not queue.empty():
                data = await queue.get()
                # Process the data
                print(f"Angle: {data['a_deg']}°, Distance: {data['d_mm']}mm, Quality: {data['q']}")
            else:
                await asyncio.sleep(0.1)

    # Run the example
    try:
        run(process_scan_data())
    except KeyboardInterrupt:
        lidar.reset()

