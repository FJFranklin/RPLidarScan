# Copyright 2026 Francis James Franklin
#
# With thanks to: https://github.com/m2-farzan/ros2-asyncio
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

import abc

from typing import Tuple, Dict

import numpy as np

class UI_Component(abc.ABC):
    @abc.abstractmethod
    def update(self) -> bool:
        """Update if necessary."""
        pass

    @abc.abstractmethod
    def set_task_group(self, tg):
        """Set the current asyncio event loop controller."""
        pass

    @abc.abstractmethod
    def app_will_end(self):
        """Notification that the application is ending."""
        pass

class Lidar2D_DataManager(abc.ABC):
    """An abstract base class for managing 2D lidar scan data
    """

    def __init__(self, scaling:float, radians:bool) -> None:
        """Initialise, specifying the scaling, i.e., 1 if distances are in metres, and whether angles are in radians."""
        self.__L2D_data_updated: bool = False
        self.__L2D_data_distances: Dict = {}
        self.__L2D_data_intensities: Dict = {}
        self.__L2D_data_scaling: float = scaling
        self.__L2D_data_radians: bool = radians

    @property
    def L2D_updated(self) -> bool:
        """True if the data has changed since last accessed."""
        return self.__L2D_data_updated

    @property
    def L2D_scaling(self) -> float:
        """The scaling, i.e., 1 if distances are in metres, 0.001 if in millimetres."""
        return self.__L2D_data_scaling

    @property
    def L2D_radians(self) -> bool:
        """The scaling, i.e., 1 if distances are in metres, 0.001 if in millimetres."""
        return self.__L2D_data_radians

    @property
    def L2D_data(self) -> Tuple[Dict, Dict]:
        """Gets the scan data as a tuple of dictionaries."""
        self.__L2D_data_updated: bool = False
        return self.__L2D_data_distances, self.__L2D_data_intensities

    @L2D_data.setter
    def L2D_data(self, data: Tuple[Dict, Dict]) -> None:
        """Sets the scan data as a tuple of dictionaries."""
        self.__L2D_data_updated: bool = True
        D, I = data
        self.__L2D_data_distances = D
        self.__L2D_data_intensities = I

    @abc.abstractmethod
    def L2D_update(self) -> bool:
        """Check to see if new lidar data is available; returns False if application should end."""
        pass

    @abc.abstractmethod
    def L2D_app_will_end(self) -> None:
        """Notification that the application is ending."""
        pass

    def L2D_analyse(self) -> None:
        count = len(self.__L2D_data_distances)

        if count < 3:
            print("no data")
            return
        else:
            print("count={c} ".format(c=count), end="")

        D = np.ndarray((count, 4))
        F = np.zeros((count, 1), dtype=np.uint8)

        for i, (k, v) in enumerate(self.__L2D_data_distances.items()):
            if self.__L2D_data_radians:
                D[i,0] = k
            else:
                D[i,0] = np.radians(k)
            if v is None:
                D[i,1] = 12.0
                F[i] = 0x01 # flag as out-of-range
            else:
                D[i,1] = v * self.__L2D_data_scaling
        print("#oor={o} ".format(o=np.sum(F)), end="")

        D[:,2] = D[:,1] * np.cos(D[:,0]) # x
        D[:,3] = D[:,1] * np.sin(D[:,0]) # y

        leftmost = np.argmin(D[:,2])
        F[leftmost] |= 0x02 # flag as a hull point
        print("left@{i} ".format(i=leftmost))
