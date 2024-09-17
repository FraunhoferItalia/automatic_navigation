#
# Copyright 2024 Fraunhofer Italia Research
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
#
from __future__ import annotations
from typing import Dict, Any, Tuple
from vehicle_recognition import (
    Vehicle,
    VehicleKinematicType,
    SensorType,
)
from . import FilteringConfiguratorBase
import yaml


class RobotLocalizationConfigurator(FilteringConfiguratorBase):
    def __init__(self, info: Dict[str, Any], vehicle: Vehicle, file: str):
        super().__init__(info, vehicle, file)

    def configure(self) -> bool:
        configuration = {}
        with open(self.file, "w") as f:
            filtering = self.generate_robot_localization_configuration()
            try:
                configuration["**/"][filtering[0]] = filtering[1]
            except KeyError:
                configuration.update({"**/": {filtering[0]: filtering[1]}})
            except TypeError:
                configuration = {"**/": {filtering[0]: filtering[1]}}

            yaml.dump(configuration, f, sort_keys=False)
        return True

    def generate_robot_localization_configuration(self) -> Tuple[str, Any]:
        y_component = False
        # if the vehicle can have omnidirectional kinematics
        for kinematic in self.vehicle.kinematics:
            if kinematic.type is VehicleKinematicType.OMNIDIRECTIONAL:
                y_component = True
        n_imu = 0
        imus: Dict[str, Any] = {}
        for sensor in self.vehicle.sensors:
            if sensor.type is SensorType.IMU:
                imus[f"imu{n_imu}"] = sensor.name + "/data"
                imus[f"imu{n_imu}_config"] = [
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    True,
                    True,
                    True,
                    False,
                ]
                n_imu += 1
        if n_imu == 1:
            imus[f"imu0"] = "imu/data"

        robot_localization_configuration = (
            "ekf_filter_node",
            {
                "ros__parameters": {
                    "frequency": 400.0,
                    "two_d_mode": True,
                    "publish_acceleration": True,
                    "publish_tf": True,
                    "map_frame": self.info["map_frame"],
                    "odom_frame": self.info["odom_frame"],
                    "base_link_frame": self.vehicle.root.name,
                    "world_frame": self.info["odom_frame"],
                    "odom0": self.info["odom_topic"],
                    "odom0_config": [
                        False,
                        False,
                        False,
                        False,
                        False,
                        False,
                        True,
                        y_component,
                        False,
                        False,
                        False,
                        True,
                        False,
                        False,
                        False,
                    ],
                }
            },
        )
        robot_localization_configuration[1]["ros__parameters"].update(imus)

        return robot_localization_configuration
