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
from . import LocalizationConfiguratorBase
from typing import Dict, Any, List, Tuple
from vehicle_recognition import Vehicle, SensorType
import yaml


class NeoLocalicazation2Configurator(LocalizationConfiguratorBase):
    def __init__(self, info: Dict[str, Any], vehicle: Vehicle, file: str):
        super().__init__(info, vehicle, file)

    def configure(self) -> bool:
        configuration = {}
        with open(self.file, "w") as f:
            lasers = []
            pointclouds = []
            for sensor in self.vehicle.sensors:
                if sensor.type is SensorType.LASER:
                    lasers += [sensor.name]
                if sensor.type is SensorType.POINTCLOUD:
                    pointclouds += [sensor.name]
            # TODO: implement a way to use pointclouds
            if not lasers or not pointclouds:
                print(
                    "LOCALIZATION WARNING: No laserscanners found, Neo Localization2 tool requires a laserscanner topic."
                )

            localization = self.generate_neo_localization2_configuration(
                lasers, pointclouds
            )
            try:
                configuration["**/"][localization[0]] = localization[1]
            except KeyError:
                configuration.update({"**/": {localization[0]: localization[1]}})
            except TypeError:
                configuration = {"**/": {localization[0]: localization[1]}}
            yaml.dump(configuration, f, sort_keys=False)
        return True

    def generate_neo_localization2_configuration(
        self, lasers: List[str] = [], pointclouds: List[str] = []
    ) -> Tuple[str, Any]:
        if lasers:
            laserscan_topic = lasers[0]

        try:
            laserscan_topic = self.info["laserscan_topic"]
        except KeyError:
            if lasers:
                print(
                    "LOCALIZATION WARNING: Laser merge topic not specified. First laser name will be taken"
                )
            else:
                print(
                    "LOCALIZATION ERROR: Laser topic not specified. Neo Localization2 works only with a laser scanner topic"
                )
                return ("neo_localization2_node", {"ros__parameters": {}})

        neo_localization2_configuration = (
            "neo_localization2_node",
            {
                "ros__parameters": {
                    # these settings where copied from here: https://github.com/neobotix/neo_mpo_700-2/blob/rolling/configs/navigation/navigation.yaml
                    "base_frame": self.vehicle.root.name,
                    "odom_frame": self.info["odom_frame"],
                    # exponential low pass gain for localization update (0 to 1)
                    #   (higher gain means odometry is less used / relied on)
                    "update_gain": 0.5,
                    # time based confidence gain when in 2D / 1D mode
                    "confidence_gain": 0.01,
                    # how many particles (samples) to spread (per update)
                    "sample_rate": 10,
                    # localization update rate [ms]
                    "loc_update_time": 100,
                    # map tile update rate [1/s]
                    "map_update_rate": 0.5,
                    # map tile size in pixels
                    "map_size": 1000,
                    # how often to downscale (half) the original map
                    "map_downscale": 0,
                    # how many 3x3 gaussian smoothing iterations are applied to the map
                    "num_smooth": 5,
                    # minimum score for valid localization (otherwise 0D mode)
                    #    higher values make it go into 0D mode earlier
                    "min_score": 0.2,
                    # odometry error in x and y [m/m] [1]
                    #    how fast to increase particle spread when in 1D / 0D mode
                    "odometry_std_xy": 0.01,
                    # odometry error in yaw angle [rad/rad] [1]
                    #  how fast to increase particle spread when in 0D mode
                    "odometry_std_yaw": 0.01,
                    # minimum particle spread in x and y [m]
                    "min_sample_std_xy": 0.025,
                    # minimum particle spread in yaw angle [rad]
                    "min_sample_std_yaw": 0.025,
                    # initial/maximum particle spread in x and y [m]
                    "max_sample_std_xy": 0.5,
                    # initial/maximum particle spread in yaw angle [rad]
                    "max_sample_std_yaw": 0.5,
                    # threshold for 1D / 2D decision making (minimum average second order gradient)
                    # if worst gradient direction is below this value we go into 1D mode
                    # if both gradient directions are below we may go into 0D mode, depending on disable_threshold
                    # higher values will make it go into 1D / 0D mode earlier
                    "constrain_threshold": 0.1,
                    # threshold for 1D / 2D decision making (with or without orientation)
                    #   higher values will make it go into 1D mode earlier
                    "constrain_threshold_yaw": 0.2,
                    # minimum number of points per update
                    "min_points": 20,
                    # solver update gain, lower gain = more stability / slower convergence
                    "solver_gain": 0.1,
                    # solver update damping, higher damping = more stability / slower convergence
                    "solver_damping": 1000.0,
                    # number of gauss-newton iterations per sample per scan
                    "solver_iterations": 20,
                    # maximum wait for getting transforms [s]
                    "transform_timeout": 0.2,
                    # if to broadcast map frame
                    "broadcast_tf": True,
                    # Scan topic
                    "scan_topic": laserscan_topic,
                    # Initial Pose topic
                    "initialpose": self.info["initialpose_topic"],
                    # Map Topic
                    "map_topic": self.info["map_frame"],
                    # Map Tile topic
                    "map_tile": self.info["map_frame"] + "_tile",
                    # Map Pose topic
                    "map_pose": self.info["map_frame"] + "_pose",
                    # particle_cloud topic
                    "particle_cloud": "particlecloud",
                    # amcl_pose topic
                    "amcl_pose": "amcl_pose",
                }
            },
        )

        return neo_localization2_configuration
