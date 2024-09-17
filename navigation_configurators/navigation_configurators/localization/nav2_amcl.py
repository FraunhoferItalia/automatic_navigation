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


class Nav2AmclConfigurator(LocalizationConfiguratorBase):
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
            if not lasers and not pointclouds:
                print(
                    "LOCALIZATION WARNING: No laserscanners found, Nav2 Amcl tool requires a laserscanner topic."
                )

            localization = self.generate_configuration(lasers, pointclouds)
            try:
                configuration["**/"][localization[0]] = localization[1]
            except KeyError:
                configuration.update({"**/": {localization[0]: localization[1]}})
            except TypeError:
                configuration = {"**/": {localization[0]: localization[1]}}
            yaml.dump(configuration, f, sort_keys=False)
        return True

    def generate_configuration(
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
                return ("nav2_amcl_node", {"ros__parameters": {}})
        configuration = (
            "nav2_amcl_node",
            {
                "ros__parameters": {
                    "alpha1": 0.2,
                    "alpha2": 0.2,
                    "alpha3": 0.2,
                    "alpha4": 0.2,
                    "alpha5": 0.2,
                    "base_frame_id": self.vehicle.root.name,
                    "beam_skip_distance": 0.5,
                    "beam_skip_error_threshold": 0.9,
                    "beam_skip_threshold": 0.3,
                    "do_beamskip": False,
                    "global_frame_id": self.info["map_frame"],
                    "lambda_short": 0.1,
                    "laser_likelihood_max_dist": 2.0,
                    "laser_max_range": 100.0,
                    "laser_min_range": -1.0,
                    "laser_model_type": "likelihood_field_prob",
                    "max_beams": 60,
                    "max_particles": 10000,
                    "min_particles": 500,
                    "odom_frame_id": self.info["odom_frame"],
                    "pf_err": 0.05,
                    "pf_z": 0.99,
                    "recovery_alpha_fast": 0.1,
                    "recovery_alpha_slow": 0.001,
                    "resample_interval": 1,
                    "robot_model_type": "nav2_amcl::OmniMotionModel",
                    "save_pose_rate": 0.5,
                    "sigma_hit": 0.2,
                    "tf_broadcast": True,
                    "transform_tolerance": 1.0,
                    "update_min_a": 0.1,
                    "update_min_d": 0.1,
                    "z_hit": 0.5,
                    "z_max": 0.05,
                    "z_rand": 0.5,
                    "z_short": 0.05,
                    "scan_topic": laserscan_topic,
                    "map_topic": self.info["map_frame"],
                    "set_initial_pose": True,
                    "always_reset_initial_pose": False,
                    "first_map_only": False,
                    "initial_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                }
            },
        )

        return configuration
