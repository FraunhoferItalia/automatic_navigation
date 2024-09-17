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
from . import MappingConfiguratorBase
from typing import Dict, Any, List
from vehicle_recognition import Vehicle, SensorType
import yaml


class SlamToolboxConfigurator(MappingConfiguratorBase):
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
                    "MAPPING WARNING: No laserscanners found, SlamToolbox tool requires a laserscanner topic."
                )

            mapping = self.generate_slam_toolbox_configuration(lasers, pointclouds)
            try:
                configuration["**/"][mapping[0]] = mapping[1]
            except KeyError:
                configuration.update({"**/": {mapping[0]: mapping[1]}})
            except TypeError:
                configuration = {"**/": {mapping[0]: mapping[1]}}
            yaml.dump(configuration, f, sort_keys=False)

        return True

    def generate_slam_toolbox_configuration(
        self, lasers: List[str] = [], pointclouds: List[str] = []
    ):
        if lasers:
            laserscan_topic = lasers[0]

        try:
            laserscan_topic = self.info["laserscan_topic"]
        except KeyError:
            if lasers:
                print(
                    "MAPPING WARNING: Laser merge topic not specified. First laser name will be taken"
                )
            else:
                print(
                    "MAPPING ERROR: Laser topic not specified. Slam toolbox works only with a laser scanner topic"
                )
                return ("slam_toolbox_node", {"ros__parameters": {}})

        slam_toolbox_configuration = (
            "slam_toolbox_node",
            {
                "ros__parameters": {
                    "solver_plugin": "solver_plugins::CeresSolver",
                    "ceres_linear_solver": "SPARSE_NORMAL_CHOLESKY",
                    "ceres_preconditioner": "SCHUR_JACOBI",
                    "ceres_trust_strategy": "LEVENBERG_MARQUARDT",
                    "ceres_dogleg_type": "TRADITIONAL_DOGLEG",
                    "ceres_loss_function": "None",
                    "odom_frame": self.info["odom_frame"],
                    "map_frame": self.info["map_frame"],
                    "map_name": self.info["map_frame"],
                    "base_frame": self.vehicle.root.name,
                    "scan_topic": laserscan_topic,
                    "mode": "mapping",
                    "debug_logging": False,
                    "throttle_scans": 1,
                    "transform_publish_period": 0.02,
                    "map_update_interval": 5.0,
                    "resolution": 0.05,
                    "max_laser_range": 5.0,
                    "minimum_time_interval": 0.5,
                    "transform_timeout": 0.2,
                    "tf_buffer_duration": 30.0,
                    "stack_size_to_use": 40000000,
                    "enable_interactive_mode": True,
                    # General Parameters
                    "use_scan_matching": True,
                    "use_scan_barycenter": True,
                    "minimum_travel_distance": 0.5,
                    "minimum_travel_heading": 0.5,
                    "scan_buffer_size": 10,
                    "scan_buffer_maximum_scan_distance": 3.0,
                    "link_match_minimum_response_fine": 0.1,
                    "link_scan_maximum_distance": 1.5,
                    "loop_search_maximum_distance": 3.0,
                    "do_loop_closing": True,
                    "loop_match_minimum_chain_size": 10,
                    "loop_match_maximum_variance_coarse": 3.0,
                    "loop_match_minimum_response_coarse": 0.35,
                    "loop_match_minimum_response_fine": 0.45,
                    # Correlation Parameters - Correlation Parameters
                    "correlation_search_space_dimension": 0.5,
                    "correlation_search_space_resolution": 0.01,
                    "correlation_search_space_smear_deviation": 0.1,
                    # Correlation Parameters - Loop Closure Parameters
                    "loop_search_space_dimension": 8.0,
                    "loop_search_space_resolution": 0.05,
                    "loop_search_space_smear_deviation": 0.03,
                    # Scan Matcher Parameters
                    "distance_variance_penalty": 0.5,
                    "angle_variance_penalty": 1.0,
                    "fine_search_angle_offset": 0.00349,
                    "coarse_search_angle_offset": 0.349,
                    "coarse_angle_resolution": 0.0349,
                    "minimum_angle_penalty": 0.9,
                    "minimum_distance_penalty": 0.5,
                    "use_response_expansion": True,
                }
            },
        )

        return slam_toolbox_configuration
