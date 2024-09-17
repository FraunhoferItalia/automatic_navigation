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
from navigation_configurators import ConfiguratorBase
from abc import abstractmethod
from vehicle_recognition import Vehicle
import yaml


class ControlConfiguratorBase(ConfiguratorBase):
    def __init__(self, info: str, vehicle: Vehicle, file: str):
        super().__init__(info, vehicle, file)
        with open(file, "r+") as f:
            try:
                controllers = yaml.safe_load(f)
            except yaml.YAMLError:
                print(f"CONTROL ERROR: Failed to load file {file} ")
                return
            default_ros_parameters = {
                "update_rate": 1000,
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                },
            }
            try:
                for key, value in default_ros_parameters.items():
                    controllers["controller_manager"]["ros__parameters"][key] = value
            except TypeError:
                controllers = {
                    "controller_manager": {"ros__parameters": default_ros_parameters}
                }

            yaml.dump(controllers, f, sort_keys=False)

    @abstractmethod
    def configure(self) -> bool:
        return False
