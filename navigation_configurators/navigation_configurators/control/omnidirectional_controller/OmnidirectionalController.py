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
from typing import List, Dict, Any, Tuple
from vehicle_recognition import Vehicle, WheelType, VehicleKinematic
from .. import ControlConfiguratorBase
import yaml


class OmnidirectionalControllerConfigurator(ControlConfiguratorBase):
    def __init__(self, info: Dict[str, Any], vehicle: Vehicle, file: str):
        super().__init__(info, vehicle, file)

    def configure(self) -> bool:
        try:
            configuration = yaml.safe_load(open(self.file, "r"))
        except yaml.YAMLError:
            print(f"CONTROL ERROR: failed to open file {self.file}")
            return False

        with open(self.file, "w") as f:
            for kinematic in self.vehicle.kinematics:
                for kinematic_configuration in kinematic.configurations:
                    controller = self.generate_controller_configuration(
                        kinematic_configuration
                    )
                    configuration[controller[0]] = controller[1]
                    configuration["controller_manager"]["ros__parameters"][
                        controller[0]
                    ] = {"type": "omnidirectional_controller/OmnidirectionalController"}

            yaml.dump(configuration, f, sort_keys=False)

        return True

    def generate_controller_configuration(
        self, configuration: VehicleKinematic.Configuration
    ) -> Tuple[str, Any]:
        wheels_names: List[str] = []
        wheels_joints: Dict[str, Dict[str, str]] = {}
        wheels_joint_interfaces: Dict[str, Dict[str, List[str]]] = {}
        limits: Dict[str, Dict[str, List[str]]] = {}
        parameters: Dict[str, Dict[str, str]] = {}
        odometry: Dict[str, Any] = {
            "velocity_variances": [0.1, 0.1, 0.1],
            "publish_position": False,
            "header_frame": self.info["odom_frame"],
            "child_frame": self.info["control_frame"],
            "topic": self.info["odom_topic"],
        }
        for wheel in self.vehicle.wheels:
            wheels_names += [wheel.name]
            wheel.reset()
            parameters[wheel.name] = wheel.compute_params(configuration.tf)
            parameters[wheel.name]["radius"] = wheel.radius
            if wheel.type is WheelType.STEERING:
                if wheel.params["d"] == 0:
                    wheels_joints[wheel.name] = {
                        "type": "steering",
                        "drive": wheel.joints[1].name,
                        "steer": wheel.joints[0].name,
                    }
                    wheels_joint_interfaces[wheel.joints[0].name] = {
                        "command": ["position"],
                        "state": ["position"],
                    }
                    wheels_joint_interfaces[wheel.joints[1].name] = {
                        "command": ["velocity"],
                        "state": ["velocity"],
                    }

                    for joint in wheel.joints:
                        interfaces = []
                        limits_dict = {}
                        if joint.limits.effort != float("inf"):
                            interfaces += ["effort"]
                            limits_dict["effort"] = {"upper": joint.limits.effort}
                        if joint.limits.velocity != float("inf"):
                            interfaces += ["velocity"]
                            limits_dict["velocity"] = {"upper": joint.limits.velocity}
                        if joint.limits.upper != float(
                            "inf"
                        ) and joint.limits.lower != float("inf"):
                            interfaces += ["position"]
                            limits_dict["position"] = {
                                "upper": joint.limits.upper,
                                "lower": joint.limits.lower,
                                "is_angle": True,
                            }
                        limits[joint.name] = limits_dict
                        limits[joint.name]["interfaces"] = interfaces

                else:
                    wheels_joints[wheel.name] = {
                        "type": "castor",
                        "drive": wheel.joints[1].name,
                        "steer": wheel.joints[0].name,
                    }
                    wheels_joint_interfaces[wheel.joints[0].name] = {
                        "command": ["velocity"],
                        "state": ["position", "velocity"],
                    }
                    wheels_joint_interfaces[wheel.joints[1].name] = {
                        "command": ["velocity"],
                        "state": ["velocity"],
                    }
            elif wheel.type is WheelType.FIXED:
                wheels_joints[wheel.name] = {
                    "type": "fixed",
                    "drive": wheel.joints[0].name,
                }
                wheels_joint_interfaces[wheel.joints[0].name] = {
                    "command": ["velocity"],
                    "state": ["velocity"],
                }
        controller = (
            str(configuration.name),
            {
                "ros__parameters": {
                    "wheels": wheels_names,
                    "wheel_joints": wheels_joints,
                    "joint_interfaces": wheels_joint_interfaces,
                    "limits": limits,
                    "parameters": parameters,
                    "odometry": odometry,
                    "cmd_vel_topic": self.info["controller_cmd_vel_topic"],
                    "differential_cmd_vel": (
                        True if not "omnidirectional" in configuration.name else False
                    ),
                }
            },
        )

        return controller
