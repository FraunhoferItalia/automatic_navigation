#
# Copyright 2022-2024 Fraunhofer Italia Research
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
import os
import yaml
from typing import Dict, Any
from vehicle_recognition import Vehicle, VehicleKinematicType
from vehicle_recognition.utils import get_euler_angles

from navigation_deployment import parse_yaml
from fhi_ros2.launch import get_absolute_file_path
from navigation_configurators import get_configurator, parse_configurator_info


def read_automatic_navigation_file(
    automatic_navigation_file_path: str,
) -> Dict[str, Any]:
    default_configurations = parse_yaml(
        get_absolute_file_path("navigation_deployment/config/default.yaml")
    )
    try:
        automatic_navigation_configurations = parse_yaml(automatic_navigation_file_path)
        for key, value in default_configurations["automatic_navigation"].items():
            try:
                if not type(
                    automatic_navigation_configurations["automatic_navigation"][key]
                ) is type(value):
                    raise KeyError
            except KeyError:
                print(
                    "The fundamental automatic navigation section "
                    + key
                    + " is not defined, default will be used."
                )
                automatic_navigation_configurations["automatic_navigation"][key] = value
    except (FileNotFoundError, KeyError, TypeError) as e:
        print(
            "No automatic navigation configuration file created, default will be used. ",
            e,
        )
        automatic_navigation_configurations = default_configurations
    except yaml.YAMLError as e:
        print(f"Yaml passed wrongly defined, error: {e}")

    return automatic_navigation_configurations


def setup_vehicle(
    urdf: str,
    vehicle_configurations: Dict[str, Any],
    ignore_wheels_radius: bool = False,
) -> Vehicle:
    vehicle = Vehicle(urdf, ignore_wheels_radius)
    if vehicle.root is None:
        print("Error: No vehicle found!")
        return None
    if "wheel_radius" in vehicle_configurations.keys():
        for wheel in vehicle.wheels:
            wheel.radius = vehicle_configurations["wheel_radius"]

    vehicle_configurations["root_frame"] = vehicle.root.name
    vehicle.study_kinematics(autonomous=True)

    try:
        desired_kinematic_type = vehicle_configurations["chosen_kinematic"]["type"]
    except KeyError:
        desired_kinematic_type = ""
    if desired_kinematic_type != "":
        try:
            desired_kinematic_name = vehicle_configurations["chosen_kinematic"]["name"]
        except KeyError:
            desired_kinematic_name = ""
        try:
            use_defined = False
            kinematic_type = VehicleKinematicType(desired_kinematic_type)
            use_defined = vehicle.choose_kinematic_configuration(
                VehicleKinematicType(desired_kinematic_type), desired_kinematic_name
            )
        except ValueError:
            if not use_defined:
                print(f"Wrongly defined kinematic configuration desired")
                print(
                    f"Configuring: {vehicle.chosen_kinematic[0]} - {vehicle.chosen_kinematic[1].name}"
                )
    possible_kinematics: Dict[Dict[str, Any]] = {}
    for kinematic in vehicle.kinematics:
        for configuration in kinematic.configurations:
            x = float(configuration.tf[0, 3])
            y = float(configuration.tf[1, 3])
            theta = float(get_euler_angles(configuration.tf)[2])

            possible_kinematic = {
                "type": kinematic.type.__str__(),
                "control_frame": {"x": x, "y": y, "theta": theta},
            }
            possible_kinematics[configuration.name] = possible_kinematic

    vehicle_configurations["possible_kinematics"] = possible_kinematics

    vehicle_configurations["chosen_kinematic"] = {
        "type": vehicle.chosen_kinematic[0].__str__(),
        "name": vehicle.chosen_kinematic[1].name,
    }

    return vehicle


def study_robot_core(
    urdf,
    config_folder,
    automatic_navigation_file_path="",
    ignore_wheels_radius: bool = False,
):
    if not os.path.exists(config_folder):
        os.mkdir(config_folder)
    if automatic_navigation_file_path == "":
        automatic_navigation_file_path = os.path.join(
            config_folder, "automatic_navigation.yaml"
        )
    configurations = read_automatic_navigation_file(automatic_navigation_file_path)
    vehicle = setup_vehicle(
        urdf, configurations["automatic_navigation"]["vehicle"], ignore_wheels_radius
    )
    if vehicle is None:
        print("Failed to get the vehicle description")
        return None

    automatic_navigation_file_output_path = os.path.join(
        os.path.join(config_folder, "automatic_navigation.yaml")
    )
    yaml.safe_dump(
        configurations,
        open(automatic_navigation_file_output_path, "w"),
        sort_keys=False,
    )

    info = {}
    try:
        file = configurations["automatic_navigation"]["configurators_info"]["file"]
        try:
            package = configurations["automatic_navigation"]["configurators_info"][
                "package"
            ]
            file = package + "/" + file
        except KeyError:
            pass

        info = parse_configurator_info(file)
    except KeyError:
        configurators_info = {}
        try:
            configurators_info = configurations["automatic_navigation"][
                "configurators_info"
            ]
        except KeyError:
            pass
        info = parse_configurator_info(configurators_info)

    print("\n")
    configurators = {}
    for configurator_type, configuration in configurations["automatic_navigation"][
        "tools"
    ].items():
        configurator = get_configurator(
            configurator_type,
            configuration["tool"],
        )
        if not configurator is None:
            configurators[configurator_type] = configurator(
                info,
                vehicle,
                config_folder + "/" + configuration["config_file"],
            )

    for type, configurator in configurators.items():
        print(f"- {type.upper()}: {configurator.__class__.__name__}")
        if not configurator.configure():
            print("CONFIGURATION ERROR: Failed to configure requested tools")
            return None
        print("\n")

    # allows user to delete and modify files
    import subprocess

    process = subprocess.Popen(["chmod", "a+rwX", config_folder])
    process.communicate()

    return automatic_navigation_file_output_path
