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
import sys
from ament_index_python.packages import get_package_share_directory

sys.path.append(
    f"{get_package_share_directory('navigation_configurators')}/local/lib/python3.10/dist-packages/navigation_configurators/"
)

from typing import Dict, Any
from fhi_ros2.launch import load_yaml
from abc import ABC, abstractmethod
from vehicle_recognition import Vehicle


def parse_configurator_info(file_path_or_dict) -> Dict[str, Any]:
    default = load_yaml("navigation_configurators/config/default.yaml")[
        "navigation_configurators"
    ]
    info = {}
    try:
        if isinstance(file_path_or_dict, str):
            info = load_yaml(file_path_or_dict)["navigation_configurators"]
        elif isinstance(file_path_or_dict, dict):
            info = file_path_or_dict
        else:
            raise KeyError
        for [key, value] in default.items():
            try:
                info[key]
            except KeyError:
                print(
                    "Configurators Info Key: "
                    + key
                    + f" not given. Default will be used -> {value}"
                )
                info[key] = value

    except (FileNotFoundError, KeyError) as e:
        print(
            "No automatic navigation configurators info file given, default will be used."
        )
        info = default

    return info


class ConfiguratorBase(ABC):
    vehicle: Vehicle
    file: str
    config: Dict[str, Any]

    def __init__(self, info: Dict[str, Any], vehicle: Vehicle, file: str):
        self.vehicle = vehicle
        self.file = file
        self.info = info
        try:
            open(file, "x")
        except FileExistsError:
            pass

    @abstractmethod
    def configure(self):
        pass


import importlib

from navigation_configurators.control import ControlConfiguratorBase
from navigation_configurators.filtering import FilteringConfiguratorBase
from navigation_configurators.localization import LocalizationConfiguratorBase
from navigation_configurators.mapping import MappingConfiguratorBase
from navigation_configurators.planning import PlanningConfiguratorBase
from navigation_configurators.simulation import SimulationConfiguratorBase
from navigation_configurators.visualization import VisualizationConfiguratorBase

BASE_CLASSES = {
    "control": ControlConfiguratorBase,
    "filtering": FilteringConfiguratorBase,
    "localization": LocalizationConfiguratorBase,
    "mapping": MappingConfiguratorBase,
    "planning": PlanningConfiguratorBase,
    "simulation": SimulationConfiguratorBase,
    "visualization": VisualizationConfiguratorBase,
}


def get_configurator(configurator_type: str, configurator_name: str):
    try:
        module = importlib.import_module(
            f"navigation_configurators.{configurator_type}.{configurator_name.replace('/','.')}"
        )
    except ModuleNotFoundError as e:
        print(e)
        return

    instances = []
    for class_name in dir(module):
        cls = getattr(module, class_name)
        if not isinstance(cls, type):
            continue
        if (
            issubclass(cls, BASE_CLASSES[configurator_type])
            and not cls.__name__ == BASE_CLASSES[configurator_type].__name__
        ):
            instances.append(cls)

    return instances[0]
