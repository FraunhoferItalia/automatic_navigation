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
from vehicle_recognition import Vehicle, SensorType
from . import VisualizationConfiguratorBase
import yaml
import os


def merge_topic(namespace: str, topic: str) -> str:
    result = topic
    if not topic.startswith("/"):
        result = namespace + "/" + topic if namespace != "/" else "/" + topic
        if not result.startswith("/"):
            result = "/" + result

    return result


class Rviz2Configurator(VisualizationConfiguratorBase):
    def __init__(self, info: Dict[str, Any], vehicle: Vehicle, file: str):
        super().__init__(info, vehicle, file)

    def configure(self) -> bool:
        with open(self.file, "w") as f:
            visualization = self.generate_rviz2_configuration()
            yaml.dump(visualization, f, sort_keys=False)

        return True

    def generate_rviz2_configuration(self):
        panels, window_geometry = self.compose_panels_and_window_geometry()

        visualization_manager = self.compose_visualization_manager()

        return {
            "Panels": panels,
            "Visualization Manager": visualization_manager,
            "Window Geometry": window_geometry,
        }

    def compose_visualization_manager(self):
        fixed_frame = self.vehicle.root.name
        views = {
            "Current": {"Class": "rviz_default_plugins/TopDownOrtho", "Distance": 10.0},
            "Saved": "~",
        }
        grid = {"Class": "rviz_default_plugins/Grid", "Name": "Grid", "Enabled": True}
        tf = {"Class": "rviz_default_plugins/TF", "Enabled": True, "Name": "TF"}

        displays = [grid, tf]

        lasers = []
        for sensor in self.vehicle.sensors:
            if sensor.type is SensorType.LASER:
                lasers += [sensor.name]
        if lasers or "laserscan_topic" in self.info.keys():
            laserscan_topic = lasers[0] if lasers else ""
            if "laserscan_topic" in self.info.keys():
                laserscan_topic = self.info["laserscan_topic"]
            else:
                print(
                    "VISUALIZATION WARNING: Laser topic not specified. First laser name will be taken"
                )
            lasers_displays = [
                {
                    "Class": "rviz_default_plugins/LaserScan",
                    "Topic": {
                        "Depth": 1,
                        "Durability Policy": "Volatile",
                        "History Policy": "Keep Last",
                        "Reliability Policy": "Best Effort",
                        "Value": self.info["laserscan_topic"],
                    },
                    "Color": "170; 0; 0",
                    "Color Transformer": "FlatColor",
                    "Enabled": True,
                    "Size (m)": 0.05,
                }
            ]
            displays += lasers_displays

        if self.info["plan_topic"] != "":
            plan = {
                "Class": "rviz_default_plugins/Path",
                "Enabled": True,
                "Name": "Plan",
                "Topic": {
                    "Depth": 1,
                    "Durability Policy": "Volatile",
                    "Filter size": 10,
                    "History Policy": "Keep Last",
                    "Reliability Policy": "Reliable",
                    "Value": merge_topic(
                        self.info["namespace"], self.info["path_topic"]
                    ),
                },
            }
            displays += [plan]

        if self.info["footprint_topic"] != "":
            plan = {
                "Class": "rviz_default_plugins/Polygon",
                "Enabled": True,
                "Name": "Footprint",
                "Topic": {
                    "Depth": 1,
                    "Durability Policy": "Volatile",
                    "Filter size": 10,
                    "History Policy": "Keep Last",
                    "Reliability Policy": "Reliable",
                    "Value": merge_topic(
                        self.info["namespace"], self.info["footprint_stamped_topic"]
                    ),
                },
            }
            displays += [plan]

        if self.info["map_frame"] != "":
            map = {
                "Class": "rviz_default_plugins/Map",
                "Enabled": True,
                "Name": "Map",
                "Topic": {
                    "Depth": 1,
                    "Durability Policy": "Transient Local",
                    "History Policy": "Keep Last",
                    "Reliability Policy": "Reliable",
                    "Value": merge_topic(
                        self.info["namespace"], self.info["map_frame"]
                    ),
                },
                "Update Topic": {
                    "Depth": 1,
                    "Durability Policy": "Volatile",
                    "History Policy": "Keep Last",
                    "Reliability Policy": "Reliable",
                    "Value": merge_topic(self.info["namespace"], self.info["map_frame"])
                    + "_updates",
                },
            }
            fixed_frame = self.info["map_frame"]
            displays += [map]

        global_options = {"Fixed Frame": fixed_frame}
        return {
            "Class": "",
            "Global Options": global_options,
            "Displays": displays,
            "Tools": [
                {"Class": "rviz_default_plugins/Interact"},
                {"Class": "rviz_default_plugins/MoveCamera"},
                {"Class": "rviz_default_plugins/FocusCamera"},
                {"Class": "rviz_default_plugins/Measure"},
                {
                    "Class": "rviz_default_plugins/SetInitialPose",
                    "Topic": {"Value": self.info["initialpose_topic"]},
                },
                {
                    "Class": "rviz_default_plugins/SetGoal",
                    "Topic": {"Value": self.info["goal_pose_topic"]},
                },
            ],
        }

    def compose_panels_and_window_geometry(
        self,
    ) -> Tuple[Any, Any]:
        other_displays = []
        displays = {
            "Class": "rviz_common/Displays",
            "Name": "Displays",
            "Help Height": 70,
            "Property Tree Widget": {
                "Expanded": ["/Global Options1", "/Status1"] + other_displays
            },
            "Splitter Ratio": 0.65,
        }
        selection = {"Class": "rviz_common/Selection", "Name": "Selection"}
        time = {"Class": "rviz_common/Time", "Name": "Time"}
        views = {"Class": "rviz_common/Views", "Name": "Views"}

        panels = [displays, time, views]
        window_geometry = {
            "Displays": {"collapsed": False},
            "Selection": {"collapsed": False},
            "Time": {"collapsed": False},
            "Views": {"collapsed": True},
            "QMainWindow State": "000000ff00000000fd00000004000000000000015600000379fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003b00000379000000c700fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f00000379fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003b00000379000000a000fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000007360000003efc0100000002fb0000000800540069006d00650100000000000007360000025300fffffffb0000000800540069006d00650100000000000004500000000000000000000004c50000037900000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000",
            "Hide Left Dock": False,
            "Hide Right Dock": False,
        }

        return (panels, window_geometry)
