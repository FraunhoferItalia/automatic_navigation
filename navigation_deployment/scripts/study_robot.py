#!/usr/bin/python3
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
import argparse

from navigation_deployment.study_robot import (
    study_robot_core,
)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf", help="urdf file location")
    parser.add_argument("config_folder", help="configuration folder location")
    parser.add_argument(
        "automatic_navigation_file", default="", help="configuration folder location"
    )
    args = parser.parse_args()

    urdf = args.urdf
    config_folder = args.config_folder
    automatic_navigation_file = args.automatic_navigation_file
    study_robot_core(urdf, config_folder, automatic_navigation_file)


if __name__ == "__main__":
    main()
