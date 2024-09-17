# Automatic Navigation

The automatic navigation library is divided into three sub-packages:

- **navigation_configurators**: This package contains configurators for the various developed tools, responsible for generating the necessary configuration files based on the vehicle recognition procedure developed within the `vehicle_recognition` package.
- **navigation_launch_templates**: This package includes the launch files needed to launch the developed tools.
- **navigation_deployment**: This package contains the launch files for the various functionalities provided by the library. It uses both the configurators and the launch templates.

A more detailed explanation of the developed components can be found in the paper currently under review: *"Automatic Vehicle Kinematics Recognition and Deployment of the Navigation for Modular Wheeled Mobile Robots"*.


### Licence
urdf_parser is licensed under the terms of the Apache License 2.0. The project has received financial support by the Horizon 2020 EU Project [CONCERT](https://concertproject.eu/).