# navigation_configurators

This package contains the configurators developed for the various navigation tools. The configurators are grouped into 7 categories:

- **Control**
- **Filtering**
- **Localization**
- **Mapping**
- **Planning**
- **Simulation**
- **Visualization**

At least one configurator is available for each category.

The configurators must be developed as children of their respective base class according to the group they belong to. All configurators rely on the data extracted from the vehicle recognition procedure and its corresponding class, which is implemented within the `vehicle_recognition` package.


### Licence
urdf_parser is licensed under the terms of the Apache License 2.0. The project has received financial support by the Horizon 2020 EU Project [CONCERT](https://concertproject.eu/).