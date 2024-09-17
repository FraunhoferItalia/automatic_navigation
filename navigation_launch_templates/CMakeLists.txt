cmake_minimum_required(VERSION 3.8)
project(navigation_launch_templates)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(fhi_ros2 REQUIRED)
find_package(fhi_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

install(PROGRAMS scripts/control/switch_kinematic_manager.py
        DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/control/joy_switch_kinematic.py
        DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/control/cmd_vel_converter.py
        DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/planning/footprint_manager.py
        DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/planning/nav2_plan_transformer.py
        DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/planning/nav2_switch_manager.py
        DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch scripts
        DESTINATION share/${PROJECT_NAME})

        
ament_package()
