cmake_minimum_required(VERSION 3.8)
project(navigation_deployment)

find_package(ament_cmake REQUIRED)
find_package(fhi_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vehicle_recognition REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/automatic_navigation_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch scripts
        DESTINATION share/${PROJECT_NAME})

ament_package()
