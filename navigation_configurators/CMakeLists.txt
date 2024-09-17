cmake_minimum_required(VERSION 3.8)
project(navigation_configurators)

find_package(ament_cmake REQUIRED)
find_package(vehicle_recognition REQUIRED)
find_package(fhi_ros2 REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})
        
ament_package()
