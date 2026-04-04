cmake_minimum_required(VERSION 3.8)
project(kinova_apps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# finds
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# srvs
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveAndCapture.srv"
  DEPENDENCIES geometry_msgs sensor_msgs
)

# install python node
install(PROGRAMS
  kinova_apps/robot_service_node.py
  scripts/test_service_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
