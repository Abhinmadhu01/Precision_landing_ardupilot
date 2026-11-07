cmake_minimum_required(VERSION 3.8)
project(autonomous_flight_stack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(uav_interfaces REQUIRED)

# Install Python scripts as executable ROS 2 nodes
install(PROGRAMS
  scripts/mission_executive.py
  scripts/navigation_server.py
  scripts/landing_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()