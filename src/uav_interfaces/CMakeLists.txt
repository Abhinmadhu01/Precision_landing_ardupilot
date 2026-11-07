cmake_minimum_required(VERSION 3.8)
project(uav_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate bindings for Python and C++
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToWaypoint.action"
  "action/PrecisionLand.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()