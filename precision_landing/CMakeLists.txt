cmake_minimum_required(VERSION 3.8)
project(precision_landing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/precision_landing_ardupilot.py
  src/person_tracking.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
