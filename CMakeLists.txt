cmake_minimum_required(VERSION 3.5)
project(drone_state_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pybind11 REQUIRED)

# Add C++ executable
add_executable(drone_missions src/drone_missions.cpp)

message(STATUS "Pybind11 Include Directories: ${pybind11_INCLUDE_DIRS}")
# Include directories
include_directories(
  include
  ${pybind11_INCLUDE_DIRS}
)

# Linking
ament_target_dependencies(drone_missions
  rclcpp
  geometry_msgs
)

# Add Pybind11 module
pybind11_add_module(drone_missions_py MODULE src/drone_missions.cpp)

# Link Pybind11 module with dependencies
target_link_libraries(drone_missions_py PRIVATE
  rclcpp
  geometry_msgs
)

# Install executables
install(TARGETS
  drone_missions
  drone_missions_py
  DESTINATION lib/${PROJECT_NAME}
)

# Install package.xml
ament_package()
