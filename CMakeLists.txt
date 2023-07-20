cmake_minimum_required(VERSION 3.8)
project(drone_state_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(PYBIND11_PYTHON_VERSION 3.6)

# Find dependencies
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
set(x ../crazyflie_cpp/cmake/crazyflie_cpp-config.cmake)


find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pybind11 REQUIRED)

# Add C++ executable
add_executable(drone_missions src/drone_missions.cpp)
#add_executable(drone_py src/drone_missions_py.cpp)



message(pybind11_FOUND)
# Include directories
include_directories(
  #include
  x
  ${pybind11_INCLUDE_DIRS}

  
)

# Linking
ament_target_dependencies(drone_missions
  "rclcpp"
  "std_msgs"
  "nav_msgs"
  "geometry_msgs"
)

# Add Pybind11 module
pybind11_add_module(drone_missions_py src/drone_missions_py.cpp)

# Link the drone_ws target with the crazyfile_cpp target

#pybind11_add_module(drone_missions_py MODULE src/drone_missions.cpp)

target_include_directories(drone_missions PRIVATE   ${PYBIND11_INCLUDE_DIRS}/include ${CRAZYFILE_INCLUDE_DIRS}/include )


# Install executables
install(TARGETS
  drone_missions
  drone_missions_py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install package.xml
ament_package()


# Install Python scripts
install(PROGRAMS
  scripts/drone_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)