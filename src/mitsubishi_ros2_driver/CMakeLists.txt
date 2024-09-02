cmake_minimum_required(VERSION 3.8)
project(mitsubishi_ros2_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED) 

# Include Cpp "include" directory
include_directories(include)

add_library(mitsubishi_hardware_interface SHARED src/mitsubishi_hardware_interface.cpp)
ament_target_dependencies(mitsubishi_hardware_interface PUBLIC rclcpp std_msgs sensor_msgs pluginlib hardware_interface rclcpp_lifecycle)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Cpp executables
install(TARGETS
mitsubishi_hardware_interface
  DESTINATION lib/${PROJECT_NAME}  
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/mitsubishi_tcp.py
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface mitsubishi_ros2_control.xml)

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

ament_package()
