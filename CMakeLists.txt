cmake_minimum_required(VERSION 3.8)
project(redshift_um7_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)

# Add compile directive for shared library
add_library(redshift_um7_hardware 
  SHARED
  src/redshift_um7_comms.cpp
  src/redshift_um7_hardware.cpp
  src/redshift_um7_registers.cpp
)

target_include_directories(
  redshift_um7_hardware
  PRIVATE
  include
)

install(DIRECTORY
  src
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS redshift_um7_hardware
  EXPORT redshift_um7_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/redshift_um7_hardware
)

# Export pluginlib description file
pluginlib_export_plugin_description_file(hardware_interface redshift_um7_hardware.xml)


ament_export_libraries(
  redshift_um7_hardware
)

# Add ament dependencies
ament_target_dependencies(redshift_um7_hardware
  hardware_interface
  pluginlib
  controller_manager
  serial
  rclcpp
  rclcpp_lifecycle
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  controller_manager
  serial
  rclcpp
  rclcpp_lifecycle
)

ament_target_dependencies(redshift_um7_hardware
  geometry_msgs
  rclcpp
  sensor_msgs
  serial
  std_msgs
)

ament_package()
