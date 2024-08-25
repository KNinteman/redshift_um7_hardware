# redshift_um7_hardware
A ROS2_Control Hardware Interface for interfacing with a UM7 inertial measurement unit (IMU) via serial communication.

## Overview
This package provides a ROS2 Control hardware interface designed to interact with a UM7 IMU through a serial port. It facilitates integration with the ros2_control framework, enabling access to orientation, angular velocity, linear acceleration, magnetic field, and Euler angles from the UM7 sensor. The interface supports configuring the IMU's communication settings and processing sensor data based on different output axis conventions (ENU, robot frame, and default).

This package is a modified version of the [um7 driver library](https://github.com/ros-drivers/um7/tree/ros2) written by Clearpath robotics. The modifications have been made in order to make the driver compatible with ros2_control.

## Getting Started
To begin using redshift_um7_hardware, follow these steps:

1. Clone this repository into your ROS 2 workspace:
   ```
   cd {your_ros2_ws}/src
   git clone https://github.com/KNinteman/redshift_um7_hardware.git
   ```
2. Navigate to your ROS 2 workspace root directory and build the package:
   ```
   cd ..
   colcon build --packages-select redshift_um7_hardware
   ```
3. Ensure that the serial package is also built. You can use the following link for reference: [serial](https://github.com/wjwwood/serial/tree/ros2?tab=MIT-1-ov-file).

## Configuration
Before using the package, it needs to be properly configured. Here are some steps to consider:

- Set up a ros2_control tag in your robot's URDF according to the template provided below:
```
<ros2_control name="UM7Interface" type="sensor">
   <hardware>
      <plugin>redshift_um7_hardware/RedshiftUm7Hardware</plugin>
      <param name="serial_port"></param> <!--string-->
      <param name="baud_rate"></param> <!--int-->
      <param name="update_rate"></param> <!--int-->
      <param name="frame_id"></param> <!--string-->
      <param name="mag_updates"></param> <!--bool-->
      <param name="quat_mode"></param> <!--bool-->
      <param name="zero_gyros"></param> <!--bool-->
      <param name="tf_ned_to_enu"></param> <!--bool-->
      <param name="orientation_in_robot_frame"></param> <!--bool-->
  </hardware>

  <sensor name="redshift_um7_hardware">
      <state_interface name="orientation.x"/>
      <state_interface name="orientation.y"/>
      <state_interface name="orientation.z"/>
      <state_interface name="orientation.w"/>
      <state_interface name="angular_velocity.x"/>
      <state_interface name="angular_velocity.y"/>
      <state_interface name="angular_velocity.z"/>
      <state_interface name="linear_acceleration.x"/>
      <state_interface name="linear_acceleration.y"/>
      <state_interface name="linear_acceleration.z"/>
      <state_interface name="magnetic_field.x"/>
      <state_interface name="magnetic_field.y"/>
      <state_interface name="magnetic_field.z"/>
      <state_interface name="vector.x"/>
      <state_interface name="vector.y"/>
      <state_interface name="vector.z"/>
   </sensor>
</ros2_control>
```
Ensure that you fill in the necessary parameters such as serial_port, baud_rate, etc., based on your specific hardware setup.

## Troubleshooting
If you encounter any issues during installation or configuration, consider the following troubleshooting tips:

Double-check your serial communication settings to ensure they match those specified in the configuration files.
Verify that all dependencies, including the serial package, are properly installed and built.
Refer to the ROS 2 documentation or community forums for additional support if needed.

## Acknowledgments
I would like to acknowledge the contributions of the ROS 2 community, the developers of the ros2_control and serial packages and Clearpath Robotics for their support.
